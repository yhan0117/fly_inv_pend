% Flying Inverted Pendulum with model predicitve control
clc; clear; clear global; close all;

trial = 1; 
record = 1; 
controller = 'mpc';

%% Parameters
%   p = physical parameters
%   r = recording specs

% load test case
load("test cases/trial" + trial + ".mat")

% load precalculat5ed functions 
load("derivation/dynamics.mat")
k.pos = matlabFunction(subs(k.pos, param, p.param));

% initial states
z0 = p.z0;

%% Control
% change to the controller folder
addpath(append(pwd,'\',controller))
zt = @(t) [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0]'; p.zt = zt; 

if strcmp(controller,'lqr') 
    % control parameters
    A = double(subs(k.A, param, p.param));
    B = double(subs(k.B, param, p.param));
    c.Q = diag([.05 .05 .8 .1 .1 eps 5 5 .2 .2 .2 .1 .1 .05 .4 .4]);
    c.R = diag([.3 .3 .3 .3]);
    c.K = lqr(A,B,c.Q,c.R);
    
    % feedback control law u = f(t,z)
    u = @(t,z) control(t,z,p,c);
    global t_prev input u_prev;  
    t_prev = -inf;
    input = [];
    u_prev = 0;

    % control loop latency
    c.dt_p = 2/62;  % 2000 Hz

elseif strcmp(controller,'mpc')
    % MPC optimization params
    % 1 step control horizon
    N = 60;  c.N = N;       % predicition horizon 
    t_pred = 2; c.t_pred = t_pred; % prediction time span
    c.dt_p = t_pred/N;      % prediction step size

    % options for NLP Solver
    options = optimoptions('fmincon','TolX', 1e-7, 'TolFun', 1e-7, 'TolCon', 1e-9, 'MaxIterations', 1000,'MaxFunEvals',100000,...
        'DiffMinChange',1e-5, 'GradObj','off', 'GradConstr','off',...
        'DerivativeCheck', 'off', 'Display', 'final', 'Algorithm','sqp', 'UseParallel', true);

    % Cost matrices
    c.F = 1+(1.2*(0:N-1)/N).^16;    %    weight for each break point
    c.Q = diag([5 5 5 5 5 0.1 20 20 .5 .5 .7 1 1 1 4 4]);  % tracking error cost
    c.R = diag([0.1 0.5 0.5 0.1]);           % actuation effort

    % global variable to keep track of simulated control loop time and input 
    % during iterated call of ode45
    global dv_prev t_prev output input u_prev;  
    t_prev = -inf;
    output = [];
    dv_prev = zeros(20*N,1); % <-- first initial guess of the optimization solver
    input = [];
    u_prev = 0;

    load("opt.mat")
    for i=1:N
        dv_prev(16*(i-1)+1:16*i) = ZZ(i,:);  % linear interpolation 
        dv_prev(16*N+4*(i-1)+1:16*N+4*i) = IN(i,:);
    end

    % feedback controller u = f(t,z)
    u = @(t,z)control(t,z,p,c,k,options);
end

%% Simultation
disp("Producing simulation")
options = odeset('RelTol',1e-9,'AbsTol',1e-9);
[~, z] = ode45(@(t,z)dynamics(t,z,u,p,c,k), r.t_s, z0, options);


%% Save data
if record
    if ~exist(controller + "\data", 'dir')
       mkdir(controller + "\data")    % make sure directory exist
    end
    save(controller + "\data\trial" + trial + ".mat", 'z', 'z0', 'u', 'c','p')
end


%% Plot 
% if record
%     figure(1); clf; hold on;
%     plotCartPole(t_s,z,u); % <-- plot results
%     set(gcf,"WindowState",'maximized')
%     saveas(gcf, path + "\plots\trial" + trial + ".jpg") 
% end  


%% Animation
filename = controller + "\animations\trial" + trial + ".gif";
if record     
    if ~exist(controller + "\animations", 'dir')
        mkdir(controller + "\animations") % make sure directory exist
    elseif exist(filename, "file") 
        delete(filename)    % clear existing file
    end
end

% Create and clear a figure
figure(2); clf;
set(gcf,"position", [0,0,900,600])  % set window size
movegui(gcf, 'center');             % center animation
view(-25,25);                         % initial view angle, adjustable

animate(z,p,r,k,record,filename)  % <-- produce animation

disp("Done!!")
close all


