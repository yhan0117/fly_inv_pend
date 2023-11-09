% Flying Inverted Pendulum with model predicitve control
clc; clear; clear global; close all;

trial = 1;
record = 0; 
controller = 'lqr';

%% Parameters
%   p = physical parameters
%   r = recording specs

% load test case
load("test cases/trial" + trial + ".mat")

% load precalculat5ed functions 
load("derivation/dynamics.mat")
k.pos = matlabFunction(subs(k.pos, param, p.param));

% initial states
z0 = [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0]';

% control loop latency
p.t_l = 0.005;  % 200 Hz

% time and global variables
global t_prev input u_prev;  
t_prev = -inf;
input = [];
u_prev = 0;


%% Control
% change to the controller folder
addpath(append(pwd,'\',controller))

% control parameters
A = double(subs(k.A, param, p.param));
B = double(subs(k.B, param, p.param));
c.Q = diag([.05 .05 .8 .1 .1 eps 5 5 .2 .2 .2 .1 .1 .05 .4 .4]);
c.R = diag([.3 .3 .3 .3]);
c.K = lqr(A,B,c.Q,c.R);

% feedback control law u = f(t,z)
u = @(t,z) control(t,z,p,c);


%% Simultation
disp("Producing simulation")
options = odeset('RelTol',1e-6,'AbsTol',1e-6);
[~, z] = ode45(@(t,z)dynamics(t,z,u,p,k), r.t_s, z0, options);


%% Save data
if record
    if ~exist("data", 'dir')
       mkdir("data")    % make sure directory exist
    end
    save(controller + "\data\trial" + trial + ".mat", 'z', 'z0', 'u', 'c')
end


%% Plot 
% if record
%     figure(1); clf; hold on;
%     plotCartPole(t_s,z,u); % <-- plot results
%     set(gcf,"WindowState",'maximized')
%     saveas(gcf, path + "\plots\trial" + trial + ".jpg") 
% end


%% Animation
filename = "animations\trial" + trial + ".gif";
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
view(90,0);                         % initial view angle, adjustable

animate(z,p,r,k,record,filename)  % <-- produce animation

disp("Done!!")
close all


