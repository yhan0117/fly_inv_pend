% Flying Inverted Pendulum with model predicitve control
clc; clear; clear global; close all;

trial = 1;
record = 0; 
path = pwd;

%% Test Case Parameters
% predefined
%   p = physical parameters
%   r = recording parameters
load("../Test Cases/trial" + trial + ".mat")


%% Control

% TODO: 
%   define control params

% feedback controller u = f(t,z)
u = @(t,z)control(t,z,p,c);


%% Simultation
disp("Producing simulation")
options = odeset('RelTol',1e-8,'AbsTol',1e-8);
[~, z] = ode45(@(t,z)cartPoleDynamics(t,z,c,u,p), r.t_s, p.z0, options);
u = z(:,5);
z = z(:,1:)';


%% Save data
if record
    save("data\trial" + trial + ".mat", 'z', 'u', 'c')
end

%% Plot 
cd ../Visuals/
if record
    figure(1); clf; hold on;
    plotCartPole(t_s,z,u); % <-- plot results
    set(gcf,"WindowState",'maximized')
    saveas(gcf, path + "\plots\trial" + trial + ".jpg") 
end

%% Animation
filename = path + "\animations\trial" + trial + ".gif";
if record && exist(filename, "file") 
    delete(filename)    % clear existing file
end

% Create and clear a figure
figure(2); clf;
set(gcf,"position", [0,0,900,600])  % set window size
movegui(gcf, 'center');         % center animation
animate(z,p,r,record,filename)  % <-- produce animation

disp("Done!!")
close all


