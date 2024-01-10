% The MIT License (MIT)
%
% Copyright August, 2023 Han Yang, Universtiy of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

%% Flying Inverted Pendulum with Adaptive Model Predicitve Control

clc; clear; clear global; close all; restoredefaultpath

% load corresponding test data
trial = 1; 
try 
    load("test_data/trial" + trial + ".mat")
catch
    disp("Generate Test Data First!! (test_case.m)")
end

% controller path
controller = "ampc";

%% Parameters
%   p = physical parameters
%   k = predefined functions

% setup predefined equations
try 
    load("dynamics.mat");
catch
    disp("Generate Predefined Functions First!! (derivation.m)")
end
k.pos = matlabFunction(subs(k.pos, param, p.param));    % substitute in testing parameters
    
% initial states
z0 = [0 0 1 0 0 0 12/180*pi 9/180*pi 0 0 0 0 0 0 0 0]'; 

% reference trajectory      
c.zd = @(t) [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0]'; 

% intialize global variables to keep track of simulation time and simulate loop latency
global t_prev u_prev   
t_prev = -inf;
u_prev = zeros(4,1);

% L1 control variables
global sig_m sig_um z_hat u_L1_prev;
sig_m = zeros(4,1);     % matched uncertainty estimate
sig_um = zeros(2,1);    % unmatched uncertainty estimate
z_hat = z0(9:14);       % state prediction
u_L1_prev = zeros(4,1); % previous control action (for low pass filter)

% MPC decision variables 
% control input and states over the discretized predicted trajectory
global dv_prev  

%% Controller Parameters
%   c = controller parameters

% ------------ tuning parameters ------------
% loop frequency
f = 100;
c.ts = 1/f; p.ts = c.ts;

% MPC
% cost matrices (must be strictly positive definite)
c.Q = diag([10 10 10 1 1 1 5 5 5 5 5 1 1 1 3 3]); 
c.R = diag([0.3 0.3 0.3 0.3]);
c.F = diag([10 10 10 1 1 1 5 5 5 5 5 1 1 1 3 3]); 
% prediction horizon
c.N = 30;
% initial guess of the optimization solver decision variables
dv_prev = zeros(20*c.N,1); 
for i=1:c.N
    dv_prev(16*i-15:16*i) = z0 + (i-1)*(c.zd(0)-z0)/(c.N-1);  % linear interpolation 
end

% options for fmincon solver
options = optimoptions('fmincon','TolX', 1e-9, 'TolFun', 1e-9, 'TolCon', 1e-9, 'MaxIterations', 100,'MaxFunEvals',10000,...
    'DiffMinChange',1e-5, 'GradObj','off', 'GradConstr','off',...
    'DerivativeCheck', 'off', 'Display', 'final', 'Algorithm','sqp', 'UseParallel', true);

% L1 control
% state observer gain matrix (must be Hurwitz)
c.As = diag([-25 -25.01 -25.02 -25.03 -25.04 -25.05]); 
% low pass filter cross-over frequency
c.w_co = 2;
% ------------------------------------------

% some precalculated quantities --> don't change 
c.eAs = expm(c.As*c.ts);
c.Phi = inv(c.As)*(c.eAs - eye(6));
syms s
c.H = matlabFunction(ilaplace([[2*eye(3);zeros(1,3)] [1 0 0;0 1 0;0 0 0;0 0 1]] * inv(s*eye(6) - c.As)));

% include controller in path (change if custom controller are used)
addpath(controller + "\")

% feedback controller 
u = @(t,z)control(t,z,p_m,c,k,options); % uses mismatched parameters


%% Simultation
disp("Producing simulation")

% options for Runge-Kutta solvers
options = odeset('RelTol',1e-7,'AbsTol',1e-7);

% numerical simulation step!
[~, z] = ode45(@(t,z)dynamics(t,z,u,p,k), r.t_s, z0, options);


%% Animation
% create and clear a figure
figure(1); clf;
set(gcf,"position", [0,0,900,600])  % set window size
movegui(gcf, 'center');             % center animation
view(-25,25);                       % initial view angle, adjustable

animate(z,p,r,k)  % <-- produce animation

disp("Done!!")
close all

