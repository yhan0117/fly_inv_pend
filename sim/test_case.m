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

%% Generate Parameters for Simulation
clear;clc

%% True Parameters
p.I = [0.0045 0.0045 0.01]; % quadcopter moment of intertia
p.mg = 0.9;                 % quadcopter mass
p.mp = 0.05;                % pendulum mass
p.l = 1.5;                  % pendulum length
p.g = 9.8;                  % gravity 
p.s = 18000;                % motor saturation
p.d = 0.15;             d = p.d;    % quadcopter arm length
p.Cd = 4.370E-9;        Cd = p.Cd;  % propeller drag coefficient
p.Cl = 2.721E-8;        Cl = p.Cl;   % propeller lift coefficient

% ------------- do not change -------------
p.K = [ Cl Cl Cl Cl;    
        0 d*Cl 0 -d*Cl;
        -d*Cl 0 d*Cl 0;
        Cd -Cd Cd -Cd]; % control allocation matrix
p.param = [p.I p.mg p.mp p.d p.l p.g p.Cd p.Cl];
% ----------------------------------------


%% Parameters with Mismatch
p_m.I = [0.0045 0.0045 0.01];   % quadcopter moment of intertia
p_m.mg = 0.9;                   % quadcopter mass
p_m.mp = 0.05;                  % pendulum mass
p_m.l = 1.5;                    % pendulum length
p_m.g = 9.8;                    % gravity 
p_m.s = 18000;                  % motor saturation
p_m.d = 0.15;             d = p_m.d;    % quadcopter arm length
p_m.Cd = 4.370E-9;        Cd = p_m.Cd;  % propeller drag coefficient
p_m.Cl = 2.721E-8;        Cl = p_m.Cl;  % propeller lift coefficient
p_m.K_bias = [1.1 0.9 1.05 0.8]';       % bias of each motor
% biased control allocation matrix
p_m.K = diag(p_m.K_bias) * [Cl Cl Cl Cl; 0 d*Cl 0 -d*Cl; -d*Cl 0 d*Cl 0; Cd -Cd Cd -Cd];


%% Simulation Specifications
r.trial = 1;
r.tspan = 10;   % simulation time span
r.fps = 40;     % simulation resolution
r.t_s = linspace(0,r.tspan,r.tspan*r.fps);  % simulation time steps


%% Save Test Data
if ~exist(controller + "\test_data", 'dir')
        mkdir(controller + "\animations") % make sure directory exist
end
if exist("test_data/trial" + num2str(r.trial) + ".mat", "file") 
    delete("test_data/trial" + num2str(r.trial) + ".mat")    % clear existing file
end
save("test cases/trial" + num2str(r.trial) + ".mat")