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
% This function represents the control law that computes the input
% based on full state feedback

%% Feedback Control Law

function u = control(t,z,p,c,k,opt)

    % INPUTS:
    %   t = simulation time
    %   z = [x; y; z; psi; theta; phi; alpha; beta; dx; dy; dz; p; q; r; dalpha; dbeta] 
    %     = state of the system, n = 16
    %   p = parameter struct
    %       mg = quadcopter mass
    %       mp = pendulum mass
    %       g = gravity 
    %       I = moment of inertia, 3 vector
    %       K = motor mixing matrix
    %   c = control parameter struct
    %       .N = prediction horizon
    %       .F = weight for each break point
    %       .Q = error cost
    %       .R = actuation effort  
    %   opt = optimziation solver options
    %
    % OUTPUTS:
    %   u = u(t,z) = input as function of current states

    %% Constants & Parameters     
    % control horizon     
    N = c.N;      

    % initial state of each mpc iteration => current position wrt real time
    z0 = z; 

    % initial guess of solver    
    global dv_prev
    dv0 = dv_prev;

    % matched uncertainty coefficient matrix 
    R_IB = k.R_IB(z(4),z(5),z(6));
    g = [R_IB(:,3) zeros(3,3); zeros(3,1) inv(diag(p.I))];

    % unmatched uncertainty coefficient matrix 
    g_perp = [R_IB(:,[1,2]);zeros(3,2)];

  
    %% OCP Constraints 
    % boundary constraints
    lb = -inf(20*N,1);
    ub = inf(20*N,1);
    lb(3:16:16*(N-1)+4) = repmat(0,N,1);        % ground
    lb(16*N+1:end) = repmat(p.s_min,4*N,1);      % thrust saturation
    ub(16*N+1:end) = repmat(p.s_max,4*N,1);      % min  imium thrust

    % initial condition
    Aeq = zeros(16, 20*N);
    Aeq(1:16, 1:16) = eye(16);
    beq = z0;

    %% Model Predictive Control
    % solve the OCP
    objective = @(dv)trajCost(dv,t,c,p);        % cost function
    nonlincon = @(dv)collocationCon(dv,p,c,k);  % collocation constraint
    [dv_o,~,~,~] = fmincon(objective,dv0,[],[],Aeq,beq,lb,ub,nonlincon,opt);

    % final mpc output assuming 1 step control horizon
    u_MPC = dv_o(16*N+1:16*N+4);
    
    % set the initial guess recursively to speed up solver convergence    
    dv_prev = dv_o;
    
    % mixing
    u_MPC = p.K*u_MPC.^2;

    %% Adaptive Law
    % state prediction error
    global z_hat
    e = z_hat - z(9:14); 

    % adaptive law equation
    G = [g g_perp];
    mu = c.eAs*e;
    sig = -inv(c.Phi*G)*mu; % <-- new uncertainty estimate
    
    % divide new estimate into the matched and unmatched parts    
    global sig_m sig_um
    sig_m = sig(1:4);
    sig_um = sig(5:6);


    %% L1 Control Law
    global u_L1_prev;
    
    % transfer functions for unmatched uncertainty    
    Hm = c.H(t)*g;
    Hum = c.H(t)*g_perp;

    % uncertainty compensation
    u_L1 = sig_m + (Hm\Hum)*sig_um;

    % low pass filter
    u_L1 = u_L1_prev*exp(-c.w_co*c.ts) - u_L1*(1-exp(-c.w_co*c.ts));
    u_L1_prev = u_L1;


    %% State Observer
    % reference model
    f = [k.pos(u_MPC(1),z(4),z(5),z(6),z(7),z(8),z(15),z(16));
        (z(13)*z(14)*(p.I(2)-p.I(3)) + u_MPC(2))/p.I(1);
        (z(12)*z(14)*(p.I(3)-p.I(1)) + u_MPC(3))/p.I(2);
        (z(12)*z(13)*(p.I(2)-p.I(3)) + u_MPC(4))/p.I(3)];

    % update prediction with new uncertainty with 1st order Euler's method    
    z_hat = z_hat + (f + g*(u_L1_prev + sig_m) + g_perp*sig_um + c.As*e)*c.ts;

   
    %% Final control action
    u = u_MPC + u_L1;
    
    % control allocation
    u = sqrt(p.K\u);
 end

