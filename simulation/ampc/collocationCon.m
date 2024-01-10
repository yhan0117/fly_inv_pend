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

%% Direct Collocation to Enforce Dynamics Constraints
function [c, ceq] = collocationCon(dv,p,c,k)
    
    % INPUTS:
    %   dv = decision variable vector with first n*N corresponding to states, (n+m)N x 1
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
    % 
    % OUTPUTS:
    % ceq = collocation constraint, n(N-1) x 1  
    %     => midpoint of cubic spline must match equations of motion
    %        only accurate over small intervals
    % dceq = jacobian of ceq wrt decision vars; n(N-1) x (n+m)N

    %% Constants & Parameters
    % control parameters
    dt = c.ts;    % prediction time interval
    N = c.N;        % prediction horizon
    step = 1e-8;    % step size for approximating jacobian with finite differencing

    % initialize collocation constraint vector
    ceq = zeros(16*(N-1), 1);
    dceq = zeros(16*(N-1), 20*N);


    %% Collocation Constraint Feasibility 
    % feasibility is computed at each time step    
    for i=1:(N-1)
        % extarct from dv states and control at each interval
        z1 = dv(16*(i-1)+1:16*i);
        z2 = dv(16*i+1:16*(i+1));
        u1 = dv(16*N+4*(i-1)+1:16*N+4*i);
        u2 = dv(16*N+4*i+1:16*N+4*(i+1));
        
        % feasibility for matching spline and dynamics
        feas = collocationPoints(z1,z2,u1,u2,dt,p,k);
        ceq(16*(i-1)+1:16*i) = feas;

        % jacobian wrt states via finite difference
        for j = 1:16
          % small step in direction of jth state at time step i
          dz = zeros(16,1);
          dz(j) = step;

          % corresponding change in constraint feasibility
          dceq1 = collocationPoints(z1+dz, z2, u1, u2, dt, p,k) - feas;
          dceq2 = collocationPoints(z1, z2+dz, u1, u2, dt, p,k) - feas;

          % finite difference approximation of the gradient dceq/dz
          dceq(16*(i-1)+1:16*i, 16*(i-1)+j) = dceq1/step;
          dceq(16*(i-1)+1:16*i, 16*i+j) = dceq2/step;
        end
        
        % jacobian wrt control
        for j = 1:4
            % small step in direction of u at time step i
            du = zeros(4,1);
            du(j) = step;
    
            % corresponding change in constraint feasibility
            dceq1 = collocationPoints(z1, z2, u1+du, u2, dt, p,k) - feas;
            dceq2 = collocationPoints(z1, z2, u1, u2+du, dt, p,k) - feas;
    
            % finite difference approximation of the gradient dceq/du
            dceq(16*(i-1)+1:16*i, 16*N+4*(i-1)+j) = dceq1/step;
            dceq(16*(i-1)+1:16*i, 16*N+4*i+j) = dceq2/step;
        end
    end
    
    % no inequality constraints
    c = [];
end

%% Feasibility for a Time Interval
function ceq = collocationPoints(z1,z2,u1,u2,dt,p,k) 
    % calculate z' at each two ends of the interval (break points)
    dz1 = eom(z1, u1, p, k);
    dz2 = eom(z2, u2, p, k);

    % collocation points are defined at mid points of break points 
    % based on cubic spline formula (refer to underactuated robotics Ch.10)
    zc = 0.5*(z1+z2) - (dt/8)*(dz2-dz1);
    dzc = (3/(2*dt))*(z2-z1) - (1/4)*(dz1+dz2);
    
    % dynamics at collocation point must fit into cubic spline
    % with linear interpolation between successive control inputs
    ceq = eom(zc, (u1+u2)/2, p, k) - dzc;
end

%% System Model
function dz = eom(z,u,p,k)
    % parameters
    l = p.l;    % pendulum length
    g = p.g;    % gravity 
    I = p.I;    % moment of inertia, 3 vector
    K = p.K;    % motor mixing matrix
    L   = k.L(z(4),z(5));    % angular velocity matrix
    pos = k.pos;    % translational dynamics function handle

    % control allocation    
    U = K*u.^2;

    dz = zeros(16,1);

    % translational dynamics
    dz(1) = z(9);
    dz(2) = z(10);
    dz(3) = z(11);
    dz(9:11) = pos(U(1),z(4),z(5),z(6),z(7),z(8),z(15),z(16));

    % rotational dynamics
    dz(4:6) = L\z(12:14);
    dz(12) = (z(13)*z(14)*(I(2)-I(3)) + U(2))/I(1);
    dz(13) = (z(12)*z(14)*(I(3)-I(1)) + U(3))/I(2);
    dz(14) = (z(12)*z(13)*(I(2)-I(3)) + U(4))/I(3);

    % pendulum dynamics
    dz(7:8) = z(15:16);
    dz(15) = (g*sin(z(7)) + cos(z(7))*dz(10)+ sin(z(7))*dz(11) + 2*l*sin(z(8))*z(15)*z(16))/(l*cos(z(8)));
    dz(16) = -(cos(z(8))*dz(9) - cos(z(7))*sin(z(8))*dz(11) + sin(z(7))*sin(z(8))*dz(10) - g*cos(z(7))*sin(z(8)) + l*cos(z(8))*sin(z(8))*z(15)^2)/l;
end
