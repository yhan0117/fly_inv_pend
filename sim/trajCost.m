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

%% Cost Given a Trajectory
function J = trajCost(dv,t,c,p)
    
    % INPUTS:
    %   t = simulation time
    %   dv = decision variable vector with first n*N corresponding to states, (n+m)*N x 1
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
    %   J = scalar total cost
       
    %% Constants & Parameters
    % control parameters         
    F = c.T;        % terminal cost
    Q = c.Q;        % error cost matrix
    R = c.R;        % actuation effort cost matrix
    N = c.N;        % control horizon

    % reference trajectory
    zd = zeros(N,1);
    for i = 1:N
        zd = p.zd(t + (i-1)*p.ts);   
    end

    % extract from decision variables
    % state vector, 16 x N
    % control vector, 4 x N 
    z = zeros(16, N);
    u = zeros(4, N);
    for i = 1:N
        z(:,i) = dv(16*(i-1)+1:16*i);
        u(:,i) = dv(16*N+4*(i-1)+1:16*N+4*i);
    end

    %% Cost Function 
    % deviation from reference trajectory 
    x = z - zd;
    u = u - [0;0;0;0];

    % quadratic cost
    JF = sum(F*(x(:,end).*x(:,end)), 'all');
    JQ = sum(Q*(x.*x), 'all');
    JR = sum(R*(u.*u), 'all');
    
    % total cost
    J = JQ + JR + JF;
end