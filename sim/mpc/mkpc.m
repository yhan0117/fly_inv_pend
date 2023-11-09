% This function represents the control law that computes the input
% based on full state feedback
function [dv_o,output] = control(t,z,p,c,opt)

    % INPUTS:
    %   t = simulation time
    %   z = [x;q;dx;dq] = state of the system
    %   p = parameter struct
    %       .l  = pendulum length
    %       .m1 = cart mass
    %       .m2 = pendulum mass
    %       .g  = gravity
    %       .L  = rail length
    %       .s  = max input force (motor torque*radius)
    %       .l_un  = pendulum length with uncertainty
    %       .m1_un = cart mass with uncertainty
    %       .m2_un = pendulum mass with uncertainty
    %   c = control parameter struct
    %       .N = prediction horizon
    %       .t_pred = prediction time span
    %       .dt_p = prediction step size
    %       .T = weight for each break point
    %       .Q = error cost
    %       .R = actuation effort  
    %   opt = optimziation solver options
    %
    % OUTPUTS:
    %   u = u(t,z) = input as function of current states

    % control horizon     
    N = c.N;        

    % initial state of each mpc iteration => current position wrt real time
    z0 = z; 
    global dv_prev
    dv0 = dv_prev;   % 5 x N
    
    % boundary constraints
    lb = -inf(5*N,1);
    ub = inf(5*N,1);

    % max rail length
    lb(1:4:4*N-3) = repmat(-p.L/2,N,1);
    ub(1:4:4*N-3) = repmat(p.L/2,N,1);

    % cart force
    lb(4*N+1:end) = repmat(-p.s,N,1);
    ub(4*N+1:end) = repmat(p.s,N,1);

    % initial condition -> 4 equalities require 4 rows
    Aeq = zeros(4, 5*N);
    Aeq(1:4, 1:4) = eye(4);
    beq = z0;
    
    %%%% SQP approach of trajectory optimization 
    % nonlincon is appended to Lagrangian as part of objective function 
    % => may not be satisfied 
    % => add weight (which indirectly adds to cost) to make more siginificant
    [dv_o,~,~,output] = fmincon(@(dv) trajCost(dv,t,c,p),dv0,[],[],Aeq,beq,lb,ub,@(dv) collocationCon(dv,p,c), opt);

end

