% This function represents the control law that computes the input
% based on full state feedback
function u = control(t,z,p,c,k,opt)

    % INPUTS:
    %   t = simulation time
    %   z = [x; y; z; psi; theta; phi; alpha; beta; dx; dy; dz; p; q; r; dalpha; dbeta] 
    %     = state of the system, n = 16
    %   p = parameter struct
    %       m = quadcopter mass
    %       g = gravity 
    %       I = moment of inertia, 3 vector
    %       K = actuator dynamics matrix
    %       L = body rate to euler rate matrix B_L_I
    %       R = body frame to inertial frame rotation matrix B_R_I
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
    dv0 = dv_prev;   % (n+m) x N
    
    % boundary constraints
    lb = -inf(20*N,1);
    ub = inf(20*N,1);

    % motor speed saturation
    lb(4*N+1:end) = repmat(-p.s,N,1);
    ub(4*N+1:end) = repmat(p.s,N,1);

    % initial condition -> 4 equalities require 4 rows
    Aeq = zeros(16, 20*N);
    Aeq(1:16, 1:16) = eye(16);
    beq = z0;
    
    % solve the OCP
    objective = @(dv)trajCost(dv,t,c,p);
    nonlincon = @(dv)collocationCon(dv,p,c,k);
    [dv_o,~,~,out] = fmincon(objective,dv0,[],[],Aeq,beq,lb,ub,nonlincon,opt);

    % final mpc output
    U = (dv_o(16*N+1:16*N+4)+dv_o(16*N+5:16*N+8))/2;
    dv_prev = dv_0;
    
    u = p.K
    % output = [output;out];

    % reset guess if converges to infeasible point
    if out.constrviolation > 1e-5
        zt = p.zt(t);
        for i=1:N
            dv_prev(16*(i-1)+1:16*i) = z + (i-1)*(zt-z)/(N-1);  % linear interpolation 
        end
    end

    % L1 adaptive control
end

