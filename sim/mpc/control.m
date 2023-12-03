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
    global dv_prev output
    dv0 = [dv_prev(1:16*(N-1));p.zt(t);dv_prev(16*N+1:end-4);(p.mg+p.mp)*p.g;0;0;0];
    % boundary constraints
    lb = -inf(20*N,1);
    ub = inf(20*N,1);

    % thrust saturation
    % lb(4:16:16*(N-1)+4) = repmat(-pi/3,N,1);
    % ub(4:16:16*(N-1)+4) = repmat(pi/3,N,1);
    % lb(5:16:16*(N-1)+5) = repmat(-pi/3,N,1);
    % ub(5:16:16*(N-1)+5) = repmat(pi/3,N,1);
    lb(16*N+1:end) = repmat(p.lb,N,1);
    ub(16*N+1:end) = repmat(p.ub,N,1);

    % initial condition -> 4 equalities require 4 rows
    Aeq = zeros(16, 20*N);
    Aeq(1:16, 1:16) = eye(16);
    beq = z0;
    
    % solve the OCP
    objective = @(dv)trajCost(dv,t,c,p);
    nonlincon = @(dv)collocationCon(dv,p,c,k);
    [dv_o,~,~,out] = fmincon(objective,dv0,[],[],Aeq,beq,lb,ub,nonlincon,opt);

    % final mpc output
    u = dv_o(16*N+1:16*N+4);
    dv_prev = dv_o;
    output = [output;out];

    % reset guess if converges to infeasible point
    if out.constrviolation > 1e-9
        zt = p.zt(t);
        for i=1:N
            dv_prev(16*(i-1)+1:16*i) = z + (i-1)*(zt-z)/(N-1);  % linear interpolation
            dv_prev(16*N+4*(i-1)+1:16*N+4*i) = [(p.mg+p.mp)*p.g;0;0;0];
        end
    end

    %%
    Z = zeros(N, 16);
    for i = 1:N
        Z(i,:) = dv_prev(16*(i-1)+1:16*i);
    end   
    r.trial = 1;
    r.tspan = c.t_pred;
    r.fps = N/c.t_pred;
    r.t_s = linspace(0,r.tspan,N);

    figure(1); clf;
    set(gcf,"position", [0,0,900,600])  % set window size
    movegui(gcf, 'center');             % center animation
    view(-25,25);     
    animate(Z,p,r,k,1,"mpc\review\" + num2str(t) + ".gif")  % <-- produce animation
end

