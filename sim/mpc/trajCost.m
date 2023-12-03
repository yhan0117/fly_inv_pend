% Scalar cost of a trajectory
function J = trajCost(dv,t,c,p)
    
    % INPUTS:
    %   t = simulation time
    %   dv = decision variable vector with first n*N corresponding to states, (n+m)*N x 1
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
    %   
    % OUTPUTS:
    %   J = scalar total cost
        
    % Unpack the control parameters         
    zt = p.zt(t);   % target state (time variant)
    F = c.F';       % terminal cost
    Q = c.Q;        % error cost matrix
    R = c.R;        % actuation effort cost matrix
    N = c.N;        % control horizon 

    % extract from decision variables
    % state vector, 16 x N
    % control vector, 4 x N 
    z = zeros(16, N);
    u = zeros(4, N);

    for i = 1:N
        z(:,i) = dv(16*(i-1)+1:16*i);
        u(:,i) = dv(16*N+4*(i-1)+1:16*N+4*i);
    end

    % deviation from reference trajectory 
    x = z - zt;
    u = u - [(p.mg+p.mp)*p.g;0;0;0];

    % Cost function
    JQ = sum(Q*(x.*x)*F, 'all');
    JR = sum(R*(u.*u), 'all');
    
    % total cost
    J = JQ+JR;
end