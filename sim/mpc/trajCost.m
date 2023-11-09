% Scalar cost of a trajectory
function J = trajCost(dv,t,c,p)
    
    % INPUTS:
    %   t = simulation time
    %   dv = decision variable vector with first 4N corresponding to states, 5N x 1
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
    %   
    % OUTPUTS:
    %   J = scalar total cost
        
    % Unpack the control parameters         
    zt = p.zt(t);   % target state (time variant)
    T = c.T';       % terminal cost
    Q = c.Q;        % error cost matrix
    R = c.R;        % actuation effort cost matrix
    N = c.N;        % control horizon 

    % extract from decision variables
    % state vector, 4 x N
    % control vector, 1 x N 
    z = zeros(4, N);
    u = zeros(1, N);

    for i = 1:N
        z(:,i) = dv(4*i-3:4*i);
        u(i) = dv(4*N+i);
    end

    % error 
    e = z - zt;
    
    % Cost function
    % QR form => explicit function of decision variables
    JQ = sum(Q*(e.*e)*T, 'all');
    JR = sum(R*(u.*u), 'all');
    
    % total cost
    J = JQ+JR;

    % gradient of J
end