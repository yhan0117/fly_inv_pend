% dynamic equations of the system 
function [dz,u] = dynamics(t,z,u,p,c,k)

    % INPUTS:
    %   t = simulation time 
    %   z = [x; y; z; psi; theta; phi; alpha; beta; dx; dy; dz; p; q; r; dalpha; dbeta] 
    %     = state of the system, n = 16
    %   u = system input (pwm to each motor), m = 4
    %   p = parameter struct
    %       m = quadcopter mass
    %       g = gravity 
    %       I = moment of inertia, 3 vector
    %       K = actuator dynamics matrix
    %       L = body rate to euler rate matrix B_L_I
    %       R = body frame to inertial frame rotation matrix B_R_I
    %
    % OUTPUTS:
    %   dz = dz/dt = time derivative of states

    % global loop time    
    global t_prev input

    % unpack physical parameters
    l = p.l;    % quadcopter mass
    g = p.g;    % gravity 
    I = p.I;    % moment of inertia, 3 vector
    K = p.K;    % actuator dynamics matrix

    % predefined functions
    L   = k.L;    % angular velocity matrix
    pos = k.pos;    % translational dynamics function handle
    
    % calculate control input u, 4x1    
    if t-t_prev > c.dt_p % wait until next control loop
        disp(t)
        tic
        u = u(t,z);
        toc

        t_prev = t;

        % track input trajectory
        input = [input;u'];
    else
        % if exist('c.N','var') == 1
            global dv_prev
            % disp(c.N);
            u = dv_prev(16*c.N+1:16*(c.N+1));
        % else
        %     u = u(t_prev, z);
        % end
    end

    %%
    % evaluate rotation matrices
    L = L(z(4),z(5));
    % U = K*(u.^2);
    U = u;

    %% equations of motion
    dz = zeros(16,1);

    % quad translational dynamics
    dz(1) = z(9);
    dz(2) = z(10);
    dz(3) = z(11);
    dz(9:11) = pos(U(1),z(4),z(5),z(6),z(7),z(8),z(15),z(16));

    % quad rotational dynamics from Euler's equations
    dz(4:6) = L\z(12:14);
    dz(12) = (z(13)*z(14)*(I(2)-I(3)) + U(2))/I(1);
    dz(13) = (z(12)*z(14)*(I(3)-I(1)) + U(3))/I(2);
    dz(14) = (z(12)*z(13)*(I(2)-I(3)) + U(4))/I(3);

    % pendulum dynamics
    dz(7:8) = z(15:16);
    dz(15) = (g*sin(z(7)) + cos(z(7))*dz(10)+ sin(z(7))*dz(11) + 2*l*sin(z(8))*z(15)*z(16))/(l*cos(z(8)));
    dz(16) = -(cos(z(8))*dz(9) - cos(z(7))*sin(z(8))*dz(11) + sin(z(7))*sin(z(8))*dz(10) - g*cos(z(7))*sin(z(8)) + l*cos(z(8))*sin(z(8))*z(15)^2)/l;
        % 
    % if t > 0.5 && t < 0.8
    %     dz(15) = dz(15) + 0.1;
    %     dz(16) = dz(16) + 0.1;
    % end
end



