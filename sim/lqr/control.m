function u = control(t,z,p,c)
    % unpack control parameters
    K = c.K;

    % error term from reference trajectory
    z(3) = z(3) - 1;
    
    % feedback control law
    u = -K*z(1:16);

    % append with reference input 
    u(1) = u(1) + (p.mp + p.mg) * p.g;

    % % motor mixing
    % u = sqrt(p.K\u);
    % u = real(u);
end