% dynamic equations of the system 
function dz = dynamics(t,z,p)
    
    % control input u, 4x1    
    u = u(t,z)
    dz = zeros(16,1);
    dz(1:8) = z(9:end);
    dz()
end

