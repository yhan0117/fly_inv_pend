% generate parameters for simulation
clear;clc
p.I = [0.0045 0.0045 0.01];
p.mg = 1.7;
p.mp = 0.3;
p.l = 1.5;
p.g = 9.8;
p.d = 0.15;     d = p.d;
p.Cd = 4.370E-9;        Cd = p.Cd;
p.Cl = 2.721E-8;        Cl = p.Cl;
p.param = [p.I p.mg p.mp p.d p.l p.g p.Cd p.Cl];
p.K = [ Cl Cl Cl Cl;
        0 d*Cl 0 -d*Cl;
        -d*Cl 0 d*Cl 0;
        Cd -Cd Cd -Cd];
p.s = 18000; s = p.s;    % motor saturation
p.ub = [4*Cl*s^2 ; d*Cl*s^2 ; d*Cl*s^2 ; 2*Cd*s^2];
p.lb = [0 ; -d*Cl*s^2 ; -d*Cl*s^2 ; -2*Cd*s^2];
p.u_nom = sqrt((p.mg+p.mp)*p.g/4/Cl);
p.z0 = [0 0 1 0 0 0 10/180*pi 0 0 0 0 0 0 0 0 0]'; 

% recording specifications
r.trial = 1;
r.tspan = 2.0;
r.fps = 30;
r.t_s = linspace(0,r.tspan,r.tspan*r.fps);


if exist("test cases/trial" + num2str(r.trial) + ".mat", "file") 
    delete("test cases/trial" + num2str(r.trial) + ".mat")    % clear existing file
end
save("test cases/trial" + num2str(r.trial) + ".mat")