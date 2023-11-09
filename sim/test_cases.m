% generate parameters for simulation
clear;clc
p.I = [0.0045 0.0045 0.01];
p.mg = 1.7;
p.mp = 0.3;
p.l = 1.5;
p.g = 9.8;
p.d = 0.15;     d = p.d;
p.Cd = 1;       Cd = p.Cd;
p.Cl = 1;       Cl = p.Cl;
p.param = [p.I p.mg p.mp p.d p.l p.g p.Cd p.Cl];
p.K = [ Cl Cl Cl Cl;
        0 d*Cl 0 -d*Cl;
        -d*Cl 0 d*Cl 0;
        Cd -Cd Cd -Cd];

% recording specifications
r.trial = 1;
r.tspan = 10;
r.fps = 30;
r.t_s = linspace(0,r.tspan,r.tspan*r.fps);


if exist("test cases/trial" + num2str(r.trial) + ".mat", "file") 
    delete("test cases/trial" + num2str(r.trial) + ".mat")    % clear existing file
end
save("test cases/trial" + num2str(r.trial) + ".mat")