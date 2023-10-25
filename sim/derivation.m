% Dynamics of Quadrotor with slung load
clc;clear;close all

% Generalized Coordinates
syms x(t) y(t) z(t) phi(t) theta(t) psi(t) alpha(t) beta(t)
syms a(t) b(t) c(t)
syms dq [1 8]
syms ddq [1 8]

q = [x y z phi theta psi alpha beta];
q = q(t);


%--------------------------------------System Specs--------------------------------------%
% Parameters:
% I1 I2 I3 mg mp d l g Cd Cl
% 
% Coordinates:
% x y z phi theta psi alpha beta
% 
% Inputs:
% u1-4 = w^2
% (operating range of w 0-600)

syms I1 I2 I3
syms mg mp d l g Cd Cl
param = [I1 I2 I3 mg mp d l g Cd Cl];
I_g = [I1   0   0;
       0    I2  0;
       0    0   I3];


%--------------------------------------Kinematics--------------------------------------%
% Quad kinematics
r_go = [x,y,z].';
v_go = diff(r_go, t);  % wrt to I


% 3 Rotations all wrt to body frame
% Roll->Pitch->Yaw
R_IB = (R(psi,3)*R(theta,2)*R(phi,1)).';

w_IB = diff([phi, theta, psi].', t); % expressed in B
w_IB = w_IB(t);

% Pendulum kinematics
R_IC = R(alpha,3)*R(beta,2);
r_pg = R_IC*[0 0 l].'; % expressed in I
r_po = r_pg + r_go;
v_po = diff(r_po, t);  % wrt in I


%--------------------------------------Total Energy--------------------------------------%
% Quad energy
T_quad = mg*sum(v_go.*v_go)/2 + w_IB.'*I_g*w_IB/2 ;
U_quad = mg*g*z;


% Pendulum energy
T_pend = mp*sum(v_po.*v_po)/2;
U_pend = mp*g*sum(r_po.*[0 0 1].');


%--------------------------------------Lagrangian Formulation--------------------------------------%
% Lagrangian
L = T_quad + T_pend - U_quad - U_pend;

%
LHS = t*zeros(1,8);
for i = 1:8
    LHS(i) = simplify(diff(diff(L,diff(q(i),t)), t) - diff(L,q(i)), "Steps", 30);
end


% Generalized Forces
syms u [1 4]
syms T

B = [Cl  Cl  Cl  Cl;
     0   Cl  0   -Cl;
     -Cl 0   Cl  0;
     Cd  -Cd Cd  -Cd];
F = B*u.';

r = r_go + d*R_IB*[[1 0 0]', [0 1 0]', -[1 0 0]', -[0 1 0]'];
r = r(t);

RHS = t*zeros(1,8);
for i = 1:3
    RHS(i) = simplify(Q(q(i), F, r,u,Cl, R_IB), "Steps", 5);
end
RHS(4) = d*F(2);
RHS(5) = d*F(3);
RHS(6) = F(4);
RHS([7,8]) = 0;

% Euler-Lagrange Eqns
eqns = t*zeros(1,8);
for i = 1:8
    eqns(i) = LHS(i) == RHS(i);
end
eqns = subs(eqns, diff(q,t,2), ddq);
eqns = subs(eqns, diff(q,t), dq);

% Equations of Motion
sol = solve(eqns, ddq);
ode(1) = subs(sol.ddq1, dq, diff(q,t));
ode(2) = subs(sol.ddq2, dq, diff(q,t));
ode(3) = subs(sol.ddq3, dq, diff(q,t));
ode(4) = subs(sol.ddq4, dq, diff(q,t));
ode(5) = subs(sol.ddq5, dq, diff(q,t));
ode(6) = subs(sol.ddq6, dq, diff(q,t));
ode(7) = subs(sol.ddq7, dq, diff(q,t));
ode(8) = subs(sol.ddq8, dq, diff(q,t));

ode = simplify(ode, "Steps", 150);
ode = diff(q,t,2) == ode;

clearvars -except ode r_po R_IB q u param t % clean up workspace
save("Mats\EOM.mat")


%%
clear;
sys_parameters = [0.0140 0.0140 0.0300 1.6450 0.3 0.2500 1 9.8000 0.0005 0.0012];
save("Mats\sys_param.mat")


%--------------------------------------Functions--------------------------------------%
% Generalized Forces
function Gen_F = Q(var, F, r, u, Cl, R_IB)
    Gen_F = 0;
    for i = 1:4
        Gen_F = Gen_F + sum((u(i)*Cl*R_IB*[0 0 1]').*(diff(r(:,i), var)));
    end
end


% Rotation Matrices
function Rot_Mat = R(x, y)
    if y == 3
        Rot_Mat = [cos(x)   -sin(x) 0;
                   sin(x)   cos(x)  0;
                   0        0       1];
    elseif y == 2      
        Rot_Mat = [cos(x)   0       sin(x);
                   0        1       0;
                   -sin(x)  0       cos(x)];
    elseif y == 1
        Rot_Mat = [1        0       0;
                   0        cos(x)  -sin(x)
                   0        sin(x)  cos(x)];
    end
end

