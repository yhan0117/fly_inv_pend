% Dynamics of quadrotor with pendulum
clc;clear;close all

% Generalized Coordinates
syms x(t) y(t) z(t) phi(t) theta(t) psi(t) alpha(t) beta(t)
syms dq [1 8]
syms ddq [1 8]
syms U1

q = [x y z phi theta psi alpha beta];
q = q(t);

% physical parameters
syms I1 I2 I3
syms mg mp d l g Cd Cl
param = [I1 I2 I3 mg mp d l g Cd Cl];
I_g = [I1   0   0;
       0    I2  0;
       0    0   I3];


%% Kinematics
% quad kinematics
r_go = [x,y,z].';
v_go = diff(r_go, t);  % wrt to I

% rotational dynamics
% roll -> pitch -> yaw
R_IB = R(psi,3)*R(theta,2)*R(phi,1);

% body rates
w_IB = simplify([eye(3)*[1;0;0] R(phi,1)^-1*[0;1;0] R(phi,1)^-1*R(theta,2)^-1*[0;0;1]]);
w = w_IB*diff([phi;theta;psi], t); w = w(t);

% pendulum kinematics
R_IC = R(alpha,1)*R(beta,2);
r_pg = R_IC*[0 0 l].'; % expressed in I
r_po = r_pg + r_go;
v_po = diff(r_po, t);  % wrt in I


%% Lagrangian Formulation
% quad energy
T_quad = mg*sum(v_go.*v_go)/2 + w.'*I_g*w/2 ;
U_quad = mg*g*z;

% pendulum energy
T_pend = mp*sum(v_po.*v_po)/2;
U_pend = mp*g*sum(r_po.*[0 0 1].');

% lagrangian
L = T_quad + T_pend - U_quad - U_pend;

% euler-lagrangian equation (assuming free body)
el_eqns = t*zeros(8,1);
for i = 1:8
    el_eqns(i) = 0 == simplify(diff(diff(L,diff(q(i),t)), t) - diff(L,q(i)), "Steps", 30);
end

for i = [1:3,7:8]
    isolate(el_eqns(i), diff(q(i),t,2));
end

% obtain translational equations of motion from the implicit equations
dda = rhs(isolate(el_eqns(7), diff(q(7),t,2)))
ddb = rhs(isolate(el_eqns(8), diff(q(8),t,2)))
for i = 1:3
    eqns(i,1) = rhs(isolate(subs(el_eqns(i), diff(q(7:8),t,2), [dda ddb]), diff(q(i),t,2)));
end
eqns = diff(q(1:3).',t,2) == eqns(1:3) + R_IB*[0;0;U1/(mp+mg)];
eqns = subs(eqns, diff(q,t,2), ddq);    % change names for derivative terms
eqns = subs(eqns, diff(q,t), dq);

sol = struct2cell(solve(eqns, ddq));   % solve for 2nd order terms
for i = 1:3
    f(i,1) = simplify(subs(sol{i}, dq, diff(q,t)), "Steps", 10);
end


%% Prepare state equations
% save rotation matrices as function handles
syms a b c
k.R_IB = matlabFunction(subs(R_IB(t), [phi theta psi], [a b c]));
k.R_IC = matlabFunction(subs(R_IC(t)*[0;0;1], [alpha beta], [a b]));
k.L = matlabFunction(subs(w_IB(t), [phi theta psi], [a b c]));

% translational dynamics
syms z [1 16]
pos = subs(f(1:3), diff(q,t), z(9:end));
k.pos = simplify(subs(pos, q, z(1:8)), "Steps", 10);

save("dynamics.mat", 'k', 'param')


%% LQR
% rewrite eom into state transition equations
syms f dZ Z [1 16]
syms U [1 4]
f = f.';

% 1st order terms
f(1:8) = dZ(1:8) - Z(9:16);

% 2nd order terms
f([9:11,15:16]) = subs(rhs(el_eqns([1:3,7:8])) - [R_IB*[0;0;U(1)];0;0], diff(q,t,2), dZ(9:end));
f([9:11,15:16]) = subs(f([9:11,15:16]), diff(q,t), Z(9:end));
f([9:11,15:16]) = subs(f([9:11,15:16]), q, Z(1:8));

f(12:14) = diff(w,t) - ([w(2)*w(3)*(I2-I3) ; w(1)*w(3)*(I3-I1) ; w(1)*w(2)*(I1-I2)] + U(2:4).')./[I1;I2;I3];
f(12:14) = subs(f(12:14), diff(q,t,2), dZ(9:end));
f(12:14) = subs(f(12:14), diff(q,t), Z(9:end));
f(12:14) = subs(f(12:14), q, Z(1:8));

% linearize
E = jacobian(f,dZ);
F = jacobian(f,Z);
G = jacobian(f,U);

% substitute in reference nominal trajectory
ref = [0 0 0 zeros(1,4) 0 zeros(1,24) g 0 0 0];
E = subs(E, [Z dZ U], ref);
F = subs(F, [Z dZ U], ref);
G = subs(G, [Z dZ U], ref);

% general form
k.A = -inv(E)*F;
k.B = -inv(E)*G;
% Ar = k.A([1,2,4,5,7,8,9,10,12,13,15,16],[1,2,4,5,7,8,9,10,12,13,15,16])
% Br = k.B([1,2,4,5,7,8,9,10,12,13,15,16],:)

save("dynamics.mat", 'k', 'param')


%% Functions  
% rotation matrices
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

