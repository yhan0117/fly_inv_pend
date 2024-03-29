% The MIT License (MIT)
%
% Copyright August, 2023 Han Yang, Universtiy of Maryland
%
% Permission is hereby granted, free of charge, to any person obtaining a copy
% of this software and associated documentation files (the "Software"), to deal
% in the Software without restriction, including without limitation the rights
% to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
% copies of the Software, and to permit persons to whom the Software is
% furnished to do so, subject to the following conditions:
%
% The above copyright notice and this permission notice shall be included in
% all copies or substantial portions of the Software.
%
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
% IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
% FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
% AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
% LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
% OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
% THE SOFTWARE.

%% Derivation of the Dynamics of the Flying Inverted Pendulum
clc;clear;close all

%% System Variables
% generalized coordinates
syms x(t) y(t) z(t) phi(t) theta(t) psi(t) alpha(t) beta(t)
syms dq [1 8]
syms ddq [1 8]
syms u [1 4]

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
% quadcopter kinematics
r_go = [x,y,z].';
v_go = diff(r_go, t);  % velocity of quadcopter with respect to the Intertial frame

% rotation matrix
% roll -> pitch -> yaw
R_IB = R(psi,3)*R(theta,2)*R(phi,1);

% body rates
w_IB = simplify([eye(3)*[1;0;0] R(phi,1)^-1*[0;1;0] R(phi,1)^-1*R(theta,2)^-1*[0;0;1]]);
w = w_IB*diff([phi;theta;psi], t); w = w(t);

% pendulum kinematics
R_IC = R(alpha,1)*R(beta,2); % 1-2 rotation
r_pg = R_IC*[0 0 l].';  % relative position between pendulum and quadcopter
r_po = r_pg + r_go;     % absolute position of the pendulum
v_po = diff(r_po, t);   % velocity of pendulum with respect to the Interial frame


%% Lagrangian Formulation
% quadcopter energy
T_quad = mg*sum(v_go.*v_go)/2 + w.'*I_g*w/2; % kinetic    
U_quad = mg*g*z; % potential

% pendulum energy
T_pend = mp*sum(v_po.*v_po)/2;  % kinetic 
U_pend = mp*g*sum(r_po.*[0 0 1].'); % potential 

% lagrangian 
L = T_quad + T_pend - U_quad - U_pend;

% euler-lagrangian equation (first assume free body)
el_eqns = t*zeros(8,1);
for i = 1:8
    el_eqns(i) = 0 == simplify(diff(diff(L,diff(q(i),t)), t) - diff(L,q(i)), "Steps", 30);
end


%% Fully Isolated Equations of Motion
% isolate pendulum accelerations from equations 7 & 8
dda = rhs(isolate(el_eqns(7), diff(q(7),t,2)));
ddb = rhs(isolate(el_eqns(8), diff(q(8),t,2)));

% substitute pendulum accelerations into translational dynamics
for i = 1:3
    trans(i,1) = rhs(isolate(subs(el_eqns(i), diff(q(7:8),t,2), [dda ddb]), diff(q(i),t,2)));
end

% actuation
trans = diff(q(1:3).',t,2) == trans(1:3) + R_IB*[0;0;u1/(mp+mg)];

% solve for second order terms of position
trans = subs(trans, diff(q,t,2), ddq);    
trans = subs(trans, diff(q,t), dq);
sol = struct2cell(solve(trans, ddq));   
for i = 1:3
    pos(i,1) = simplify(subs(sol{i}, dq, diff(q,t)), "Steps", 10);
end


%% Prepare State Equations for Simulator
% save rotation matrices as function handles
syms a b c
k.R_IB = matlabFunction(subs(R_IB(t), [phi theta psi], [a b c]));
k.R_IC = matlabFunction(subs(R_IC(t)*[0;0;1], [alpha beta], [a b]));
k.L = matlabFunction(subs(w_IB(t), [phi theta psi], [a b c]));

% save translational dynamics
syms z [1 16]
pos = subs(pos(1:3), diff(q,t), z(9:end));
k.pos = simplify(subs(pos, q, z(1:8)), "Steps", 10);

save("dynamics.mat", 'k', 'param')


%% Rotation Matrix Function
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
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  �