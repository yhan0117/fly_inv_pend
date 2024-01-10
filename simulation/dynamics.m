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

%% Dynamics of the System for Integration 

function dz = dynamics(t,z,u,p,k)

    % INPUTS:
    %   t = simulation time 
    %   z = [x; y; z; psi; theta; phi; alpha; beta; dx; dy; dz; p; q; r; dalpha; dbeta] 
    %     = state of the system, n = 16
    %   u = system input (pwm to each motor), m = 4
    %   p = parameter struct
    %       mg = quadcopter mass
    %       mp = pendulum mass
    %       g = gravity 
    %       I = moment of inertia, 3 vector
    %       K = motor mixing matrix
    %       
    % OUTPUTS:
    %   dz = dz/dt = time derivative of states

    %% Constants & Parameters     
    % unpack physical parameters
    l = p.l;        % pendulum length
    g = p.g;        % gravity 
    I = p.I;        % moment of inertia, 3 vector
    K = p.K;        % actuator dynamics matrix
    mg = p.mg;      % quadcopter mass
    mp = p.mp;      % pendulum mass
    
    % predefined functions
    L   = k.L(z(4),z(5));      % angular velocity to body rate mapping


    %% Noise 
    % noise can be added here before feedback is processed by the controller    
    % example shown is random white noise added to the onboard IMU

    %{
    z(4:6) = z(4:6).*(1 + randn(3,1)*0.1)
    %}


    %% Control Step
    % global loop time      
    global t_prev u_prev
    
    % calculate control input u, 4x1    
    if t-t_prev > p.ts  % next control loop is reached
        disp(t)
        u = u(t,z);  
        t_prev = t;
        u_prev = u;
    else                % wait until next control loop
        u = u_prev; 
    end

    % input saturation
    for i = 1:4
        if u(i) < 1000
            u(i) = 1000;
        elseif u(i) > 20000
            u(i) = 20000;
        end
    end

    % control allocation
    U = K*u.^2;


    %% State Transition Equations
    dz = zeros(16,1);

    % quadcopter translational dynamics
    dz(1) = z(9);
    dz(2) = z(10);
    dz(3) = z(11);
    R1 = (mg^2*sin(z(4))*sin(z(6)) + mg*mp*sin(z(4))*sin(z(6)) + mp^2*cos(z(8))^2*sin(z(4))*sin(z(6)) - mp^2*cos(z(8))^4*sin(z(4))*sin(z(6)) + mg^2*cos(z(4))*cos(z(6))*sin(z(5)) + mg*mp*cos(z(4))*cos(z(6))*sin(z(5)) + mp^2*cos(z(8))^2*cos(z(4))*cos(z(6))*sin(z(5)) - mp^2*cos(z(8))^4*cos(z(4))*cos(z(6))*sin(z(5)) - mp^2*cos(z(7))^3*cos(z(8))^3*cos(z(4))*cos(z(5))*sin(z(8)) - mp^2*cos(z(8))^3*cos(z(6))*sin(z(7))*sin(z(8))*sin(z(4)) + mp^2*cos(z(7))^2*cos(z(8))^3*cos(z(6))*sin(z(7))*sin(z(8))*sin(z(4)) - mg*mp*cos(z(7))*cos(z(8))*cos(z(4))*cos(z(5))*sin(z(8)) - mg*mp*cos(z(8))*cos(z(6))*sin(z(7))*sin(z(8))*sin(z(4)) + mp^2*cos(z(8))^3*cos(z(4))*sin(z(7))*sin(z(8))*sin(z(5))*sin(z(6)) + mg*mp*cos(z(8))*cos(z(4))*sin(z(7))*sin(z(8))*sin(z(5))*sin(z(6)) - mp^2*cos(z(7))^2*cos(z(8))^3*cos(z(4))*sin(z(7))*sin(z(8))*sin(z(5))*sin(z(6)))/(mg*(mg + mp));
    R2 = (mp^2*cos(z(8))^4*cos(z(6))*sin(z(4)) - mp^2*cos(z(8))^2*cos(z(6))*sin(z(4)) - mg^2*cos(z(6))*sin(z(4)) + mg^2*cos(z(4))*sin(z(5))*sin(z(6)) - mg*mp*cos(z(6))*sin(z(4)) + mp^2*cos(z(7))^2*cos(z(8))^2*cos(z(6))*sin(z(4)) - 2*mp^2*cos(z(7))^2*cos(z(8))^4*cos(z(6))*sin(z(4)) + mp^2*cos(z(7))^4*cos(z(8))^4*cos(z(6))*sin(z(4)) + mg*mp*cos(z(4))*sin(z(5))*sin(z(6)) + mp^2*cos(z(8))^2*cos(z(4))*sin(z(5))*sin(z(6)) - mp^2*cos(z(8))^4*cos(z(4))*sin(z(5))*sin(z(6)) + mp^2*cos(z(7))^3*cos(z(8))^4*cos(z(4))*cos(z(5))*sin(z(7)) - mp^2*cos(z(7))^2*cos(z(8))^2*cos(z(4))*sin(z(5))*sin(z(6)) + 2*mp^2*cos(z(7))^2*cos(z(8))^4*cos(z(4))*sin(z(5))*sin(z(6)) - mp^2*cos(z(7))^4*cos(z(8))^4*cos(z(4))*sin(z(5))*sin(z(6)) + mp^2*cos(z(8))*sin(z(7))*sin(z(8))*sin(z(4))*sin(z(6)) - mp^2*cos(z(8))^3*sin(z(7))*sin(z(8))*sin(z(4))*sin(z(6)) + mp^2*cos(z(8))*cos(z(4))*cos(z(6))*sin(z(7))*sin(z(8))*sin(z(5)) + mg*mp*cos(z(8))*sin(z(7))*sin(z(8))*sin(z(4))*sin(z(6)) - mp^2*cos(z(8))^3*cos(z(4))*cos(z(6))*sin(z(7))*sin(z(8))*sin(z(5)) + mg*mp*cos(z(7))*cos(z(8))^2*cos(z(4))*cos(z(5))*sin(z(7)) + mg*mp*cos(z(8))*cos(z(4))*cos(z(6))*sin(z(7))*sin(z(8))*sin(z(5)))/(mg*(mg + mp));
    R3 = (mg^2*cos(z(4))*cos(z(5)) + mg*mp*cos(z(4))*cos(z(5)) + mp^2*cos(z(7))^2*cos(z(8))^2*cos(z(4))*cos(z(5)) - mp^2*cos(z(7))^4*cos(z(8))^4*cos(z(4))*cos(z(5)) + mp^2*cos(z(7))^3*cos(z(8))^4*cos(z(6))*sin(z(7))*sin(z(4)) - mp^2*cos(z(7))*cos(z(8))*sin(z(8))*sin(z(4))*sin(z(6)) - mp^2*cos(z(7))*cos(z(8))^4*cos(z(6))*sin(z(7))*sin(z(4)) + mp^2*cos(z(7))*cos(z(8))^3*sin(z(8))*sin(z(4))*sin(z(6)) - mp^2*cos(z(7))^3*cos(z(8))^4*cos(z(4))*sin(z(7))*sin(z(5))*sin(z(6)) - mp^2*cos(z(7))*cos(z(8))*cos(z(4))*cos(z(6))*sin(z(8))*sin(z(5)) - mg*mp*cos(z(7))*cos(z(8))*sin(z(8))*sin(z(4))*sin(z(6)) + mp^2*cos(z(7))*cos(z(8))^3*cos(z(4))*cos(z(6))*sin(z(8))*sin(z(5)) + mp^2*cos(z(7))*cos(z(8))^4*cos(z(4))*sin(z(7))*sin(z(5))*sin(z(6)) - mg*mp*cos(z(7))*cos(z(8))^2*cos(z(6))*sin(z(7))*sin(z(4)) - mg*mp*cos(z(7))*cos(z(8))*cos(z(4))*cos(z(6))*sin(z(8))*sin(z(5)) + mg*mp*cos(z(7))*cos(z(8))^2*cos(z(4))*sin(z(7))*sin(z(5))*sin(z(6)))/(mg*(mg + mp));
    cross = l*mp*(z(15)^2*cos(z(8))^2+z(16)^2);
    dz(9) = (U(1)*R1 + cross*sin(z(8)))/(mp+mg);
    dz(10) = (U(1)*R2 - cross*cos(z(8))*sin(z(7)))/(mp+mg);
    dz(11) = (U(1)*R3 + cross*cos(z(8))*cos(z(7)))/(mp+mg) - g;

    % above is equal to this predefined term
    % dz(9:11) = k.pos(u(1),z(4),z(5),z(6),z(7),z(8),z(15),z(16)) 

    % quadcopter rotational dynamics from Euler's equations
    dz(4:6) = L\z(12:14);
    dz(12) = (z(13)*z(14)*(I(2)-I(3)) + U(2))/I(1);
    dz(13) = (z(12)*z(14)*(I(3)-I(1)) + U(3))/I(2);
    dz(14) = (z(12)*z(13)*(I(2)-I(3)) + U(4))/I(3);

    % pendulum dynamics
    dz(7:8) = z(15:16);
    dz(15) = U(1)*(cos(z(7))*R2 + sin(z(7))*R3)/(l*(mp+mg)*cos(z(8))) + 2*z(15)*z(16)*tan(z(8));
    dz(16) = -U(1)*(cos(z(8))*R1 + sin(z(7))*sin(z(8))*R2 - cos(z(7))*sin(z(8))*R3)/(l*(mp+mg)) - z(15)*z(15)*sin(2*z(8))/2;


    %% Unknwon Disturbance
    % disturbances can be added here
    % example shown is an unknown force on the latittude of the quadcopter

    %{
    if t < 1.5 && t > 1
        dz(9) = dz(9) + 3;
        dz(10) = dz(10) - 4;
    end
    %}

end



