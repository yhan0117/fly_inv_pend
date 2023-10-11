% Simulation of dynamics with OL control 
clc;clear;close all
addpath Mats
addpath Animation
load("EOM.mat")

% animation specs
fps = 20;
duration = 15;

% physical parameters
p = [   0.0140;
        0.0140;
        0.0300; 
        1.6450; 
        0.3; 
        0.2500; 
        1; 
        9.8000; 
        0.0005; 
        0.0012;  ].';


%--------------------------------------State Feedback--------------------------------------%
% const input
input = param(8)*(param(4) + param(5))/(4*param(10));
input = input*ones(1, 4);

%--------------------------------------State Space Transformation--------------------------------------%
ode = subs(ode, u, input);
ode = subs(ode, param, p);

V1 = odeToVectorField(ode);
F = matlabFunction(V1,'vars',{'t','Y'});

%--------------------------------------Numerical Solution--------------------------------------%
% alpha x beta y z phi theta psi
x0 = [0 0];
y0 = [0 0];
z0 = [3 0];
a0 = [0 1];
b0 = [pi*0.8 0];
ph0 = [0 0];
t0 = [0 0];
ps0 = [0 0];

sol = ode45(F, [0 duration], [a0 x0 b0 y0 z0 ph0 t0 ps0]);
t_ = linspace(0, duration, duration*fps);
q_ = deval(sol,t_)';

x = q_(:,3);
y = q_(:,7);
z = q_(:,9);
alpha = q_(:,1); 
beta = q_(:,5);
phi = q_(:,11);
theta = q_(:,13);
psi = q_(:,15);

r_po = subs(r_po, param, p);
tmp = zeros(3, length(t_));
R = zeros(3, 3, length(t_));
for i = 1:length(t_)
    tmp(:,i) = subs(r_po, q([1:3,7:8]) , [x(i) y(i) z(i) alpha(i) beta(i)]);
    R(:,:,i) = subs(R_IB, q(4:6), [phi(i) theta(i) psi(i)]);
end

r_ = double(vpa(tmp));
x_po = r_(1,:);
y_po = r_(2,:);
z_po = r_(3,:);

R_IB = double(vpa(R));

clearvars -except t_ x y z x_po y_po z_po R_IB fps
disp("Numerical Solution Done")

%% --------------------------------------Animation--------------------------------------%
clc;clf

POV = 'left';
pb_speed = 1;

% gif name
recording = 0;
filename = "Animation.gif";
if recording
    % clear file
    if exist(filename, "file") 
        delete(filename)
    end
end

% Axis and labels
figure(1); hold on
title(sprintf('Trajectory\nTime: %0.2f sec', t_(1)), 'Interpreter', 'Latex');
xlabel('x', 'Interpreter', 'Latex')
ylabel('y', 'Interpreter', 'Latex')
zlabel('z', 'Interpreter', 'Latex')

grid minor; axis equal; rotate3d on;
xlim([-2 2]); ylim([-2 2]); zlim([1.5 3.5]); 
set(gcf,'WindowState','fullscreen')

% Plotting with no color to set axis limits
plot3(x,y,z,'Color','none');
plot3(x_po,y_po,z_po,'Color','none');


% Plotting the first iteration
p = plot3(x(1),y(1),z(1),'b');
m = scatter3(x(1),y(1),z(1),'filled','b','square');
p_ = plot3(x_po(1),y_po(1),z_po(1),'r');
m_ = scatter3(x_po(1),y_po(1),z_po(1),10,'filled','r');
L = plot3([x_po(1), x(1)], [y_po(1), y(1)], [z_po(1), z(1)],'k', "LineWidth", 0.1);

% Draw the quad
l = 0.125;
r = 0.04;
h = 0.02;
r_mg = [l 0 0;0 l 0;-l 0 0;0 -l 0];
mtr = R_IB(:,:,1)*r_mg.'+ [x(1) y(1) z(1)].';
v = [r -r -h; r r -h; -r r -h; -r -r -h;r -r h; r r h; -r r h; -r -r h];
V1 = R_IB(:,:,1)*v.'+ mtr(:,1);
V2 = R_IB(:,:,1)*v.'+ mtr(:,2);
V3 = R_IB(:,:,1)*v.'+ mtr(:,3);
V4 = R_IB(:,:,1)*v.'+ mtr(:,4);

face = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
box1 = patch('Vertices',V1.','Faces',face,'FaceColor',[0.1 0.1 0.8],'FaceAlpha',0.2);
box2 = patch('Vertices',V2.','Faces',face,'FaceColor',[0.1 0.1 0.8],'FaceAlpha',0.2);
box3 = patch('Vertices',V3.','Faces',face,'FaceColor',[0.8 0.1 0.1],'FaceAlpha',0.2);
box4 = patch('Vertices',V4.','Faces',face,'FaceColor',[0.1 0.8 0.1],'FaceAlpha',0.2);

arm1 = plot3([mtr(1,1), mtr(1,3)], [mtr(2,1), mtr(2,3)], [mtr(3,1), mtr(3,3)],'k', "LineWidth", 0.5);
arm2 = plot3([mtr(1,2), mtr(1,4)], [mtr(2,2), mtr(2,4)], [mtr(3,2), mtr(3,4)],'k', "LineWidth", 0.5);

% Iterating through the length of the time array
for k = 1:length(t_)
    % Updating the line
    p.XData = x(1:k);
    p.YData = y(1:k);
    p.ZData = z(1:k);
    p_.XData = x_po(1:k);
    p_.YData = y_po(1:k);
    p_.ZData = z_po(1:k);

    % Updating the point
    m.XData = x(k); 
    m.YData = y(k);
    m.ZData = z(k);
    m_.XData = x_po(k); 
    m_.YData = y_po(k);
    m_.ZData = z_po(k);

    % Updating the link
    L.XData = [x_po(k), x(k)];
    L.YData = [y_po(k), y(k)];
    L.ZData = [z_po(k), z(k)];
    
    % updating box
    mtr = R_IB(:,:,k)*r_mg.'+ [x(k) y(k) z(k)].';
    V1 = R_IB(:,:,k)*v.'+ mtr(:,1);
    V2 = R_IB(:,:,k)*v.'+ mtr(:,2);
    V3 = R_IB(:,:,k)*v.'+ mtr(:,3);
    V4 = R_IB(:,:,k)*v.'+ mtr(:,4);

    set(box1,'Faces',face,'Vertices',V1.','FaceColor',[0.1 0.1 0.8]);
    set(box2,'Faces',face,'Vertices',V2.','FaceColor',[0.1 0.1 0.8]);
    set(box3,'Faces',face,'Vertices',V3.','FaceColor',[0.8 0.1 0.1]);
    set(box4,'Faces',face,'Vertices',V4.','FaceColor',[0.1 0.8 0.1]);
    
    arm1.XData = [mtr(1,1), mtr(1,3)];
    arm2.XData = [mtr(1,2), mtr(1,4)];
    arm1.YData = [mtr(2,1), mtr(2,3)];
    arm2.YData = [mtr(2,2), mtr(2,4)];
    arm1.ZData = [mtr(3,1), mtr(3,3)];
    arm2.ZData = [mtr(3,2), mtr(3,4)];

    % Updating the title
    title(sprintf('Trajectory\nTime: %0.2f sec', t_(k)),'Interpreter','Latex');

    % Saving the figure
    if recording
        frame = getframe(gcf);
    
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if k == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime', 1/fps/pb_speed);
        else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', 1/fps/pb_speed);
        end
    end
end

close all

