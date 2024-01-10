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

%% Animation of the Simulated Results
function animate(z,p,r,k)

    %% Motor and Pendulum Position Conversions
    hold on % <-- very important
    
    % extract states and rotation matrix
    R_IB = @(i) k.R_IB(z(i,4),z(i,5),z(i,6));
    x = z(:,1);
    y = z(:,2);
    z = z(:,3);
    
    % pendulum location
    r_po = zeros(numel(r.t_s),3);
    for i = 1:numel(r.t_s)
        r_po(i,:) = (k.R_IC(z(i,7),z(i,8))*p.l)';
    end
    x_po = r_po(:,1) + x;
    y_po = r_po(:,2) + y;
    z_po = r_po(:,3) + z;

    %% Formatting
    % axis and labels
    title(sprintf('Time: %0.2f sec', r.t_s(1)), 'Interpreter', 'Latex',"FontSize",18);
    xlabel('x', 'Interpreter', 'Latex',"FontSize",22)
    ylabel('y', 'Interpreter', 'Latex',"FontSize",22)
    zlabel('z', 'Interpreter', 'Latex',"FontSize",22)
    grid minor; axis equal; rotate3d on;
    xlim([-2 2]); ylim([-2 2]); zlim([0 3]); 

    % draw the quad
    d = p.d;
    w = 0.04;
    h = 0.02;
    
    r_mg = [d 0 0;0 d 0;-d 0 0;0 -d 0];
    mtr = R_IB(1)*r_mg.'+ [x(1) y(1) z(1)].';
    v = [w -w -h; w w -h; -w w -h; -w -w -h;w -w h; w w h; -w w h; -w -w h];
    V1 = R_IB(1)*v.'+ mtr(:,1);
    V2 = R_IB(1)*v.'+ mtr(:,2);
    V3 = R_IB(1)*v.'+ mtr(:,3);
    V4 = R_IB(1)*v.'+ mtr(:,4);
    
    face = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    box1 = patch('Vertices',V1.','Faces',face,'FaceColor',[0.1 0.1 0.8],'FaceAlpha',0.2);
    box2 = patch('Vertices',V2.','Faces',face,'FaceColor',[0.1 0.1 0.8],'FaceAlpha',0.2);
    box3 = patch('Vertices',V3.','Faces',face,'FaceColor',[0.8 0.1 0.1],'FaceAlpha',0.2);
    box4 = patch('Vertices',V4.','Faces',face,'FaceColor',[0.1 0.8 0.1],'FaceAlpha',0.2);
    
    arm1 = plot3([mtr(1,1), mtr(1,3)], [mtr(2,1), mtr(2,3)], [mtr(3,1), mtr(3,3)],'k', "LineWidth", 0.5);
    arm2 = plot3([mtr(1,2), mtr(1,4)], [mtr(2,2), mtr(2,4)], [mtr(3,2), mtr(3,4)],'k', "LineWidth", 0.5);
    
    % draw the pendulum
    L = plot3([x_po(1), x(1)], [y_po(1), y(1)], [z_po(1), z(1)],'k', "LineWidth", 0.1);
    [X,Y,Z] = sphere;
    r_p = 0.03;
    pend = surf(X*r_p+x_po(1), Y*r_p+y_po(1), Z*r_p+z_po(1), "FaceColor","k","EdgeColor","none");

    %% Make the Movie 
    % iterate through the time span
    for k = 1:length(r.t_s)
    
        % updating the pendulum 
        L.XData = [x_po(k), x(k)];
        L.YData = [y_po(k), y(k)];
        L.ZData = [z_po(k), z(k)];
            
        pend.XData = X*r_p+x_po(k);
        pend.YData = Y*r_p+y_po(k);
        pend.ZData = Z*r_p+z_po(k);

        % updating quad
        mtr = R_IB(k)*r_mg.'+ [x(k) y(k) z(k)].';
        V1 = R_IB(k)*v.'+ mtr(:,1);
        V2 = R_IB(k)*v.'+ mtr(:,2);
        V3 = R_IB(k)*v.'+ mtr(:,3);
        V4 = R_IB(k)*v.'+ mtr(:,4);
    
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

        % updating the title
        title(sprintf('Time: %0.2f sec', r.t_s(k)),'Interpreter','Latex',"FontSize",18);
    
        % match real time
        pause(1/r.fps)

    end
end
