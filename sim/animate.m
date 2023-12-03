% animation of the simulated dynamics 
function animate(z,p,r,k,record,filename)
    hold on % <-- very important
    
    % extract states and rotation matrix
    R_IB = @(i) k.R_IB(z(i,4),z(i,5),z(i,6));
    r_po = zeros(numel(r.t_s),3);
    for i = 1:numel(r.t_s)
        r_po(i,:) = (k.R_IC(z(i,7),z(i,8))*p.l)';
    end
    x = z(:,1);
    y = z(:,2);
    z = z(:,3);
    x_po = r_po(:,1) + x;
    y_po = r_po(:,2) + y;
    z_po = r_po(:,3) + z;

    % axis and labels
    title(sprintf('Trajectory\nTime: %0.2f sec', r.t_s(1)), 'Interpreter', 'Latex');
    xlabel('x', 'Interpreter', 'Latex')
    ylabel('y', 'Interpreter', 'Latex')
    zlabel('z', 'Interpreter', 'Latex')
    grid minor; axis equal; rotate3d on;
    xlim([-2 2]); ylim([-2 2]); zlim([0 3]); 
    
    % plotting with no color to set axis limits
    plot3(x,y,z,'Color','none');
    plot3(x_po,y_po,z_po,'Color','none');
    
    % plotting the first iteration
    % p = plot3(x(1),y(1),z(1),'b');
    % m = scatter3(x(1),y(1),z(1),'filled','b','square');
    % p_ = plot3(x_po(1),y_po(1),z_po(1),'r');
    % m_ = scatter3(x_po(1),y_po(1),z_po(1),10,'filled','r');
    L = plot3([x_po(1), x(1)], [y_po(1), y(1)], [z_po(1), z(1)],'k', "LineWidth", 0.1);
    
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
    
    % iterating through the time span
    for k = 1:length(r.t_s)
        % updating the line
        % p.XData = x(1:k);
        % p.YData = y(1:k);
        % p.ZData = z(1:k);
        % p_.XData = x_po(1:k);
        % p_.YData = y_po(1:k);
        % p_.ZData = z_po(1:k);
    
        % updating the point
        m.XData = x(k); 
        m.YData = y(k);
        m.ZData = z(k);
        m_.XData = x_po(k); 
        m_.YData = y_po(k);
        m_.ZData = z_po(k);
    
        % updating the link
        L.XData = [x_po(k), x(k)];
        L.YData = [y_po(k), y(k)];
        L.ZData = [z_po(k), z(k)];
        
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
        title(sprintf('Trajectory\nTime: %0.2f sec', r.t_s(k)),'Interpreter','Latex');
    
        % saving the figure
        if record
            frame = getframe(gcf);
            im = frame2im(frame);
            [imind,cm] = rgb2ind(im,256);
            if k == 1
                imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime', 1/r.fps);
            else
                imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime', 1/r.fps);
            end
        else
            pause(1/r.fps)
        end
    end
end
