close all

figure;
set(gcf, 'Position',  [1968           1        1553         790])
set(gca,'Color','black') % set background color
set(gca,'xcolor','black') 
set(gca,'ycolor','black') 
set(gcf,'color','black');

[ha, pos] = tight_subplot(2,3,[.01 .03],[.1 .01],[.01 .01]);

for i = 1:6
    axes(ha(i));
    set(gca,'Color','black') % set background color
    axis([-800 1000, -200 1000])
    axis equal
    set(gca,'xcolor','black') 
    set(gca,'ycolor','black') 
    set(gcf,'color','black');
end
    
axes(ha(1));
hold on
f_MINS = animatedline('Color',[0     1     0],'LineWidth',2);
f_gt1 = animatedline('Color',[0.0588    1.0000    1.0000],'LineWidth',2);

axes(ha(2));
hold on
f_lidar = animatedline('Color',[0     1     0],'LineWidth',2);
f_gt2 = animatedline('Color',[0.0588    1.0000    1.0000],'LineWidth',2);

axes(ha(3));
hold on
f_gps = animatedline('Color',[0     1     0],'LineWidth',2);
f_gt3 = animatedline('Color',[0.0588    1.0000    1.0000],'LineWidth',2);

axes(ha(5));
hold on
f_wheel = animatedline('Color',[0     1     0],'LineWidth',2);
f_gt4 = animatedline('Color',[0.0588    1.0000    1.0000],'LineWidth',2);

axes(ha(6));
hold on
f_vio = animatedline('Color',[0     1     0],'LineWidth',2);
f_gt5 = animatedline('Color',[0.0588    1.0000    1.0000],'LineWidth',2);

cnt_mins = 1;
cnt_lidar = 1;
cnt_gps = 1;
cnt_wheel = 1;
cnt_vio = 1;
for k = 1:size(Groundtruth,1)
    t = Groundtruth(k,1);
    if t > MINS(cnt_mins,1)
        addpoints(f_MINS,MINS(cnt_mins,2),MINS(cnt_mins,3));
        cnt_mins = cnt_mins + 1;
    end
    if t > LiDAR_VIO(cnt_lidar,1)
        addpoints(f_lidar,LiDAR_VIO(cnt_lidar,2),LiDAR_VIO(cnt_lidar,3));
        cnt_lidar = cnt_lidar + 1;
    end
    if t > GPS_VIO(cnt_gps,1)
        addpoints(f_gps,GPS_VIO(cnt_gps,2),GPS_VIO(cnt_gps,3));
        cnt_gps = cnt_gps + 1;
    end
    if t > Wheel_VIO(cnt_wheel,1)
        addpoints(f_wheel,Wheel_VIO(cnt_wheel,2),Wheel_VIO(cnt_wheel,3));
        cnt_wheel = cnt_wheel + 1;
    end
    if t > VIO(cnt_vio,1)
        addpoints(f_vio,VIO(cnt_vio,2),VIO(cnt_vio,3));
        cnt_vio = cnt_vio + 1;
    end
    addpoints(f_gt1,Groundtruth(k,2),Groundtruth(k,3));
    addpoints(f_gt2,Groundtruth(k,2),Groundtruth(k,3));
    addpoints(f_gt3,Groundtruth(k,2),Groundtruth(k,3));
    addpoints(f_gt4,Groundtruth(k,2),Groundtruth(k,3));
    addpoints(f_gt5,Groundtruth(k,2),Groundtruth(k,3));
    drawnow
end

