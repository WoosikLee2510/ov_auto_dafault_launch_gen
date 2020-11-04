clear
close all
clc

time_limit = 40;
time_ratio = 1;

fid = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/path_spencer_large_01_loop_2.txt','rt');
traj = textscan(fid,'%f%f%f%f%f%f%f%f%[^\n]', 'Delimiter',' ', 'MultipleDelimsAsOne',true);
fclose(fid);

j = 0;
for i=1:size(traj{1,4},1)
    quat = rotm2quat(rotx(20 * j) * roty(-10 * j) * rotz(-10 * j));
    
    traj{1,4}(i) = round(2*randi(2)-3) + 0.25;
    traj{1,5}(i) = quat(1);
    traj{1,6}(i) = quat(2);
    traj{1,7}(i) = quat(3);
    traj{1,8}(i) = quat(4);
    j = j+1;
end

fid = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/path_spencer_large_01_short_mod4.txt','w');
for i=1:size(traj{1,4},1)
    fprintf(fid, '%4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f\n', traj{1,1}(i) * time_ratio,traj{1,2}(i),traj{1,3}(i),traj{1,4}(i),traj{1,5}(i),traj{1,6}(i),traj{1,7}(i),traj{1,8}(i));
    if traj{1,1}(i) > time_limit
        break;
    end
end


fclose(fid);

