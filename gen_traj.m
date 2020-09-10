clear
close all
clc

fid = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/path_spencer_large_01_short.txt','rt');
traj = textscan(fid,'%f%f%f%f%f%f%f%f%[^\n]', 'Delimiter',' ', 'MultipleDelimsAsOne',true);
fclose(fid);

for i=1:size(traj{1,4},1)
    traj{1,4}(i) = rand * 2.5 + 0.125;
end

fid = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/path_spencer_large_01_short_mod.txt','w');
for i=1:size(traj{1,4},1)
    fprintf(fid, '%4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f\n', traj{1,1}(i),traj{1,2}(i),traj{1,3}(i),traj{1,4}(i),traj{1,5}(i),traj{1,6}(i),traj{1,7}(i),traj{1,8}(i));
end


fclose(fid);
