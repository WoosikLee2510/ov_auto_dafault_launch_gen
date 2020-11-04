clear
close all
clc
%% read the file to save the map
fid_save = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/floorplan_spencer_large_more_plane.txt','w');

%% read map if wanna use preexisting map
fid = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/floorplan_spencer_large.txt','rt');
found_header = false;
pi = 1;
while ~feof(fid)
Line = fgetl(fid);

if(found_header && size(Line,2) ~= 0)
    C = strsplit(Line,',')
    C = str2double(strrep(C,',','.'));
    patch_x{pi} = [C(1), C(1), C(3), C(3)];
    patch_y{pi} = [C(2), C(4), C(2), C(4)];
    pi = pi + 1;
end

% copy header
if found_header == false
    fprintf(fid_save, Line);
    fprintf(fid_save, '\n');
end
    
if Line == "# start_x, start_y, end_x, end_y"
    found_header = true;
end

end
fclose(fid);

%% read traj just for visualization
fid = fopen('/home/wlee/workspace/sara/catkin_ws/src/estimation/odometry/multisensor_odometry/udel_viwo/dataset/path_spencer_large_01_loop_2.txt','rt');
traj = textscan(fid,'%f%f%f%f%f%f%f%f%[^\n]', 'Delimiter',' ', 'MultipleDelimsAsOne',true);
fclose(fid);

%% add planes
% whats the min max for x and y?
min_x = Inf;
max_x = -Inf;
min_y = Inf;
max_y = -Inf;
for i = 1 : size(patch_x,2)
    for j = 1: size(patch_x{i},2)
        if patch_x{i}(j) < min_x
            min_x = patch_x{i}(j);
        end
        if patch_x{i}(j) > max_x
            max_x = patch_x{i}(j);
        end
        if patch_y{i}(j) < min_x
            min_y = patch_y{i}(j);
        end
        if patch_y{i}(j) > max_x
            max_y = patch_y{i}(j);
        end
    end
end
% append more pilar
id = size(patch_x,2);
for i = 1 : 300
    x = (max_x - min_x) * rand + min_x;
    y = (max_y - min_y) * rand + min_y;
    patch_x{id + 1} = [x, x, x+3, x+3];
    patch_y{id + 1} = [y, y, y, y];
    
    patch_x{id + 2} = [x, x, x, x];
    patch_y{id + 2} = [y, y+0.5, y, y+0.5];
    
    patch_x{id + 3} = [x, x, x+3, x+3];
    patch_y{id + 3} = [y+0.5, y+0.5, y+0.5, y+0.5];
    
    patch_x{id + 4} = [x+3, x+3, x+3, x+3];
    patch_y{id + 4} = [y, y+0.5, y, y+0.5];
    id = id + 4;
end

%% show resulting map
figure;
hold on;
for i = 1:size(patch_x,2)
    patch(patch_x{1,i}, patch_y{1,i}, [0 0 0 0])
end
scale = 3.29;
plot(scale*traj{1,2},scale*traj{1,3})

%% Save the map

for i = 1:size(patch_x,2)
    fprintf(fid_save, sprintf('%.2f',patch_x{i}(1)));
    fprintf(fid_save, ',');
    fprintf(fid_save, sprintf('%.2f',patch_y{i}(1)));
    fprintf(fid_save, ',');
    fprintf(fid_save, sprintf('%.2f',patch_x{i}(3)));
    fprintf(fid_save, ',');
    fprintf(fid_save, sprintf('%.2f',patch_y{i}(2)));
    fprintf(fid_save, '\n');
end

fclose(fid_save);

