clear all;
clc;
ttc_results = dir('*.csv');
% The FAST-BRIED combination
result_1 = ttc_results(14).name;
ttc_full_1 = importdata(result_1);
ttc_camera_1 = ttc_full_1.data(1:52,1);
ttc_lidar_1 = ttc_full_1.data(54:76,2);
% The SIFT-BRISK combination
result_2 = ttc_results(39).name;
ttc_full_2 = importdata(result_2);
ttc_camera_2 = ttc_full_2.data(1:52,1);
ttc_lidar_2 = ttc_full_2.data(1:52,2);

plot(1:1:52, ttc_camera_1, 1:1:52, ttc_camera_2);
xlabel('Frames');
ylabel('TTC/s');
grid on;

figure;
plot(54:76, ttc_lidar_1);
grid on;
xlabel('Frames');
ylabel('TTC/s');