addpath('tools')
more off
close all
clear all

% Load laser scans and robot poses.
load("../data/laser")
%laser = read_robotlaser('../data/csail.log');

% Extract robot poses: Nx3 matrix where each row is in the form: [x y theta]
% 原始的数据是一维的
poses = [laser.pose];
% 把数据调整成 N*3维：先变成3*N，然后转置成N*3
poses = reshape(poses,3,size(poses,2)/3)';

% Initial cell occupancy probability.
prior = 0.50;
% Probabilities related to the laser range finder sensor model.
probOcc = 0.9;
probFree = 0.35;

% Map grid size in meters. Decrease for better resolution.
gridSize = 0.1;

% Set up map boundaries and initialize map.
border = 30;
robXMin = min(poses(:,1));
robXMax = max(poses(:,1));
robYMin = min(poses(:,2));
robYMax = max(poses(:,2));
% map加上边框的范围
mapBox = [robXMin-border robXMax+border robYMin-border robYMax+border];
% 计算出左下角的坐标
offsetX = mapBox(1);
offsetY = mapBox(3);
% 计算出长和宽
mapSizeMeters = [mapBox(2)-offsetX mapBox(4)-offsetY];
% 根据网格精度和map大小算出栅格地图大小，ceil向大取整
mapSize = ceil([mapSizeMeters/gridSize]);

% Used when updating the map. Assumes that prob_to_log_odds.m
% has been implemented correctly.
% 计算log(p(x)/(1-p(x)))，即对数占有概率，背景概率
logOddsPrior = prob_to_log_odds(prior);

% The occupancy value of each cell in the map is initialized with the prior.
% 产生 mapSize*mapSize 大小的栅格地图
map = logOddsPrior*ones(mapSize);
disp('Map initialized. Map size:'), disp(size(map))

% Map offset used when converting from world to map coordinates.
% 初始位置为栅格地图的左下角
offset = [offsetX; offsetY];

% Main loop for updating map cells.
% You can also take every other point when debugging to speed up the loop (t=1:2:size(poses,1))
% poses 为 N*3，for loop为逐个取出位姿
for(t=1:size(poses,1))
%for(t=1:50)
	t
	% Robot pose at time t.
  % t时刻的机器人位姿
	robPose = [poses(t,1);poses(t,2);poses(t,3)];
	
	% Laser scan made at time t.
  % t时刻的laser数据
	sc = laser(1,t);
	% Compute the mapUpdate, which contains the log odds values to add to the map.
	[mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, sc, robPose, gridSize, offset, probOcc, probFree);
  % 减去 l_0，即先验概率
	mapUpdate -= logOddsPrior*ones(size(map));
	% Update the occupancy values of the affected cells.
  % 更新地图概率
	map += mapUpdate;
	
	% Plot current map and robot trajectory so far.
        plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t);
endfor
