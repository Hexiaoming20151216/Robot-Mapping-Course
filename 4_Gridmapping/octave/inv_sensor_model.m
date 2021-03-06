function [mapUpdate, robPoseMapFrame, laserEndPntsMapFrame] = inv_sensor_model(map, scan, robPose, gridSize, offset, probOcc, probFree)
% Compute the log odds values that should be added to the map based on the inverse sensor model
% of a laser range finder.

% map is the matrix containing the occupancy values (IN LOG ODDS) of each cell in the map.
% scan is a laser scan made at this time step. Contains the range readings of each laser beam.
% robPose is the robot pose in the world coordinates frame.
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% probOcc is the probability that a cell is occupied by an obstacle given that a
% laser beam endpoint hit that cell.
% probFree is the probability that a cell is occupied given that a laser beam passed through it.

% mapUpdate is a matrix of the same size as map. It has the log odds values that need to be added for the cells
% affected by the current laser scan. All unaffected cells should be zeros.
% robPoseMapFrame is the pose of the robot in the map coordinates frame.
% laserEndPntsMapFrame are map coordinates of the endpoints of each laser beam (also used for visualization purposes).

% Initialize mapUpdate.
size(map);
mapUpdate = zeros(size(map));

% Robot pose as a homogeneous transformation matrix.
% 将机器人位姿转换为齐次变换矩阵
robTrans = v2t(robPose);

% TODO: compute robPoseMapFrame. Use your world_to_map_coordinates implementation.
%%%%%%%%%%%%%%%
%robPose
%%%%%%%%%%%%%%%
% robPose为列向量 [x;y]，将机器人位姿映射成栅格地图坐标
robPoseMapFrame(1:2) = world_to_map_coordinates(robPose(1:2),gridSize,offset);
robPoseMapFrame(3) = robPose(3);
%%%%%%%%%%%%%%%
%robPoseMapFrame
%%%%%%%%%%%%%%%
% Compute the Cartesian coordinates of the laser beam endpoints.
% Set the third argument to 'true' to use only half the beams for speeding up the algorithm when debugging.
% 根据激光数据计算出在机器人坐标系下的齐次坐标
laserEndPnts = robotlaser_as_cartesian(scan, 30, false);

% Compute the endpoints of the laser beams in the world coordinates frame.
% 转换到世界坐标系下的齐次坐标
laserEndPnts = robTrans*laserEndPnts;

% TODO: compute laserEndPntsMapFrame from laserEndPnts. Use your world_to_map_coordinates implementation.
% 将激光束端点坐标转换成栅格地图下的世界坐标
laserEndPntsMapFrame = world_to_map_coordinates(laserEndPnts(1:2,:),gridSize,offset);
%laserEndPntsMapFrame(3,:) = laserEndPnts(3,:);
%%%%%%%%%%%%
%laserEndPntsMapFrame
%%%%%%%%%%%%

% freeCells are the map coordinates of the cells through which the laser beams pass.
freeCells = [];

% Iterate over each laser beam and compute freeCells.
% Use the bresenham method available to you in tools for computing the X and Y
% coordinates of the points that lie on a line.
% Example use for a line between points p1 and p2:
% [X,Y] = bresenham(map,[p1_x, p1_y; p2_x, p2_y]);
% You only need the X and Y outputs of this function.
% columns返回二维数组的列数
% 根据机器人位置和激光束端点坐标计算出freecell
for sc=1:columns(laserEndPntsMapFrame)
%%%	sc
        %TODO: compute the XY map coordinates of the free cells along the laser beam ending in laserEndPntsMapFrame(:,sc)	
	%%%%%%%%%%%%%%
	%robPoseMapFrame(1)
	%robPoseMapFrame(2)
	%%%%%%%%%%%%%%
%	[~,~,map,X,Y] = bresenham(map,[robPoseMapFrame(1), robPoseMapFrame(2); laserEndPntsMapFrame(:,sc)'],0);
  % 传入参数为机器人2D位置，激光束端点位置
	[X,Y] = bresenham2([robPoseMapFrame(1), robPoseMapFrame(2); laserEndPntsMapFrame(1,sc), laserEndPntsMapFrame(2,sc)]);
        %TODO: add them to freeCells
  % 所有的空freecell，2*N维
	freeCells = [freeCells, [X;Y]];
%	X
%	Y

endfor


%TODO: update the log odds values in mapUpdate for each free cell according to probFree.
% 遍历freecells
for i=1:size(freeCells,2)
    % 取出x，y坐标
    tmpR = freeCells(1,i);
    tmpC = freeCells(2,i);
    % 根据激光测量为free的概率计算出log odds
    mapUpdate(tmpR,tmpC) = prob_to_log_odds(probFree);
endfor

%TODO: update the log odds values in mapUpdate for each laser endpoint according to probOcc.
% 更新占用 log odds
for i=1:size(laserEndPnts,2)
    mapUpdate(laserEndPntsMapFrame(1,i),laserEndPntsMapFrame(2,i)) = prob_to_log_odds(probOcc);
endfor

end
