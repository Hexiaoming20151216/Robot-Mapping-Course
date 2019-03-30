function [pntsMap] = world_to_map_coordinates(pntsWorld, gridSize, offset)
% Convert points from the world coordinates frame to the map frame.
% pntsWorld is a matrix of N points with each column representing a point in world coordinates (meters).
% gridSize is the size of each grid in meters.
% offset = [offsetX; offsetY] is the offset that needs to be subtracted from a point
% when converting to map coordinates.
% pntsMap is a 2xN matrix containing the corresponding points in map coordinates.

% TODO: compute pntsMap
% repmat 重复产生矩阵，重复维度为后两个参数 M*N
% 实现对pntsWorld中的每个列向量减去offset，其中offset为列向量，并换算成栅格地图中的坐标
% floor向下取整
pntsMap = floor((pntsWorld - repmat(offset,1,size(pntsWorld,2)))./gridSize);

end
