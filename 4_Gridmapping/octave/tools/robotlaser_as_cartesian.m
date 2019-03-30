function points = robotlaser_as_cartesian(rl, maxRange = 15, subsample = false)

% 激光测距点数
numBeams = length(rl.ranges);
% 最大测距距离
maxRange=min(maxRange, rl.maximum_range);
% apply the max range
% 这是两个代码段的 & 操作，还以为是什么新奇的负号呢..........
% 左边得到小于maxRange的索引，右边得到>0的索引，最后的结果是同时满足两个条件
idx = rl.ranges<maxRange    &     rl.ranges>0;

if (subsample)
	idx(2:2:end) = 0;
endif

% 根据idx取出符合要求的数据
% 计算出符合要求的数据对应的激光旋转角度，linspace(start, end, num)均匀生成num个从start到end的数据
angles = linspace(rl.start_angle, rl.start_angle + numBeams*rl.angular_resolution, numBeams)(idx);
% 将角度-距离转换成机器人坐标系下的齐次坐标
points = [rl.ranges(idx) .* cos(angles); rl.ranges(idx) .* sin(angles); ones(1, length(angles))];
% 激光偏置，应该是什么补偿值，矫正误差用的，查看数据都是几乎为零的数字
% 转换成齐次变换矩阵，无限接近单位阵
transf = v2t(rl.laser_offset);

% apply the laser offset
% 得到最后的校正后的在机器人坐标系下的齐次点坐标
points = transf * points;

end
