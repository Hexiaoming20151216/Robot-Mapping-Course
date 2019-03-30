function points = robotlaser_as_cartesian(rl, maxRange = 15, subsample = false)

% ���������
numBeams = length(rl.ranges);
% ��������
maxRange=min(maxRange, rl.maximum_range);
% apply the max range
% ������������ε� & ����������Ϊ��ʲô����ĸ�����..........
% ��ߵõ�С��maxRange���������ұߵõ�>0�����������Ľ����ͬʱ������������
idx = rl.ranges<maxRange    &     rl.ranges>0;

if (subsample)
	idx(2:2:end) = 0;
endif

% ����idxȡ������Ҫ�������
% ���������Ҫ������ݶ�Ӧ�ļ�����ת�Ƕȣ�linspace(start, end, num)��������num����start��end������
angles = linspace(rl.start_angle, rl.start_angle + numBeams*rl.angular_resolution, numBeams)(idx);
% ���Ƕ�-����ת���ɻ���������ϵ�µ��������
points = [rl.ranges(idx) .* cos(angles); rl.ranges(idx) .* sin(angles); ones(1, length(angles))];
% ����ƫ�ã�Ӧ����ʲô����ֵ����������õģ��鿴���ݶ��Ǽ���Ϊ�������
% ת������α任�������޽ӽ���λ��
transf = v2t(rl.laser_offset);

% apply the laser offset
% �õ�����У������ڻ���������ϵ�µ���ε�����
points = transf * points;

end
