function [X,Y] = bresenham2(mycoords)

% BRESENHAM: Generate a line profile of a 2d image 
%            using Bresenham's algorithm
% [myline,mycoords] = bresenham(mymat,mycoords,dispFlag)
%
% - For a demo purpose, try >> bresenham();
%
% - mymat is an input image matrix.
%
% - mycoords is coordinate of the form: [x1, y1; x2, y2]
%   which can be obtained from ginput function
%
% Author: N. Chattrapiban
%
% Ref: nprotech: Chackrit Sangkaew; Citec
% Ref: http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
% 
% See also: tut_line_algorithm

% round�����������������ʵ������������
% [rob_x rob_y
%  lr_x  lr_y]
x = round(mycoords(:,1));
y = round(mycoords(:,2));
% �ж�delta x �� delta y �Ĳ���
steep = (abs(y(2)-y(1)) > abs(x(2)-x(1)));
% ���delta y �� delta x �� �򽻻�x��y
% swap�Ǳ�����ĩβ�Լ������.........
if steep, [x,y] = swap(x,y); end
% ���ִ��֮��һ����x(2)>=x(1)
if x(1)>x(2), 
    [x(1),x(2)] = swap(x(1),x(2));
    [y(1),y(2)] = swap(y(1),y(2));
end
% ��������ifִ�����֮��delta_x > delta_y��x(1)<=x(2)
% �������ߵ���״�Ǵ����µ����ϻ�����µ����ϵ�ֱ�ߣ��Ҿ���б��С�� 1 
% delxһ��>=0
delx = x(2)-x(1);
dely = abs(y(2)-y(1));
error = 0;
x_n = x(1);
y_n = y(1);
% ������ʽѡ��deltaС����һ������ y ����
if y(1) < y(2), ystep = 1; else ystep = -1; end 
for n = 1:delx+1
    if steep,
        X(n) = x_n;
        Y(n) = y_n;
    else
        X(n) = y_n;
        Y(n) = x_n;
    end    
    x_n = x_n + 1;
    error = error + dely;
    % ����λ����
    if bitshift(error,1) >= delx, % same as -> if 2*error >= delx, 
        y_n = y_n + ystep;
        error = error - delx;
    end    
end

% ���յõ�һЩ�������ߵĵ�����
temp = X;
X = Y;
Y = temp;


function [q,r] = swap(s,t)
% function SWAP
q = t; r = s;
