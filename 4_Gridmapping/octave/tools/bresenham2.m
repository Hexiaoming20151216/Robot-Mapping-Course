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

% round返回最近的整数，其实就是四舍五入
% [rob_x rob_y
%  lr_x  lr_y]
x = round(mycoords(:,1));
y = round(mycoords(:,2));
% 判断delta x 和 delta y 的差异
steep = (abs(y(2)-y(1)) > abs(x(2)-x(1)));
% 如果delta y 比 delta x 大， 则交换x，y
% swap是本函数末尾自己定义的.........
if steep, [x,y] = swap(x,y); end
% 这句执行之后一定有x(2)>=x(1)
if x(1)>x(2), 
    [x(1),x(2)] = swap(x(1),x(2));
    [y(1),y(2)] = swap(y(1),y(2));
end
% 以上两个if执行完毕之后，delta_x > delta_y，x(1)<=x(2)
% 即最后的线的形状是从右下到左上或从左下到右上的直线，且绝度斜率小于 1 
% delx一定>=0
delx = x(2)-x(1);
dely = abs(y(2)-y(1));
error = 0;
x_n = x(1);
y_n = y(1);
% 步进方式选择delta小的那一个，即 y 方向
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
    % 左移位操作
    if bitshift(error,1) >= delx, % same as -> if 2*error >= delx, 
        y_n = y_n + ystep;
        error = error - delx;
    end    
end

% 最终得到一些列沿着线的点坐标
temp = X;
X = Y;
Y = temp;


function [q,r] = swap(s,t)
% function SWAP
q = t; r = s;
