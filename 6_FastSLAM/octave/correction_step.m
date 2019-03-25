function particles = correction_step(particles, z)

% Weight the particles according to the current map of the particle
% and the landmark observations z.
% z: struct array containing the landmark observations.
% Each observation z(j) has an id z(j).id, a range z(j).range, and a bearing z(j).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.

% Number of particles
numParticles = length(particles);

% Number of measurements in this time step
% 观测到的landmark数量
m = size(z, 2);

% TODO: Construct the sensor noise matrix Q_t (2 x 2)
% 给观测加上噪声
Q_t = 0.01*eye(2);
% process each particle
% 1. 根据上一时刻产生当前时刻的粒子
% 2. 用当前时刻的粒子表示的位姿对landmark进行EKF
% 3. 使用预测观测值和真是观测值之间的差异更新粒子权重
for i = 1:numParticles
  % 获取粒子预测的机器人位姿
  robot = particles(i).pose;
  % process each measurement
  for j = 1:m
    % Get the id of the landmark corresponding to the j-th observation
    % particles(i).landmarks(l) is the EKF for this landmark
    l = z(j).id;

    % The (2x2) EKF of the landmark is given by
    % its mean particles(i).landmarks(l).mu
    % and by its covariance particles(i).landmarks(l).sigma

    % If the landmark is observed for the first time:
    % 第一次观测到
    if (particles(i).landmarks(l).observed == false)

      % TODO: Initialize its position based on the measurement and the current robot pose:
	    % 根据预测的机器人状态预测landmark的观测值
      particles(i).landmarks(l).mu = [robot(1)+z(j).range*cos(z(j).bearing + robot(3)); robot(2) + z(j).range*sin(z(j).bearing + robot(3))];

      % get the Jacobian with respect to the landmark position
      [h, H] = measurement_model(particles(i), z(j));

      % TODO: initialize the EKF for this landmark

      particles(i).landmarks(l).sigma = H\Q_t*inv(H)';

      % Indicate that this landmark has been observed
      particles(i).landmarks(l).observed = true;
    % 非第一次观测到
    else

      % get the expected measurement
      % 根据粒子预测的位姿和H
      % 对landmark使用EKF进行预测更新
      [expectedZ, H] = measurement_model(particles(i), z(j));

      % TODO: compute the measurement covariance
	
	Q = H*particles(i).landmarks(l).sigma*H' + Q_t;

      % TODO: calculate the Kalman gain
	
	K = particles(i).landmarks(l).sigma*H'/Q;

      % TODO: compute the error between the z and expectedZ (remember to normalize the angle)
	
	z_diff = [z(j).range;z(j).bearing] - expectedZ;
	z_diff(2) = normalize_angle(z_diff(2));

      % TODO: update the mean and covariance of the EKF for this landmark

	particles(i).landmarks(l).mu = particles(i).landmarks(l).mu + K*z_diff;
	particles(i).landmarks(l).sigma = (eye(2) - K*H)*particles(i).landmarks(l).sigma;

      % TODO: compute the likelihood of this observation, multiply with the former weight
      %       to account for observing several features in one time step
	particles(i).weight = particles(i).weight*1/sqrt(det(2*pi*Q))*exp(-1/2*z_diff'/Q*z_diff);
    end

  end % measurement loop
end % particle loop

end
