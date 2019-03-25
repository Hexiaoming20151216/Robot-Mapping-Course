% resample the set of particles.
% A particle has a probability proportional to its weight to get
% selected. A good option for such a resampling method is the so-called low
% variance sampling, Probabilistic Robotics pg. 109
function newParticles = resample(particles)

numParticles = length(particles);

w = [particles.weight];

% normalize the weight
w = w / sum(w);

% consider number of effective particles, to decide whether to resample or not
useNeff = false;
%useNeff = true;
if useNeff
  neff = 1. / sum(w.^2);
  neff
  if neff > 0.5*numParticles
    newParticles = particles;
    for i = 1:numParticles
      newParticles(i).weight = w(i);
    end
    return;
  end
end

newParticles = struct;

% TODO: implement the low variance re-sampling

% the cummulative sum
% 将权重从左到右逐个累加
cs = cumsum(w);
% 取出最后一个累加值，也就是所有权值和的结果
weightSum = cs(length(cs));

% initialize the step and the current position on the roulette wheel
% 步长
step = weightSum / numParticles;
% 产生0-weighSum间的均匀分布的随机数
position = unifrnd(0, weightSum);
idx = 1;

% walk along the wheel to select the particles
for i = 1:numParticles
  position += step;
  if (position > weightSum)
    position -= weightSum;
    idx = 1;
  end
  % 找到position落在哪一个weight轮盘空间上了
  while (position > cs(idx))
    idx++;
  end
  % 产生新的粒子
  newParticles(i) = particles(idx);
  % 分配一样的权重
  newParticles(i).weight = 1/numParticles;
end

end
