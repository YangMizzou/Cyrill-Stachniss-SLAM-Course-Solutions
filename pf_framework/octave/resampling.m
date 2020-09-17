% Turn off pagination:
more off;

clear all;
randn('state',0);
rand('state',0);
% close all;

% how many particles
numParticles = 1000;

% initialize the particles array
particles = struct;
for i = 1:numParticles
  particles(i).weight = 1. / numParticles;
  particles(i).pose = normrnd([0 0]', [1 2]');
  particles(i).history = cell(0);
end


% re-weight the particles according to their distance to [0 0]
sigma = diag([0.2 0.2]);
for i = 1:numParticles
  particles(i).weight = exp(-1/2 * particles(i).pose' * inv(sigma) * particles(i).pose);
end

resampledParticles = resample(particles);

% plot the particles before (red) and after resampling (blue)
bpos = [particles.pose];
apos = [resampledParticles.pose];
figure;
plot(bpos(1,:), bpos(2,:), 'r+', 'MarkerSize', 5);
hold on;
plot(apos(1,:), apos(2,:), 'b*', 'MarkerSize', 5);
