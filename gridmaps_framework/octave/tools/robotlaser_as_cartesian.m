function points = robotlaser_as_cartesian(rl, maxRange, subsample)

numBeams = length(rl.ranges);
maxRange=min(maxRange, rl.maximum_range);
% apply the max range
idx = rl.ranges<maxRange & rl.ranges>0;

if (subsample)
	idx(2:2:end) = 0;
end

angles = linspace(rl.start_angle, rl.start_angle + numBeams*rl.angular_resolution, numBeams);
points = [rl.ranges(idx) .* cos(angles(idx)); rl.ranges(idx) .* sin(angles(idx)); ones(1, length(angles(idx)))];
transf = v2t(rl.laser_offset);

% apply the laser offset
points = transf * points;

end
