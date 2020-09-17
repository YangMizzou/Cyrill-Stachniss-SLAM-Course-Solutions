function plot_map(map, mapBox, robPoseMapFrame, poses, laserEndPntsMapFrame, gridSize, offset, t)

	    clf;
    map=map';
    imshow((ones(size(map)) - log_odds_to_prob(map)), 'InitialMagnification', 200);
% 	s= size(map); 
%         set(gcf, 'position', [50 50 s*6]);
%         set(gca, 'position', [.05 .05 .9 .9]); 
    hold on;
	traj = [poses(1:t,1)';poses(1:t,2)'];
	traj = world_to_map_coordinates(traj, gridSize, offset);
	plot(traj(1,:),traj(2,:),'g');  hold on;
	plot(robPoseMapFrame(1),robPoseMapFrame(2),'bo','markersize',5,'linewidth',4); hold on;
    plot(laserEndPntsMapFrame(1,:),laserEndPntsMapFrame(2,:),'ro','markersize',2); hold on;
    pause(0.1);
    
%   filename = sprintf('../plots/gridmap_%03d.png', t);
% 	print(f,filename, '-dpng');
end