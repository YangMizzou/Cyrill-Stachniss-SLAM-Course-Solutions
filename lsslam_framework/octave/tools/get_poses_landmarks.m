% extract the offset of the poses and the landmarks

function [poses, landmarks] = get_poses_landmarks(g)

poses = [];
landmarks = [];

for i = 1 : length(g.idLookup)
  dim = g.idLookup(i).dimension;
  offset = g.idLookup(i).offset;
  if (dim == 3)
    poses = [poses; offset];
  elseif (dim == 2)
    landmarks = [landmarks; offset];
  end
end

end
