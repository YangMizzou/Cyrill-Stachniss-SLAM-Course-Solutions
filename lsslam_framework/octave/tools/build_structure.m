% calculates the non-zero pattern of the Hessian matrix of a given graph

function idx = build_structure(g)

idx = [];

% elements along the diagonal
for i = 1 : length(g.idLookup)
  dim = g.idLookup(i).dimension;
  offset = g.idLookup(i).offset;
  [r,c] = meshgrid(offset+1 : offset+dim, offset+1 : offset+dim);
  idx = [idx; [vec(r) vec(c)]];
end

% off-diagonal elements
for eid = 1:length(g.edges)
  edge = g.edges(eid);
  if (strcmp(edge.type, 'P') ~= 0)
    [r,c] = meshgrid(edge.fromIdx:edge.fromIdx+2, edge.toIdx:edge.toIdx+2);
    idx = [idx; [vec(r) vec(c); vec(c) vec(r)]];
  elseif (strcmp(edge.type, 'L') ~= 0)
    [r,c] = meshgrid(edge.fromIdx:edge.fromIdx+2, edge.toIdx:edge.toIdx+1);
    idx = [idx; [vec(r) vec(c); vec(c) vec(r)]];
  end
end

%idx = sort(idx);

end
