function writeLoopCsv(fname, R, names, p)
% writeLoopCsv - write traces of several runs to one CSV for gnuplot:
% t, target, P_<name>..., u_<name>... (MPa, mm/s, or MPa of force/A),
% every fifth scan.
  n = numel(R); t = R{1}.t; M = [t R{1}.tgt/1e6];
  hdr = 't,target';
  for i = 1:n, M = [M R{i}.P/1e6]; hdr = [hdr ',P_' names{i}]; end
  for i = 1:n
    if R{i}.mode == 'V', M = [M R{i}.u*1e3]; else M = [M R{i}.u/p.A/1e6]; end
    hdr = [hdr ',u_' names{i}];
  end
  fid = fopen(fname, 'w'); fprintf(fid, '%s\n', hdr); fclose(fid);
  dlmwrite(fname, M(1:5:end,:), '-append', 'precision', '%.5g');
end
