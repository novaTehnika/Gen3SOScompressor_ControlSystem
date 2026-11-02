function traceWriteCsv(file, data, meta)
%TRACEWRITECSV Write a trace to a CSV file: the metadata as "# key: value"
%   lines, then the header line of column names (traceColumnNames) and one
%   line per record.
%
%   traceWriteCsv(file, data, meta)
%
%   file   path of the file, overwritten
%   data   N x traceLayout().nCol matrix of records, all values finite;
%          N may be 0, which writes the metadata and header only
%   meta   struct from traceMeta; a numeric field is written with %.10g,
%          a char field as it is (newlines become spaces)
%
%   Errors when data has the wrong number of columns, holds a NaN or Inf,
%   or the file cannot be opened. A gap in the recording shows as a jump
%   of the seq column, never as a missing or NaN row.

L = traceLayout();
if ~isnumeric(data) || size(data, 2) ~= L.nCol
    error('traceWriteCsv: data needs %d columns, got %d', L.nCol, size(data, 2));
end
if ~all(isfinite(data(:)))
    error('traceWriteCsv: data holds a NaN or Inf');
end
fid = fopen(file, 'w');
if fid < 0
    error('traceWriteCsv: cannot open %s', file);
end
keys = fieldnames(meta);
for k = 1:numel(keys)
    v = meta.(keys{k});
    if ischar(v)
        txt = regexprep(v, '[\r\n]+', ' ');
    else
        txt = sprintf('%.10g', double(v));
    end
    fprintf(fid, '# %s: %s\n', keys{k}, txt);
end
fprintf(fid, '%s\n', strjoin(traceColumnNames(), ','));
if ~isempty(data)
    fprintf(fid, [repmat('%.10g,', 1, L.nCol - 1) '%.10g\n'], double(data)');
end
fclose(fid);
end
