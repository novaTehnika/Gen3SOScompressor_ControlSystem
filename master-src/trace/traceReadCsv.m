function d = traceReadCsv(file)
%TRACEREADCSV Read a trace CSV (traceWriteCsv) into a struct.
%
%   d = traceReadCsv(file)
%
%   d.meta   struct of the "# key: value" lines; a value is numeric when
%            it reads as a number, else char
%   d.names  cell row of the header's column names
%   d.data   N x n matrix of the records
%   d.<name> each column as a column vector (0 x 1 when there are no
%            records), named by the header
%   d.file   the file name
%
%   Columns beyond those traceLayout names are kept, so a file from a
%   later version of the recorder still reads. Errors when the file
%   cannot be opened, has no header line, or a record's field count
%   differs from the header's.

fid = fopen(file, 'r');
if fid < 0
    error('traceReadCsv: cannot open %s', file);
end
cleaner = onCleanup(@() fclose(fid));

meta = struct();
header = '';
while true
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    line = strtrim(line);
    if isempty(line)
        continue
    end
    if line(1) == '#'
        tok = regexp(line, '^#\s*(\w+):\s*(.*)$', 'tokens', 'once');
        if ~isempty(tok)
            val = str2double(tok{2});
            if isnan(val) || ~isreal(val)
                meta.(tok{1}) = tok{2};
            else
                meta.(tok{1}) = val;
            end
        end
    else
        header = line;
        break
    end
end
if isempty(header)
    error('traceReadCsv: %s has no header line', file);
end
names = strtrim(strsplit(header, ','));
n = numel(names);
for k = 1:n
    if ~isvarname(names{k})
        error('traceReadCsv: %s: column name "%s" is not a name', file, names{k});
    end
end

% The first record line fixes the field count before textscan reads on.
data = zeros(0, n);
pos = ftell(fid);
first = '';
while true
    line = fgetl(fid);
    if ~ischar(line)
        break
    end
    if ~isempty(strtrim(line))
        first = line;
        break
    end
end
if ~isempty(first)
    nFields = numel(strfind(first, ',')) + 1;
    if nFields ~= n
        error('traceReadCsv: %s: %d columns, header names %d', file, nFields, n);
    end
    fseek(fid, pos, 'bof');
    c = textscan(fid, repmat('%f', 1, n), 'Delimiter', ',', 'CollectOutput', true);
    if ~isempty(c) && ~isempty(c{1})
        data = c{1};
    end
    if size(data, 2) ~= n
        error('traceReadCsv: %s: %d columns, header names %d', file, size(data, 2), n);
    end
end

d.meta = meta;
d.names = names;
d.data = data;
for k = 1:n
    d.(names{k}) = data(:, k);
end
d.file = file;
end
