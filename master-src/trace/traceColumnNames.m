function names = traceColumnNames()
%TRACECOLUMNNAMES Names of the trace record's data columns, in column
%   order: the fields of traceLayout().col sorted by their index. They
%   are the header line of a trace CSV (traceWriteCsv / traceReadCsv).

L = traceLayout();
fields = fieldnames(L.col);
index = cellfun(@(f) L.col.(f), fields);
[~, order] = sort(index);
names = fields(order)';
end
