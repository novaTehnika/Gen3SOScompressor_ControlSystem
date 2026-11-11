function bytes = ctlEncode(msg)
%CTLENCODE One message between the app and the controller as bytes.
%   bytes = ctlEncode(msg) is the JSON text of the struct msg followed by
%   a line feed, as a uint8 row: one message per line on the local TCP
%   connection (ctlDecode reads them back). NaN and Inf become null.

bytes = uint8([jsonencode(msg), char(10)]);
end
