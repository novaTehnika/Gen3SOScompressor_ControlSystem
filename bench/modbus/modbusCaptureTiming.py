#!/usr/bin/env python3
"""Split a Modbus TCP capture into controller time and client time.

Usage: python3 modbusCaptureTiming.py capture.pcapng [controller-ip]

Needs tshark. For every transaction on port 502 the script reports the
delay from the request frame to the controller's TCP ACK and to its Modbus
response (the controller's share), and the gap from a response to the
client's next request (the client's share). Histograms show the service
tick: an ACK delay spread evenly over 0 to T ms means requests wait for a
T ms service cycle.
"""
import collections
import statistics
import subprocess
import sys


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    plc = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.1'
    fields = ['frame.time_epoch', 'ip.src', 'tcp.flags.str', 'tcp.len',
              'mbtcp.trans_id', 'modbus.func_code', 'tcp.analysis.flags']
    cmd = ['tshark', '-r', path, '-Y', 'tcp.port==502', '-T', 'fields',
           '-E', 'separator=|']
    for f in fields:
        cmd += ['-e', f]
    rows = [l.split('|') for l in subprocess.run(
        cmd, check=True, capture_output=True, text=True).stdout.splitlines()]

    req = {}
    ack = {}
    cur = None
    last_resp = None
    d_ack, d_resp, d_ack_resp, gaps = [], [], [], []
    per_fc = collections.defaultdict(list)
    flagged = collections.Counter()
    for r in rows:
        t = float(r[0])
        src, flags, ln, tid, fc, an = r[1], r[2], int(r[3] or 0), r[4], r[5], r[6]
        if an:
            flagged.update(an.split(','))
        if src != plc and ln > 0 and tid:
            req[tid] = t
            cur = tid
            if last_resp is not None:
                gaps.append((t - last_resp) * 1e3)
        elif src == plc and ln == 0 and cur in req and 'A' in flags:
            ack[cur] = t
            d_ack.append((t - req[cur]) * 1e3)
        elif src == plc and ln > 0 and tid in req:
            dt = (t - req.pop(tid)) * 1e3
            d_resp.append(dt)
            per_fc[fc].append(dt)
            if tid in ack:
                d_ack_resp.append((t - ack.pop(tid)) * 1e3)
            last_resp = t

    print(f'{len(d_resp)} transactions, {len(req)} unanswered, '
          f'tcp analysis flags: {dict(flagged) or "none"}')
    stats('controller: request -> ACK', d_ack)
    stats('controller: request -> response', d_resp)
    for fc in sorted(per_fc, key=int):
        stats(f'  FC{fc}', per_fc[fc])
    stats('controller: ACK -> response', d_ack_resp)
    stats('client: response -> next request', gaps)
    hist('controller request -> ACK, 0.5 ms bins', d_ack, 0.5)
    hist('controller ACK -> response, 0.5 ms bins', d_ack_resp, 0.5)
    hist('client response -> next request, 1 ms bins', gaps, 1.0)


def stats(name, x):
    x = sorted(x)
    n = len(x)
    if not n:
        print(f'{name}: none')
        return
    pct = lambda q: x[min(n - 1, int(q * n))]
    print(f'{name:36s} n={n:5d} median {statistics.median(x):6.2f} '
          f'p95 {pct(.95):6.2f} p99 {pct(.99):6.2f} max {x[-1]:6.2f} ms')


def hist(name, x, width):
    h = collections.Counter(round(int(v / width) * width, 2) for v in x)
    print(name)
    print('  ' + ' '.join(f'{k:g}:{c}' for k, c in sorted(h.items())))


if __name__ == '__main__':
    main()
