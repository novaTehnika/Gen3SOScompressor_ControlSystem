#!/usr/bin/env python3
"""Split a Modbus TCP capture into controller time and client time.

Usage: python3 modbusCaptureTiming.py capture.pcapng [controller-ip]

Needs tshark. For every transaction on port 502 the script reports the
delay from the request frame to the controller's TCP ACK and to its Modbus
response (the controller's share), and the gap from a response to the
client's next request (the client's share). Histograms show the service
tick: an ACK delay spread evenly over 0 to T ms means requests wait for a
T ms service cycle.

It also breaks the controller's time down by function code and register
count (a modbusCountSweep run): per count, the request -> response time,
the share of responses sent in the same tick as the ACK (ACK -> response
under half a tick), and per function code the slope of the median
against the count, the controller's time per register for
slave-sim/modbusLinkConfig.m (perRegister).
"""
import collections
import statistics

TICK_MS = 4.0       # the controller's service tick (G_cfgScanTime)
import subprocess
import sys


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    path = sys.argv[1]
    plc = sys.argv[2] if len(sys.argv) > 2 else '192.168.1.1'
    fields = ['frame.time_epoch', 'ip.src', 'tcp.flags.str', 'tcp.len',
              'mbtcp.trans_id', 'modbus.func_code', 'tcp.analysis.flags',
              'modbus.word_cnt']
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
    count = {}                      # transaction -> (fc, registers)
    per_count = collections.defaultdict(list)
    same_tick = collections.defaultdict(list)
    flagged = collections.Counter()
    for r in rows:
        t = float(r[0])
        src, flags, ln, tid, fc, an = r[1], r[2], int(r[3] or 0), r[4], r[5], r[6]
        words = r[7] if len(r) > 7 else ''
        if an:
            flagged.update(an.split(','))
        if src != plc and ln > 0 and tid:
            req[tid] = t
            cur = tid
            if words:
                count[tid] = (int(fc.split(',')[0]), int(words.split(',')[0]))
            if last_resp is not None:
                gaps.append((t - last_resp) * 1e3)
        elif src == plc and ln == 0 and cur in req and 'A' in flags:
            ack[cur] = t
            d_ack.append((t - req[cur]) * 1e3)
        elif src == plc and ln > 0 and tid in req:
            dt = (t - req.pop(tid)) * 1e3
            d_resp.append(dt)
            per_fc[fc].append(dt)
            key = count.pop(tid, None)
            if key:
                per_count[key].append(dt)
            if tid in ack:
                a = (t - ack.pop(tid)) * 1e3
                d_ack_resp.append(a)
                if key:
                    same_tick[key].append(a < TICK_MS / 2)
            last_resp = t

    print(f'{len(d_resp)} transactions, {len(req)} unanswered, '
          f'tcp analysis flags: {dict(flagged) or "none"}')
    stats('controller: request -> ACK', d_ack)
    stats('controller: request -> response', d_resp)
    for fc in sorted(per_fc, key=int):
        stats(f'  FC{fc}', per_fc[fc])
    stats('controller: ACK -> response', d_ack_resp)
    stats('client: response -> next request', gaps)
    if per_count:
        print('\nby function code and register count (controller, '
              'request -> response):')
        for key in sorted(per_count):
            share = same_tick.get(key, [])
            st = (f' same tick {100 * sum(share) / len(share):5.1f} %'
                  if share else '')
            stats(f'  FC{key[0]:02d} x {key[1]:3d} registers', per_count[key])
            if st:
                print(f'{"":38s}{st}')
        for fc in sorted({k[0] for k in per_count}):
            pts = [(k[1], statistics.median(v)) for k, v in per_count.items()
                   if k[0] == fc]
            if len(pts) >= 2:
                n = len(pts)
                mx = sum(c for c, _ in pts) / n
                my = sum(m for _, m in pts) / n
                sxx = sum((c - mx) ** 2 for c, _ in pts)
                slope = sum((c - mx) * (m - my) for c, m in pts) / sxx
                # A slope within the scatter can come out negative; the
                # model's term is not
                print(f'FC{fc:02d}: median {my - slope * mx:.3f} ms '
                      f'+ {slope * 1e3:.2f} us per register; '
                      f'modbusLinkConfig perRegister = '
                      f'{max(slope, 0.0) * 1e-3:.3g} s')
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
