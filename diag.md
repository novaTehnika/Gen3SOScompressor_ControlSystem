>> opts.probe = true;                                   % PRG_StepProbe loaded
[path, plan] = charStopPath(struct('fillVolume_ml',187,'chargeratio',353/187));                       % prints the budget table
r = charRun('stops', path, opts);
rs = charStopDistance([r.capFiles {r.file}]);        % D and T0 per direction
rl = charDriveLag([r.capFiles {r.file}]);            % the lag under load
rk = charBulkModulus(r.file, struct('chargeRatio', 2));   % the ramps
one-pass stopping-distance path: 5 levels, speeds [1 2 5 7] mm/s, charge 2:1 air, fill 343 mL
 P MPa   k MPa/mm     x mm     budget      v     d mm  stop mm     rise
   2.0       0.21    274.0       2.00      1    4.920    0.060     2.00
   2.0       0.21    274.0       2.00      2    4.840    0.140     2.00
   2.0       0.21    274.0       2.00      5    4.480    0.500     2.00
   2.0       0.21    274.0       2.00      7    4.140    0.840     2.00
   4.0       0.74    278.7       2.00      1    1.837    0.060     2.00
   4.0       0.74    278.7       2.00      2    1.757    0.140     2.00
   4.0       0.74    278.7       2.00      5    1.397    0.500     2.00
   4.0       0.74    278.7       2.00      7    1.057    0.840     2.00
   6.0       1.46    280.3       2.00      1    1.038    0.060     2.00
   6.0       1.46    280.3       2.00      2    0.958    0.140     2.00
   6.0       1.46    280.3       2.00      5  skipped
   6.0       1.46    280.3       2.00      7  skipped
   8.0       2.23    281.1       2.00      1    0.713    0.060     2.00
   8.0       2.23    281.1       2.00      2    0.633    0.140     2.00
   8.0       2.23    281.1       2.00      5  skipped
   8.0       2.23    281.1       2.00      7  skipped
  10.0       2.96    281.6       1.00      1    0.260    0.060     1.00
  10.0       2.96    281.6       1.00      2  skipped
  10.0       2.96    281.6       1.00      5  skipped
  10.0       2.96    281.6       1.00      7  skipped
64 steps, 26 stops, about 88 s of motion
charRun stops: 64 steps, guards 11.0 MPa, 0.5..364.0 mm, probe true
  step 1: ramp 2 mm/s to 2 MPa

RUN STOPPED: master status 10 during step 1, phase run; master status 10, fault code 4

step type      target              seconds   x start     x end   P start     P end
1    ramp      2 mm/s to 2 MPa         7.0 225.000 mm 229.514 mm  0.003 MPa  0.002 MPa
2    dwell     0 mm/s              not run
3    move      1 mm/s for 4.91975 mm  not run
4    dwell     0 mm/s              not run
5    move      -1 mm/s for 4.91975 mm  not run
6    dwell     0 mm/s              not run
7    move      2 mm/s for 4.83975 mm  not run
8    dwell     0 mm/s              not run
9    move      -2 mm/s for 4.83975 mm  not run
10   dwell     0 mm/s              not run
11   move      5 mm/s for 4.47975 mm  not run
12   dwell     0 mm/s              not run
13   move      -5 mm/s for 4.47975 mm  not run
14   dwell     0 mm/s              not run
15   move      7 mm/s for 4.13975 mm  not run
16   dwell     0 mm/s              not run
17   move      -7 mm/s for 4.13975 mm  not run
18   dwell     0 mm/s              not run
19   ramp      2 mm/s to 4 MPa     not run
20   dwell     0 mm/s              not run
21   move      1 mm/s for 1.83712 mm  not run
22   dwell     0 mm/s              not run
23   move      -1 mm/s for 1.83712 mm  not run
24   dwell     0 mm/s              not run
25   move      2 mm/s for 1.75712 mm  not run
26   dwell     0 mm/s              not run
27   move      -2 mm/s for 1.75712 mm  not run
28   dwell     0 mm/s              not run
29   move      5 mm/s for 1.39712 mm  not run
30   dwell     0 mm/s              not run
31   move      -5 mm/s for 1.39712 mm  not run
32   dwell     0 mm/s              not run
33   move      7 mm/s for 1.05712 mm  not run
34   dwell     0 mm/s              not run
35   move      -7 mm/s for 1.05712 mm  not run
36   dwell     0 mm/s              not run
37   ramp      2 mm/s to 6 MPa     not run
38   dwell     0 mm/s              not run
39   move      1 mm/s for 1.03755 mm  not run
40   dwell     0 mm/s              not run
41   move      -1 mm/s for 1.03755 mm  not run
42   dwell     0 mm/s              not run
43   move      2 mm/s for 0.957549 mm  not run
44   dwell     0 mm/s              not run
45   move      -2 mm/s for 0.957549 mm  not run
46   dwell     0 mm/s              not run
47   ramp      2 mm/s to 8 MPa     not run
48   dwell     0 mm/s              not run
49   move      1 mm/s for 0.71291 mm  not run
50   dwell     0 mm/s              not run
51   move      -1 mm/s for 0.71291 mm  not run
52   dwell     0 mm/s              not run
53   move      2 mm/s for 0.63291 mm  not run
54   dwell     0 mm/s              not run
55   move      -2 mm/s for 0.63291 mm  not run
56   dwell     0 mm/s              not run
57   ramp      2 mm/s to 10 MPa    not run
58   dwell     0 mm/s              not run
59   move      1 mm/s for 0.259546 mm  not run
60   dwell     0 mm/s              not run
61   move      -1 mm/s for 0.259546 mm  not run
62   dwell     0 mm/s              not run
63   ramp      -2 mm/s to 2 MPa    not run
64   dwell     0 mm/s              not run
0 captures, exchange failures 0, probe failures 0, exchange p99 226.5 ms
saved C:\Users\Admin\Documents\GitHub\Gen3SOScompressor\bench\common\captures\stops_char_20261030_215040.csv
Error using charStopDistance (line 85)
charStopDistance: no stop (a reference step to zero from at least 0.3 mm/s) in 1 file(s)


>> opts.probe = false;                                   % PRG_StepProbe loaded
[path, plan] = charStopPath(struct('fillVolume_ml',187,'chargeratio',353/187));                       % prints the budget table
r = charRun('stops', path, opts);
rs = charStopDistance([r.capFiles {r.file}]);        % D and T0 per direction
rl = charDriveLag([r.capFiles {r.file}]);            % the lag under load
rk = charBulkModulus(r.file, struct('chargeRatio', 2));   % the ramps
one-pass stopping-distance path: 5 levels, speeds [1 2 5 7] mm/s, charge 2:1 air, fill 343 mL
 P MPa   k MPa/mm     x mm     budget      v     d mm  stop mm     rise
   2.0       0.21    274.0       2.00      1    4.920    0.060     2.00
   2.0       0.21    274.0       2.00      2    4.840    0.140     2.00
   2.0       0.21    274.0       2.00      5    4.480    0.500     2.00
   2.0       0.21    274.0       2.00      7    4.140    0.840     2.00
   4.0       0.74    278.7       2.00      1    1.837    0.060     2.00
   4.0       0.74    278.7       2.00      2    1.757    0.140     2.00
   4.0       0.74    278.7       2.00      5    1.397    0.500     2.00
   4.0       0.74    278.7       2.00      7    1.057    0.840     2.00
   6.0       1.46    280.3       2.00      1    1.038    0.060     2.00
   6.0       1.46    280.3       2.00      2    0.958    0.140     2.00
   6.0       1.46    280.3       2.00      5  skipped
   6.0       1.46    280.3       2.00      7  skipped
   8.0       2.23    281.1       2.00      1    0.713    0.060     2.00
   8.0       2.23    281.1       2.00      2    0.633    0.140     2.00
   8.0       2.23    281.1       2.00      5  skipped
   8.0       2.23    281.1       2.00      7  skipped
  10.0       2.96    281.6       1.00      1    0.260    0.060     1.00
  10.0       2.96    281.6       1.00      2  skipped
  10.0       2.96    281.6       1.00      5  skipped
  10.0       2.96    281.6       1.00      7  skipped
64 steps, 26 stops, about 88 s of motion
charRun stops: 64 steps, guards 11.0 MPa, 0.5..364.0 mm, probe false

RUN STOPPED: master status 10 during step 0, phase wait; master status 10, fault code 4

step type      target              seconds   x start     x end   P start     P end
1    ramp      2 mm/s to 2 MPa     not run
2    dwell     0 mm/s              not run
3    move      1 mm/s for 4.91975 mm  not run
4    dwell     0 mm/s              not run
5    move      -1 mm/s for 4.91975 mm  not run
6    dwell     0 mm/s              not run
7    move      2 mm/s for 4.83975 mm  not run
8    dwell     0 mm/s              not run
9    move      -2 mm/s for 4.83975 mm  not run
10   dwell     0 mm/s              not run
11   move      5 mm/s for 4.47975 mm  not run
12   dwell     0 mm/s              not run
13   move      -5 mm/s for 4.47975 mm  not run
14   dwell     0 mm/s              not run
15   move      7 mm/s for 4.13975 mm  not run
16   dwell     0 mm/s              not run
17   move      -7 mm/s for 4.13975 mm  not run
18   dwell     0 mm/s              not run
19   ramp      2 mm/s to 4 MPa     not run
20   dwell     0 mm/s              not run
21   move      1 mm/s for 1.83712 mm  not run
22   dwell     0 mm/s              not run
23   move      -1 mm/s for 1.83712 mm  not run
24   dwell     0 mm/s              not run
25   move      2 mm/s for 1.75712 mm  not run
26   dwell     0 mm/s              not run
27   move      -2 mm/s for 1.75712 mm  not run
28   dwell     0 mm/s              not run
29   move      5 mm/s for 1.39712 mm  not run
30   dwell     0 mm/s              not run
31   move      -5 mm/s for 1.39712 mm  not run
32   dwell     0 mm/s              not run
33   move      7 mm/s for 1.05712 mm  not run
34   dwell     0 mm/s              not run
35   move      -7 mm/s for 1.05712 mm  not run
36   dwell     0 mm/s              not run
37   ramp      2 mm/s to 6 MPa     not run
38   dwell     0 mm/s              not run
39   move      1 mm/s for 1.03755 mm  not run
40   dwell     0 mm/s              not run
41   move      -1 mm/s for 1.03755 mm  not run
42   dwell     0 mm/s              not run
43   move      2 mm/s for 0.957549 mm  not run
44   dwell     0 mm/s              not run
45   move      -2 mm/s for 0.957549 mm  not run
46   dwell     0 mm/s              not run
47   ramp      2 mm/s to 8 MPa     not run
48   dwell     0 mm/s              not run
49   move      1 mm/s for 0.71291 mm  not run
50   dwell     0 mm/s              not run
51   move      -1 mm/s for 0.71291 mm  not run
52   dwell     0 mm/s              not run
53   move      2 mm/s for 0.63291 mm  not run
54   dwell     0 mm/s              not run
55   move      -2 mm/s for 0.63291 mm  not run
56   dwell     0 mm/s              not run
57   ramp      2 mm/s to 10 MPa    not run
58   dwell     0 mm/s              not run
59   move      1 mm/s for 0.259546 mm  not run
60   dwell     0 mm/s              not run
61   move      -1 mm/s for 0.259546 mm  not run
62   dwell     0 mm/s              not run
63   ramp      -2 mm/s to 2 MPa    not run
64   dwell     0 mm/s              not run
0 captures, exchange failures 0, probe failures 0, exchange p99 87.2 ms
saved C:\Users\Admin\Documents\GitHub\Gen3SOScompressor\bench\common\captures\stops_char_20261030_220520.csv
Error using charStopDistance (line 85)
charStopDistance: no stop (a reference step to zero from at least 0.3 mm/s) in 1 file(s)