set datafile separator ","
set key autotitle columnhead outside right
set grid
set term pngcairo size 1200,900 font ",11"
set output "valve_dip.png"
set title "Pressure dip vs valve opening time, 1 mL/s final flow, liquid-only chamber at 10 MPa"
set xlabel "valve opening time (s)"; set ylabel "dip (MPa)"
set logscale x; set xrange [0.05:15]; set xtics add ("step" 0.07)
plot for [i=2:8] "valve_dip.csv" using ($1==0?0.07:$1):i with linespoints
unset logscale; set xrange [*:*]
set output "valve03.png"
set multiplot layout 2,1 title "valve opens over 0.3 s to 1 mL/s"
set xlabel "t (s)"; set ylabel "P (MPa)"; set xrange [0:6]
plot "valve03.csv" using 1:2 with lines lc "black" dt 2, for [i=3:9] "" using 1:i with lines
set ylabel "velocity cmd (mm/s)"
plot for [i=10:15] "valve03.csv" using 1:i with lines
unset multiplot
