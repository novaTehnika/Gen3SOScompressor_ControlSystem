set datafile separator ","
set key autotitle columnhead outside right
set grid
set term pngcairo size 1200,900 font ",11"
do for [sc in "compress stiffstep discharge"] {
  set output sc.".png"
  set multiplot layout 3,1 title sc
  set ylabel "P (MPa)"
  plot sc.".csv" using 1:2 with lines lc "black" dt 2, for [i=3:8] "" using 1:i with lines
  set ylabel "velocity cmd (mm/s)"
  plot for [i=9:13] sc.".csv" using 1:i with lines
  set ylabel "force cmd / A (MPa)"; set xlabel "t (s)"
  plot sc.".csv" using 1:14 with lines
  unset xlabel
  unset multiplot
}
