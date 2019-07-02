# initial VWC measurements, comercial vs radar, using topp eqn

set terminal postscript eps size 6,3 color  "Helvetica" 35
set output "vwc.eps"

set key top center
#set key at screen 15, 90

set grid

set ylabel "VWC (cm^3/cm^3)" font "Helvetica-Bold,25
set yrange [0:0.6] #0.6 is the highest VWC we should ever see, it's saturation for clay soils
set xrange [0:5] 
set xtics 1,1,4
set xlabel "moisture level" font "Helvetica-Bold,25
set key right outside
plot "radar.dat" t "radar measurement"  w lp lc 1 lt 1 lw 5 pt 7 ps 2, \
     "teros.dat" t "commercial sensor"  w lp lc 2 lt 1 lw 5 pt 9 ps 2



