# num of packets received that are from TX / num of packets transmitted by TX

set terminal postscript eps size 6,3 color  "Helvetica" 35
set output "snr.eps"

set key top center
#set key at screen 15, 90

set grid

set ylabel "SNR (dB)" font "Helvetica-Bold,25
set yrange [0:50]
set xlabel "soil depth (cm)" font "Helvetica-Bold,25
#set format x "%.1f"
set key right outside
plot "X4.dat" t "7.29Ghz" w lp lc 1 lt 1 lw 5 pt 0 ps 2, \
     "X2.dat" t "5.2Ghz"  w lp lc 2 lt 1 lw 5 pt 0 ps 2, \
     "X1-low.dat" t "1.5Ghz"  w lp lc 3 lt 1 lw 5 pt 0 ps 2	


