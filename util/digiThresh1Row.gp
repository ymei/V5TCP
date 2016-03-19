# gnuplot -e "fname=xxx" xxx.gp
if(!exists("fname")) fname="a.dat"
if(!exists("ofname")) ofname="a.png"
if(!exists("GringPh")) GringPh="15mV"
if(!exists("row")) row=0

ppf = 0.1 / (125e3 / 2**7 / 72.0)
ulprob(x)=x>2.0 ? 2.0 : x

set output ofname
set term png size 550, 1000

set xrange [0.45:0.75]
set xlabel 'VR8B [V], 4BDAC=0xf'
set yrange [-1:73]
set ylabel 'col + probability'

set title sprintf("Row%2d,CSA\\_VREF=0.6V, ARST\\_VREF=0.8V, GringPulseHeight=%s", row, GringPh)

plot for [i=0:71] sprintf("<awk '$1==%d {print $1, $2, $3}' %s", i, fname) u ($0*0.001+0.45):(i+ulprob($2/($3*ppf))*0.9) w l t ''

unset output

