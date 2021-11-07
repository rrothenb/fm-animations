for frame in `seq 1 99`
do
  time ./run.sh $1 $frame 10000000
done
