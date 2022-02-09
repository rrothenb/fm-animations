for frame in `seq 0 2047`
do
  time ./run.sh $1 $frame 1000000
done
