for frame in `seq 0 1023`
do
  time ./run.sh $1 $frame 10000000
done
