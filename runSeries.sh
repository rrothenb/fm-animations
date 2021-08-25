for frame in `seq 101 100 1500`
do
  time ./run.sh $frame 10000000
done
