for frame in `seq 163 719`
do
  time ./run.sh $1 $frame 5000000
done
