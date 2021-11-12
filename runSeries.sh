for frame in `seq 1 1000`
do
  time ./run.sh $1 $frame 2500000
done
