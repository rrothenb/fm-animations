for frame in `seq 50 250 1000`
do
  time ./run.sh $1 $frame 500000
done
