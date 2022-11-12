for frame in `seq 0 199`
do
  time ./run.sh $1 $frame 2500000
done
