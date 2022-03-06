for frame in `seq 0 767`
do
  time ./run.sh $1 $frame 1500000
done
