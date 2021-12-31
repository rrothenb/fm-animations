for frame in `seq 1 16`
do
  time ./run.sh $1 $frame 1000000
done
