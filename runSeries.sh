for frame in `seq 1 8`
do
  time ./run.sh $1 $frame 5000000
done
