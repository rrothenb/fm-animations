for frame in `seq 1 64`
do
  time ./run.sh $1 $frame 5000000
done
