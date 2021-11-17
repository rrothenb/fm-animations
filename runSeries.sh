for frame in `seq 1 720`
do
  time ./run.sh $1 $frame 5000000
done
