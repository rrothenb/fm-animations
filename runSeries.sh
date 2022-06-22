for frame in `seq 0 48 767`
do
  time ./run.sh $1 $frame 10000000
done
