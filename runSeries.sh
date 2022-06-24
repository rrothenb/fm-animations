for frame in `seq 309 767`
do
  time ./run.sh $1 $frame 10000000
done
