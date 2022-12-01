for frame in `seq 0 31`
do
  time ./run.sh $1 $frame 15000000
done
