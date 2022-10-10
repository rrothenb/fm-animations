for frame in `seq 32 255`
do
  time ./run.sh $1 $frame 5000000
done
