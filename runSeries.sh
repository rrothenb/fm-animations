for frame in `seq 1 32`
do
  time ./run.sh $1 $frame 1500000
done
