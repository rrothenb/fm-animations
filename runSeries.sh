for frame in `seq 0 1535`
do
  time ./run.sh $1 $frame 2500000
done
