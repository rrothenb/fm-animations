for frame in `seq 0 3071`
do
  time ./run.sh $1 $frame 2500000
done
