for frame in `seq 0 7`
do
  time ./run.sh $1 $frame 500000
done
