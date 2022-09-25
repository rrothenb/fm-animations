for frame in `seq 1 20 201`
do
  time ./run.sh $1 $frame 3000000
done
