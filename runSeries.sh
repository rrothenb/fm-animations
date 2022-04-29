for frame in `seq 20 47`
do
  time ./run.sh $1 $frame 25000000
done
