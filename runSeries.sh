for frame in `seq 91 511`
do
  time ./run.sh $1 $frame 15000000
done
