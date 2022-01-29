for frame in `seq 500 800`
do
  time ./run.sh $1 $frame 10000000
done
