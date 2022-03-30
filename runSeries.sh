for frame in `seq 0 69`
do
  time ./run.sh $1 $frame 5000000
done
