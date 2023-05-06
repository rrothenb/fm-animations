for frame in `seq 0 999`
do
    time ./run.sh $1 $frame 5000000
done
