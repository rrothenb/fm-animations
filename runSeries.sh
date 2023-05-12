for frame in `seq 45480 45600`
do
    time ./run.sh $1 $frame 50000000
done
