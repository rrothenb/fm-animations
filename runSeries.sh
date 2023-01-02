for frame in `seq 0 255`
do
    time ./run.sh $1 $frame 1000000
done
