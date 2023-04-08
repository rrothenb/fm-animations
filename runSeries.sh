for frame in `seq 129 3 255`
do
    time ./run.sh $1 $frame 100000000
done
