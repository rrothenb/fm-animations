for frame in $(seq 1008 1043)
do
  time ./run.sh "$1" "$frame" 2500000
done
