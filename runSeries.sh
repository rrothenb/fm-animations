for frame in $(seq 0 127)
do
  time ./run.sh "$1" "$frame" 100000
done
