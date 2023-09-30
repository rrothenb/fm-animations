fixedargs=`echo $* | sed "s/--aspect-ratio/-a/"`
fixedargs=`echo $fixedargs | sed "s/--height/-h/"`
fixedargs=`echo $fixedargs | sed "s/--samples/-s/"`
options=""
while getopts 'a:h:s:' opt $fixedargs; do
  case "$opt" in
    a)
      options="$options -a $OPTARG"
      ;;

    h)
      options="$options -h $OPTARG"
      ;;

    s)
      options="$options -s $OPTARG"
      ;;

    ?)
      echo "Usage: $(basename $0) [-r Number of rows] [-f First row] series frame triangles"
      exit 1
      ;;
  esac
done
shift "$(($OPTIND -1))"
for frame in 431 432 433 451 466 467 471 502 508 509 545 558
do
    time ./run.sh $options 148 $frame $2
    mv mitsuba.ply paper.ply
    mv mitsuba.blend.rgbe paper.rgbe
    rm 148-$frame.jpg
    time ./run.sh $options $1 $frame $2
done
