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
for frame in 45 61 83 84 88 98 104 108 114 116 118 124 144 153 154 155
do
    time ./run.sh $options $1 $frame $2
done
