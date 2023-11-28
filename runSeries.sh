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
for frame in 597 601 618 619 620 623 649 650 666 671 672 705 706 707 715 736 737 753 766 767 782 797 798 799 801 802 816 834 847 880 884 885 898 946 977 991 1018 1095 1147 1148 1174 1180 1228 1285 1294 1342 1343 1394 1396 1408 1409 1410 1411 1416 1427 1429 1434 1459 1477 1480 1497 1523 1530 1534 1541 1542
do
    time ./run.sh $options $1 $frame $2
done
