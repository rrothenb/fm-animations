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
for frame in 69 75 78 79 86 91 99 104 127 133 134 135 139 153 174 175 180 191 192 194 222 223 224 229 233 242 256 258 275 276 279 280 283 303 338 351 376 383 387 397 418 422 428 440 453 454 455 463 467 479 497 534 546 550 552 553 554 574 575 579 597 629 631 632 668 709 735 743 800 818 871 873 899 907 929 934 939 947 958 959 962 966 970 971 986 988 989 994
do
    time ./run.sh $options $1 $frame $2
done
