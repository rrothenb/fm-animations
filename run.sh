fixedargs=`echo $* | sed "s/--number-rows/-r/" `
fixedargs=`echo $fixedargs | sed "s/--first-row/-f/"`
fixedargs=`echo $fixedargs | sed "s/--aspect-ratio/-a/"`
fixedargs=`echo $fixedargs | sed "s/--height/-h/"`
fixedargs=`echo $fixedargs | sed "s/--samples/-s/"`
numrows=1
firstrow=0
height=0
options=""
while getopts 'r:f:a:h:s:' opt $fixedargs; do
  case "$opt" in
    r)
      numrows=$OPTARG
      options="$options -numrows $OPTARG"
      ;;

    f)
      firstrow=$OPTARG
      ;;

    a)
      options="$options -aspectratio $OPTARG"
      ;;

    h)
      height=$OPTARG
      options="$options -height $height"
      ;;

    s)
      options="$options -samples $OPTARG"
      ;;

    ?)
      echo "Usage: $(basename $0) [-r Number of rows] [-f First row] series frame triangles"
      exit 1
      ;;
  esac
done
shift "$(($OPTIND -1))"
time go run series$1.go $options -frame $2 -desiredtriangles $3
cat data/$2.header.ply data/$2.data.ply > mitsuba.ply
rm data/$2.data.ply
#mv data/$2.roughness.rgbe mitsuba.roughness.rgbe
#convert mitsuba.roughness.rgbe mitsuba.roughness.jpg
mv data/$2.blend.rgbe mitsuba.blend.rgbe
convert mitsuba.blend.rgbe mitsuba.blend.jpg
mv data/$2.land.blend.rgbe mitsuba.land.blend.rgbe
convert mitsuba.land.blend.rgbe mitsuba.land.blend.jpg
mv data/$2.metal.blend.rgbe mitsuba.metal.blend.rgbe
convert mitsuba.metal.blend.rgbe mitsuba.metal.blend.jpg
#cp mitsuba.blend.jpg $2.blend.jpg
mv data/$2.rgbe mitsuba.rgbe
#convert mitsuba.rgbe -rotate 180 mitsuba.rgbe
convert mitsuba.rgbe mitsuba.env.jpg
#mv data/$2.clay.1.color.rgbe clay.1.color.rgbe
#convert clay.1.color.rgbe clay.1.color.jpg
#mv data/$2.secondary.mask.rgbe mitsuba.secondary.mask.rgbe
#convert mitsuba.secondary.mask.rgbe mitsuba.secondary.mask.jpg
mv data/$2.texture.rgbe mitsuba.texture.rgbe
convert mitsuba.texture.rgbe mitsuba.texture.jpg
#exit
if [ $numrows -eq "1" ]
then
  time mitsuba -Doffset=0 -m scalar_rgb test.xml
  convert test.exr -auto-gamma -modulate 100,150,100 -sigmoidal-contrast 5x0% $2.jpg
else
  for row in `seq $firstrow $(($numrows-1))`
  do
    time mitsuba -Doffset=$(($row*$height/$numrows)) -m scalar_rgb test.xml
    convert test.exr -auto-gamma -modulate 100,150,100 -sigmoidal-contrast 5x0% $2.$row.jpg
    convert $2.{?,??}.jpg -append $2.jpg
    mv test.exr $2.$row.exr
  done
fi
