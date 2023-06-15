fixedargs=`echo $* | sed "s/--number-rows/-r/" | sed "s/--first-row/-f/"`
echo $fixedargs
numrows=0
firstrow=0
while getopts 'r:f:h' opt $fixedargs; do
  case "$opt" in
    r)
      numrows=$OPTARG
      ;;

    f)
      firstrow=$OPTARG
      ;;

    ?|h)
      echo "Usage: $(basename $0) [-r Number of rows] [-f First row] series frame triangles"
      exit 1
      ;;
  esac
done
shift "$(($OPTIND -1))"
echo $*

time go run series$1.go -frame $2 -desiredtriangles $3
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
time mitsuba -m scalar_rgb test.xml
convert test.exr -auto-gamma -modulate 100,150,100 -sigmoidal-contrast 5x0% $2.jpg
