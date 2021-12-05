for row in `seq 0 9`
do
  time go run series$1.go -frame $2 -desiredtriangles $3 -row $row
  cat data/$2.header.ply data/$2.data.ply > mitsuba.ply
  rm data/$2.data.ply
  #mv data/$2.roughness.rgbe mitsuba.roughness.rgbe
  #convert mitsuba.roughness.rgbe mitsuba.roughness.jpg
  mv data/$2.blend.rgbe mitsuba.blend.rgbe
  convert mitsuba.blend.rgbe mitsuba.blend.jpg
  #cp mitsuba.blend.jpg $2.blend.jpg
  mv data/$2.rgbe mitsuba.rgbe
  #convert mitsuba.rgbe -rotate 180 mitsuba.rgbe
  convert mitsuba.rgbe mitsuba.env.jpg
  mv data/$2.metal.blend.rgbe mitsuba.metal.blend.rgbe
  convert mitsuba.metal.blend.rgbe mitsuba.metal.blend.jpg
  mv data/$2.texture.rgbe mitsuba.texture.rgbe
  convert mitsuba.texture.rgbe mitsuba.texture.jpg
  #exit
  time mitsuba test.xml
  convert test.exr -brightness-contrast 25x30 $2.$row.jpg
done