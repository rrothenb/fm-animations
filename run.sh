time go run series$1.go -frame $2 -desiredtriangles $3
cat data/$2.header.ply data/$2.data.ply > mitsuba.ply
rm data/$2.data.ply
#mv data/$2.blend.rgbe mitsuba.blend.rgbe
#convert mitsuba.blend.rgbe mitsuba.blend.jpg
mv data/$2.rgbe mitsuba.rgbe
#convert mitsuba.rgbe -rotate 180 mitsuba.rgbe
convert mitsuba.rgbe mitsuba.env.jpg
mv data/$2.blend.rgbe mitsuba.blend.rgbe
convert mitsuba.blend.rgbe mitsuba.blend.jpg
mv data/$2.land.blend.rgbe mitsuba.land.blend.rgbe
convert mitsuba.land.blend.rgbe mitsuba.land.blend.jpg
#mv data/$2.texture.rgbe mitsuba.texture.rgbe
#convert mitsuba.texture.rgbe mitsuba.texture.jpg
#exit
time mitsuba test.xml
convert test.exr -auto-gamma -brightness-contrast 20x30 -modulate 100,125,100 $2.jpg
