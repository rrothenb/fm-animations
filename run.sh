time go run series2.go -frame $1 -desiredtriangles $2
cat data/$1.header.ply data/$1.data.ply > mitsuba.ply
rm data/$1.data.ply
mv data/$1.rgbe mitsuba.rgbe
mv data/$1.roughness.rgbe mitsuba.roughness.rgbe
mv data/$1.blend.rgbe mitsuba.blend.rgbe
time mitsuba test.xml
convert test.exr $1.jpg
