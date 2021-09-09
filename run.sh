time go run series7.go -frame $1 -desiredtriangles $2
cat data/$1.header.ply data/$1.data.ply > mitsuba.ply
rm data/$1.data.ply
mv data/$1.rgbe mitsuba.rgbe
#convert mitsuba.rgbe $1.env.jpg
mv data/$1.roughness.rgbe mitsuba.roughness.rgbe
#convert mitsuba.roughness.rgbe $1.roughness.jpg
mv data/$1.blend.rgbe mitsuba.blend.rgbe
#convert mitsuba.blend.rgbe $1.blend.jpg
time mitsuba test.xml
convert test.exr $1.jpg
