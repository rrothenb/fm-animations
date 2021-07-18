time go run series2.go -frame $1 -desiredtriangles 2500000
cat data/$1.header.ply data/$1.data.ply > mitsuba.ply
rm data/$1.data.ply
mv data/$1.rgbe mitsuba.rgbe
time mitsuba test.xml
convert test.exr $1.jpg
