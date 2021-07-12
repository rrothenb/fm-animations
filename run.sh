time go run series1.6.go -frame 0 -desiredtriangles 25000000
cat data/0.header.ply data/0.data.ply > mitsuba.ply
mv data/0.rgbe mitsuba.rgbe
time mitsuba test.xml
