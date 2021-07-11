for frame in `seq 1059 5000`
do
  time go run series1.6.go -frame $frame -desiredtriangles 15000000
  cat data/$frame.header.ply data/$frame.data.ply > mitsuba.ply
  rm data/$frame.data.ply
  mv data/$frame.rgbe mitsuba.rgbe
  time mitsuba test.xml
  convert test.exr $frame.jpg
done
