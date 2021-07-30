for offset in `seq 401 100 31401`
do
  echo "rendering $offset..."
  time mitsuba -Doffset=$offset test.xml
  convert test.exr $offset.jpg
done
