sed "s/OFFSET/$1/" 4.sensor.xml > sensor.xml
time mitsuba -m scalar_rgb test.xml
cp test.exr 4.$1.exr
convert test.exr -auto-gamma -normalize -modulate 100,150,100 -brightness-contrast 30x40 4.$1.jpg
