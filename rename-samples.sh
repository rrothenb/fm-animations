for file in *.jpg; do
  hue=`convert $file -colors 1 -resize 1x1 -colorspace HSV txt:- | tail -1 | cut -f2 -d' ' | cut -f1 -d, | cut -c2- | cut -f1 -d. | sed "s/^\([1-9][0-9]\)$/0\1/" | sed "s/^\([0-9]\)$/00\1/"`
  cp $file ../spd-samples-3/$hue-$file
done
