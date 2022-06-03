for eta in $(seq 1 142); do for k in $(seq 1 142); do
  cp spd/$eta.spd test.eta.spd
  cp spd/$k.spd test.k.spd
  echo "### processing  $eta-$k"
  time mitsuba -m scalar_rgb spds.xml > /dev/null
  sleep 1
  convert spds.exr -auto-gamma spds.jpg
  sleep 1
  rm spds.exr
  for sample in spd-samples-2/*
  do
    difference=`compare spds.jpg $sample -metric PHASH null: 2>&1`
    echo "$difference - $sample"
    match=`echo $difference | sed "s/^[0-9]\..*$/match/" | sed "s/^0$/match/" | sed "s/^[1-3][0-9]\..*$/match/"`
    if [ "$match" == "match" ]
    then
      echo "### Found a match for $eta-$k ($sample)"
      mv spds.jpg spd-samples-rejects/$eta-$k.jpg
      break
    fi
  done
  if [ "$match" != "match" ]
  then
    mv spds.jpg spd-samples-2/$eta-$k.jpg
  fi
done; done
