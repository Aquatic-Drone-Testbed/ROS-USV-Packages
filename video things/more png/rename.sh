a=1
for i in radar_image*.jpg; do
  new=$(printf "radar_image%03d.jpg" "$a") # Zero-pads numbers to three digits
  mv -- "$i" "$new"
  let a=a+1
done
