#!/bin/bash

for i in {1..200}
do
	v4l2-ctl --device /dev/video0 --set-fmt-video=width=410,height=308,pixelformat=MJPG --stream-mmap --stream-to=$i.png --stream-count=1
	read -p "Saved: $i, Hit enter to take next picture"
done
