#!/bin/bash

FRAME_RATE=10

while true; do
	if v4l2-ctl --list-devices | grep -q "/dev/video0" 
	then
+        ffmpeg -f v4l2 -i /dev/video0 -vf "scale=640:360,blackdetect=d=0.1:pix_th=.10" -preset ultrafast -video_size 640x360 -r ${FRAME_RATE} -c:v mjpeg -q:v 2 -f mjpeg udp://3.145.184.145:12345?pkt_size=1316
+        # ffmpeg -f v4l2 -i /dev/video0 -vf "scale=640:360,blackdetect=d=0.1:pix_th=.10" -preset ultrafast -video_size 640x360 -r ${FRAME_RATE} -c:v mjpeg -q:v 2 -f mjpeg udp://100.66.47.31:12345?pkt_size=1316

	echo "video0 is connected"
    else
        sleep 2
    fi
done
