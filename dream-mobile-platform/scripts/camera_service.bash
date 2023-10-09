#!/bin/bash

while true; do
    if v412-ctl --list-devices | grep -q "/dev/video0";
        ffmpeg -f v4l2 -i /dev/video0 -vf "scale=640:360,blackdetect=d=0.1:pix_th=.10" -preset ultrafast -video_size 640x360 -r 5 -c:v mjpeg -q:v 2 -f mjpeg udp://100.66.47.31:12345?pkt_size=1316
    else
        sleep 1
    fi
done
