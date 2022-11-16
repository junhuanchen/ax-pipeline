#!/bin/sh

export LD_LIBRARY_PATH=$(dirname $0)/lib:$LD_LIBRARY_PATH

route add -net 239.255.255.250 netmask 255.255.255.255 usb0

cd $(dirname $0)

# for gc4653 input axsample rgb model not mp4 & onvif
# ./sample_vin_ivps_joint_venc_rtsp_vo -m /home/models/yolov5s.joint -p ./config/yolov5s.json -v 0 -c 2 

# for gc4653
./sample_vin_ivps_joint_venc_rtsp_vo_onvif_mp4v2 -m ./model/yolov5s_sub_nv12_11.joint -p ./config/yolov5s.json -v 0 -c 2 

# for os04a
# ./sample_vin_ivps_joint_venc_rtsp_vo_onvif_mp4v2 -m ./model/yolov5s_sub_nv12_11.joint -p ./config/yolov5s.json -v 0 -c 0 -e 2 

#!/bin/sh

cd -

