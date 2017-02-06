#!/bin/bash

killall -u root -- python
killall -u root -- mjpg_streamer
sleep 2
/usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_file.so  -f /root/programmes_python/jpg" -o "/usr/local/lib/output_http.so -w /usr/local/www" &
python /root/programmes_python/QuatreRoues_SuiviCouleur.py


