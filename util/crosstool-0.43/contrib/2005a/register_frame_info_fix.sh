#!/bin/sh
# give a gcc as the 1st argument, flags as 2nd arg
$1 $2 -shared -o register_frame_info_fix.so register_frame_info_fix.c
