#!/bin/sh

# Program to clean up all but the 20 most recent *.*.* files in /tmp
# Avoids running out of disk space when running gcc's "make test"
# If you are doing remote testing an embedded platform without much disk space,
# transfer this to the embedded system and run it in the background
# while running the test suite.
JAIL=/jail
while true; do
   ls -t $JAIL/tmp/*.*.* 2>/dev/null | tail +20 | xargs rm -f > /dev/null 2>&1
   sleep 60
done
