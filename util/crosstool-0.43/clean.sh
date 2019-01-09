#!/bin/sh
# Prepare a copy for distribution
set -x
rm -rf log log[0-9] *.log boards build dejagnu-1.4.3 dejagnu-1.4.3.tar.gz result tarballs jail.tar.gz *.sum2
find . -type f | xargs chmod 644
find . -type d | xargs chmod 755
find . -name '*.sh' | xargs chmod 755
chmod 755 config.guess

# And, what the heck, list all files containing the current version number
find . -name buildlogs -prune -o -type f -print | xargs grep -l '[^0-9]0\.43' 
