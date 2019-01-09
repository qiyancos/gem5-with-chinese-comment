#!/bin/sh

route add default gw 11.3.0.1

(sleep 10; killall ProtocolServer; rm /bin/ProtocolServer; rm -rf /opt/ixps) &
echo killing ps in ten seconds, hopefully DoD will have finished by then

rm -rf /jail
mount -t nfs -o soft,nolock,intr,nfsvers=2 192.168.123.4:/jail /jail
