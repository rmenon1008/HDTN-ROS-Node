#!/bin/bash

for j in hdtn-one-process hdtn-ingress hdtn-egress-async hdtn-storage hdtn-scheduler bpsink-async  bpgen-async bpreceivefile bpsendfile hdtn-router web_interface
do
for i in `pidof $j`
do
 kill -SIGINT $i
done
done

sleep 6

for j in hdtn-one-process hdtn-ingress hdtn-egress-async hdtn-storage hdtn-scheduler bpsink-async bpgen-async bpreceivefile bpsendfile hdtn-router web_interface

do
for i in `pidof $j`
do
 kill -9 $i
done
done
