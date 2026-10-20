#!/bin/bash

echo "crawler-headlight-shutdown script started"

GPIO=6

for i in 1 2 3 4 5
do
    /usr/bin/gpioset gpiochip0 ${GPIO}=1
    sleep 0.25
    /usr/bin/gpioset gpiochip0 ${GPIO}=0
    sleep 0.25
done

/usr/bin/gpioset gpiochip0 ${GPIO}=1

echo "crawler-headlight-shutdown script finished"
exit 0
