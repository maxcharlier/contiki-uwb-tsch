#!/bin/sh

# firmware lab
sudo udevadm trigger && cd ~/contiki-uwb-tsch/ && git pull origin thomas-mobility && cd ~/contiki-uwb-tsch/examples/zolertia/zoul/dw1000/localisation-mobility && echo ----- Node 0x01 && make localisation-server.upload TARGET=zoul BOARD=firefly-dw1000 NODEID=0x01 PORT=/dev/anchor1 && echo ----- Node 0x02 && make localisation-client.upload TARGET=zoul BOARD=firefly-dw1000 NODEID=0x02 RPL_LEAF_ONLY=1 PORT=/dev/anchor2