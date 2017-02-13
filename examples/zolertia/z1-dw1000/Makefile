CONTIKI = ../../..

ifndef TARGET
TARGET=z1-dw1000
endif
LANG=en

 
+CFLAGS += -O3 -Wall -Werror

PORT=/dev/ttyUSB
PORT_Z1_6=/dev/z1_6
PORT_Z1_7=/dev/z1_7
PORT_Z1_8=/dev/z1_8
PORT_Z1_9=/dev/z1_9
PORT_Z1_10=/dev/z1_10

all: test-dw1000

CONTIKI_WITH_RIME = 1
#TARGET_LIBFILES += -lm #add math lib

include $(CONTIKI)/Makefile.include

# exit with crtl+a +x
z1-less0:
	make z1-reset MOTES=$(PORT)0 && picocom -b 115200 $(PORT)0
z1-less1:
	make z1-reset MOTES=$(PORT)1 && picocom -b 115200 $(PORT)1
z1-less2:
	make z1-reset MOTES=$(PORT)2 && picocom -b 115200 $(PORT)2
z1-less3:
	make z1-reset MOTES=$(PORT)3 && picocom -b 115200 $(PORT)3
z1-less6:
	make z1-reset MOTES=$(PORT)3 && picocom -b 115200 $(PORT)3

z1-login0:
	make z1-reset MOTES=$(PORT)0 && make login MOTES=$(PORT)0 
z1-login1:
	make z1-reset MOTES=$(PORT)1 && make login MOTES=$(PORT)1 
z1-login2:
	make z1-reset MOTES=$(PORT)2 && make login MOTES=$(PORT)2 
z1-login3:
	make z1-reset MOTES=$(PORT)3 && make login MOTES=$(PORT)3 

# exit with crtl+a +k +y
z1-screen0:
	make z1-reset MOTES=$(PORT)0 && screen $(PORT)0 115200
z1-screen1:
	make z1-reset MOTES=$(PORT)1 && screen $(PORT)1 115200
z1-screen2:
	make z1-reset MOTES=$(PORT)2 && screen $(PORT)2 115200
z1-screen3:
	make z1-reset MOTES=$(PORT)3 && screen $(PORT)3 115200

reset:
	make z1-reset && make login

login-6:
	make z1-reset MOTES=$(PORT_Z1_6) && make login MOTES=$(PORT_Z1_6)
login-7:
	make z1-reset MOTES=$(PORT_Z1_7) && make login MOTES=$(PORT_Z1_7)
login-8:
	make z1-reset MOTES=$(PORT_Z1_8) && make login MOTES=$(PORT_Z1_8)
login-9:
	make z1-reset MOTES=$(PORT_Z1_9) && make login MOTES=$(PORT_Z1_9)
login-10:
	make z1-reset MOTES=$(PORT_Z1_10) && make login MOTES=$(PORT_Z1_10)

screen-6:
	make z1-reset MOTES=$(PORT_Z1_6) && screen $(PORT_Z1_6) 115200
screen-10:
	make z1-reset MOTES=$(PORT_Z1_10) && screen $(PORT_Z1_10) 115200


sender:
	make frame-sender.upload MOTES=$(PORT_Z1_6)
anchor:
	make anchor.upload MOTES=$(PORT_Z1_6)
calibrator:
	make calibrator2.upload MOTES=$(PORT_Z1_6)
sender-debug:
	make frame-sender-debug.upload MOTES=$(PORT_Z1_6)
master10:
	make master.upload MOTES=$(PORT_Z1_10)

receiver:
	make frame-receiver.upload MOTES=$(PORT_Z1_10)