all: simple-client simple-server
CONTIKI = ../../../../..

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

MODULES += core/net/mac/tsch
MODULES += examples/zolertia/zoul/dw1000/localisation-mobility/libs

ifdef WITH_COMPOWER
APPS+=powertrace
CFLAGS+= -DCONTIKIMAC_CONF_COMPOWER=1 -DWITH_COMPOWER=1 -DQUEUEBUF_CONF_NUM=4
endif

ifdef SERVER_REPLY
CFLAGS+=-DSERVER_REPLY=$(SERVER_REPLY)
endif
ifdef PERIOD
CFLAGS+=-DPERIOD=$(PERIOD)
endif

ifdef IS_MOBILE
CFLAGS+=-DIS_MOBILE=$(IS_MOBILE)
endif

ifdef IS_LOCATION_SERVER
CFLAGS+=-DIS_LOCATION_SERVER=$(IS_LOCATION_SERVER)
endif

ifeq ($(MAKE_WITH_NON_STORING),1)
CFLAGS += -DWITH_NON_STORING=1
endif

CONTIKI_WITH_IPV6 = 1
include $(CONTIKI)/Makefile.include
