CONTIKI = ../../../../../..

include Makefile.target

CFLAGS += -std=gnu99 -DPROJECT_CONF_H=\"project-conf.h\"

CONTIKI_WITH_IPV6 = 1

MODULES += core/net/mac/tsch
MODULES += examples/zolertia/zoul/dw1000/testbed


include $(CONTIKI)/Makefile.include
