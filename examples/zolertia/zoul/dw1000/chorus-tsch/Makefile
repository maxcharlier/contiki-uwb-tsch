CONTIKI = ../../../../..

include Makefile.target

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

CONTIKI_WITH_IPV6 = 1

MODULES += core/net/mac/tsch
# MODULES += examples/zolertia/zoul/dw1000/chorus-tsch
PROJECT_SOURCEFILES += shedule-chorus.c

include $(CONTIKI)/Makefile.include
