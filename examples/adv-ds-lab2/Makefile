TARGET=cooja
CONTIKI_PROJECT = nullnet-unicast
CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

all: $(CONTIKI_PROJECT)

CONTIKI = ../..

#use this to enable TSCH: 
MAKE_MAC = MAKE_MAC_TSCH
#MAKE_MAC ?= MAKE_MAC_CSMA
MAKE_NET = MAKE_NET_NULLNET
include $(CONTIKI)/Makefile.include

compile-and-simulate: $(CONTIKI)/tools/cooja/dist/cooja.jar
	java -mx1024m -jar ../../tools/cooja/dist/cooja.jar -nogui='nullnet-unicast-cooja-script-123456.csc' -contiki='../..'

compile-and-simulate2: $(CONTIKI)/tools/cooja/dist/cooja.jar
	java -mx1024m -jar ../../tools/cooja/dist/cooja.jar -nogui='nullnet-unicast-cooja-script-123457.csc' -contiki='../..'

compile-and-simulate3: $(CONTIKI)/tools/cooja/dist/cooja.jar
	java -mx1024m -jar ../../tools/cooja/dist/cooja.jar -nogui='nullnet-unicast-cooja-script-123458.csc' -contiki='../..'

check-results:	
	cat COOJA.testlog |grep "Sink"
	cat COOJA.testlog |grep ":1:.*Battery"
	cat COOJA.testlog |grep "collected"
	

