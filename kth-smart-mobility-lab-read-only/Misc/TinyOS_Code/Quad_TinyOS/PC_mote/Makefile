COMPONENT=ServControl_BaseC

CFLAGS += -I$(shell pwd)/..

PFLAGS += -DPRINTF_BUFFER_SIZE=1000 -DPIN_DEBUG -DTKN154_BEACON_DISABLED
CFLAGS += -I$(TOSDIR)/lib/printf 

#include ../Makefile.include
# To use the TKN15.4 MAC instead of a platform's default MAC protocol first
# include the TinyOS "Makerules" file as usual ...
include $(MAKERULES)
# ... and then include the TKN15.4 "Makefile.include" file. That's all.
# Hint: type "make <platform> verbose" to see the aggregate include path.
include $(TOSDIR)/lib/mac/tkn154/Makefile.include

