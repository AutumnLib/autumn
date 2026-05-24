################################################################################
######################### User configurable parameters #########################
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=-I$(ROOT)

USE_PACKAGE:=1
EXCLUDE_COLD_LIBRARIES:=
ifeq ($(IS_LIBRARY),1)
EXCLUDE_COLD_LIBRARIES+=$(FWDIR)/lib$(LIBNAME).a
endif

ifdef CONSUMER_TEMPLATE
IS_LIBRARY:=0
else
IS_LIBRARY:=1
endif
LIBNAME:=autumn
VERSION:=0.1.0

ifeq ($(IS_LIBRARY),0)
EXCLUDE_SRCDIRS+=src/autumn
endif

EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main $(SRCDIR)/team/assets,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

TEMPLATE_SYSTEM_FILES=$(shell find $(INCDIR)/autumn -name '*.hpp') $(FWDIR)/hot-cold-asset.mk
TEMPLATE_USER_FILES=$(ROOT)/static/example.txt $(ROOT)/team/config.hpp $(ROOT)/team/autonomous.hpp $(ROOT)/team/driver.hpp $(SRCDIR)/main.cpp $(SRCDIR)/team/assets.cpp

.DEFAULT_GOAL=quick

.PHONY: libautumn
libautumn: library
	$(VV)cp $(BINDIR)/$(LIBNAME).a $(FWDIR)/lib$(LIBNAME).a

.PHONY: template-release
template-release: libautumn
	$(MAKE) template CONSUMER_TEMPLATE=1

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
