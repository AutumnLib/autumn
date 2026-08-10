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

EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main $(SRCDIR)/assets,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

TEMPLATE_SYSTEM_FILES=$(shell find $(INCDIR)/autumn -name '*.hpp') $(FWDIR)/hot-cold-asset.mk
TEMPLATE_USER_FILES=$(ROOT)/static/example.txt $(ROOT)/templates/config.hpp $(ROOT)/templates/autonomous.hpp $(ROOT)/templates/driver.hpp $(SRCDIR)/main.cpp $(SRCDIR)/assets.cpp

.DEFAULT_GOAL=quick

.PHONY: libautumn
libautumn: library
	$(VV)cp $(BINDIR)/$(LIBNAME).a $(FWDIR)/lib$(LIBNAME).a

.PHONY: template-release
template-release: libautumn
	$(MAKE) template

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
