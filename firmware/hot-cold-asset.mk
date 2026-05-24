ifneq (,$(findstring template,$(MAKECMDGOALS)))
ASSET_FILES=$(wildcard static/*)
else
ASSET_FILES=$(wildcard static/*) $(wildcard static.lib/*)
endif

TEMPLATE_USER_FILES+=$(wildcard static/*)

ASSET_OBJ=$(addprefix $(BINDIR)/, $(addsuffix .o, $(ASSET_FILES)) )

GETALLOBJ=$(sort $(call ASMOBJ,$1) $(call COBJ,$1) $(call CXXOBJ,$1)) $(ASSET_OBJ)

.SECONDEXPANSION:
$(ASSET_OBJ): $$(patsubst bin/%,%,$$(basename $$@))
	$(VV)mkdir -p $(BINDIR)/static
	$(VV)mkdir -p $(BINDIR)/static.lib
	@echo "ASSET $@"
	$(VV)$(OBJCOPY) -I binary -O elf32-littlearm -B arm $^ $@
