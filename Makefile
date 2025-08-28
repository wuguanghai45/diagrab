SOURCES =  main.c diag_qcom.c sahara.c log.c
CROSS-COMPILE =
CC := $(CROSS-COMPILE)gcc

# Stolen from Linux build system
comma = ,
try-run = $(shell set -e; ($(1)) >/dev/null 2>&1 && echo "$(2)" || echo "$(3)")
cc-option = $(call try-run, $(CC) $(1) -c -xc /dev/null -o /dev/null,$(1),$(2))

CFLAGS ?= -s
WFLAGS := -Wall \
	$(call cc-option,-Wextra) \
	$(call cc-option,-Wwrite-strings) \
	$(call cc-option,-Wno-sign-compare) \
	$(call cc-option,-Wno-unused-function)
CFLAGS += $(WFLAGS)

simcom-log_SOURCES = main.c diag_qcom.c sahara.c log.c tftp.c

all: clean
	$(CC) $(CFLAGS) ${simcom-log_SOURCES} -o diaggrab -lpthread -ldl

clean:
	@-rm -rf diaggrab