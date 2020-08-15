# Makefile for Basler pylon C sample programs
.PHONY            : all clean

# The programs to build
SUBDIRS           := ActionCommands \
                     Chunks \
                     Events \
                     GenApiParam \
                     GrabTwoCameras \
                     OverlappedGrab \
                     ParametrizeCamera \
                     SimpleGrab \
                     SurpriseRemoval

PYLON_ROOT ?= /opt/pylon5

# Rules for building
all:
	for subDirectory in $(SUBDIRS); do \
		$(MAKE) -C $$subDirectory PYLON_ROOT="$(PYLON_ROOT)" all; \
		if test $$? -ne 0; then exit 1; fi; \
	done

clean:
	for subDirectory in $(SUBDIRS); do \
		$(MAKE) -C $$subDirectory PYLON_ROOT="$(PYLON_ROOT)" clean; \
	done

.PHONY: install
install:
