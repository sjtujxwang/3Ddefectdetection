# Makefile for Basler pylon sample programs
.PHONY            : all clean

# The programs to build
SUBDIRS           := LegacyAcquireSingleFrame \
                     LegacyAcquireSingleFrame_ChunkImage \
                     LegacyAcquireSingleFrame_Gen \
                     LegacyCameraEvents \
                     LegacyRegisterRemovalCallback

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
