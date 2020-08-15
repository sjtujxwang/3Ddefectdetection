# Makefile for Basler pylon C sample program
.PHONY: all clean

# The program to build
NAME       := OverlappedGrab

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5

# Build tools and flags
LD         := $(CC)
CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags pylonc)
CFLAGS     := #e.g., CFLAGS=-g -O0 for debugging
LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath pylonc)
LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs pylonc)

# Rules for building
all: $(NAME)

$(NAME): $(NAME).o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(NAME).o: $(NAME).c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

clean:
	$(RM) $(NAME).o $(NAME)
