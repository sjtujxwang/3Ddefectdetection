# Makefile for Bcon Adapter sample library
.PHONY: all clean

# The shared library to build
NAME     := libBconAdapterMySample.so

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5

CC       ?= gcc
CPPFLAGS := -I. $(shell $(PYLON_ROOT)/bin/pylon-config --cflags-only-I)
CFLAGS   := -fPIC -fvisibility=hidden -Wall
LDFLAGS  := -shared
DEPS     := MyBconAdapterLogging.h
OBJS     := MyBconAdapterLibrary.o MyBconAdapterI2CConnection.o MyBconAdapterLogging.o MyBconAdapterEnumerator.o

# Rules for building
%.o: %.c $(DEPS)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c -o $@ $<

all: $(NAME)

$(NAME): $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^

clean:
	$(RM) $(OBJS) $(NAME)
