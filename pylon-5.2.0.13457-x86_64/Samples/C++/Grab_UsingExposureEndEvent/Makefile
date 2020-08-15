# Makefile for Basler pylon sample program
.PHONY: all clean

# The program to build
NAME       := Grab_UsingExposureEndEvent

# Installation directories for pylon
PYLON_ROOT ?= /opt/pylon5

# Build tools and flags
LD         := $(CXX)
CPPFLAGS   := $(shell $(PYLON_ROOT)/bin/pylon-config --cflags)
CXXFLAGS   := #e.g., CXXFLAGS=-g -O0 for debugging
LDFLAGS    := $(shell $(PYLON_ROOT)/bin/pylon-config --libs-rpath)
LDLIBS     := $(shell $(PYLON_ROOT)/bin/pylon-config --libs)

# Rules for building
all: $(NAME)_GigE $(NAME)_Usb $(NAME)_Universal

%: %.o
	$(LD) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(NAME)_GigE.o: $(NAME).cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DUSE_GIGE -c -o $@ $<

$(NAME)_Usb.o: $(NAME)_Usb.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -DUSE_USB -c -o $@ $<

$(NAME)_Universal.o: $(NAME)_Universal.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

clean:
	$(RM) $(NAME)_GigE.o $(NAME)_GigE
	$(RM) $(NAME)_Usb.o $(NAME)_Usb
	$(RM) $(NAME)_Universal.o $(NAME)_Universal
    
