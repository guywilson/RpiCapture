###############################################################################
#                                                                             #
# MAKEFILE for Rpi capture tool                                               #
#                                                                             #
# (c) Guy Wilson 2020                                                         #
#                                                                             #
###############################################################################

# Version number for RpiCapture
MAJOR_VERSION = 1
MINOR_VERSION = 0

# Directories
SOURCE = src
BUILD = build
DEP = dep

# What is our target
TARGET = capture

# Tools
VBUILD = vbuild
CPP = g++
C = gcc
LINKER = g++

# postcompile step
PRECOMPILE = @ mkdir -p $(BUILD) $(DEP)
# postcompile step
POSTCOMPILE = @ mv -f $(DEP)/$*.Td $(DEP)/$*.d

CPPFLAGS = -c -O1 -Wall -pedantic -I/opt/vc/include -std=c++11
CFLAGS = -c -O1 -Wall -pedantic -I/opt/vc/include
DEPFLAGS = -MT $@ -MMD -MP -MF $(DEP)/$*.Td

# Libraries
STDLIBS = -pthread -lstdc++
EXTLIBS = -lmmal -lmmal_core -lmmal_util -lvcos -lbcm_host -lmmal_vc_client -lm -ldl

COMPILE.cpp = $(CPP) $(CPPFLAGS) $(DEPFLAGS) -o $@
COMPILE.c = $(C) $(CFLAGS) $(DEPFLAGS) -o $@
LINK.o = $(LINKER) -L/opt/vc/lib $(STDLIBS) -o $@

CSRCFILES = $(wildcard $(SOURCE)/*.c)
CPPSRCFILES = $(wildcard $(SOURCE)/*.cpp)
OBJFILES = $(patsubst $(SOURCE)/%.c, $(BUILD)/%.o, $(CSRCFILES)) $(patsubst $(SOURCE)/%.cpp, $(BUILD)/%.o, $(CPPSRCFILES))
DEPFILES = $(patsubst $(SOURCE)/%.c, $(DEP)/%.d, $(CSRCFILES)) $(patsubst $(SOURCE)/%.cpp, $(DEP)/%.d, $(CPPSRCFILES))

all: $(TARGET)

# Compile C/C++ source files
#
$(TARGET): $(OBJFILES)
	$(LINK.o) $^ $(EXTLIBS)

$(BUILD)/%.o: $(SOURCE)/%.c
$(BUILD)/%.o: $(SOURCE)/%.c $(DEP)/%.d
	$(PRECOMPILE)
	$(COMPILE.c) $<
	$(POSTCOMPILE)

$(BUILD)/%.o: $(SOURCE)/%.cpp
$(BUILD)/%.o: $(SOURCE)/%.cpp $(DEP)/%.d
	$(PRECOMPILE)
	$(COMPILE.cpp) $<
	$(POSTCOMPILE)

.PRECIOUS = $(DEP)/%.d
$(DEP)/%.d: ;

-include $(DEPFILES)

version:
	$(VBUILD) -incfile capture.ver -template version.c.template -out $(SOURCE)/version.c -major $(MAJOR_VERSION) -minor $(MINOR_VERSION)

clean:
	rm -r $(BUILD)
	rm -r $(DEP)
	rm $(TARGET)
