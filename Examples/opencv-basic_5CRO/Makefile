#
#	make file
#

.SUFFIXES : .cpp .o 

# 	GNU C++ compiler
CC = g++

TARGET      = $(notdir $(shell pwd))	## current foldername is target name

BUILD_PATH 	= ./build

BIN_PATH 	= $(BUILD_PATH)/bin
OBJ_PATH 	= $(BUILD_PATH)/obj

SRC_PATH 	= .

SRCS		= $(wildcard $(SRC_PATH)/*.cpp)
OBJS 		= $(SRCS:$(SRC_PATH)/%.cpp=$(OBJ_PATH)/%.o)

INCS	+= -I.
INCS	+= -I/usr/local/include

CXXFLAGS = $(INCS) -c -O2 #-std=c++11 -W -Wall -O0 $(shell pkg-config --cflags opencv)
	
# 	Link Options
LIBS		+= -lpthread

# OpenCV default
LIBS	   	+= $(shell pkg-config --libs opencv)

LDFLAGS     = $(LIVDIRS) -lm $(LIBS)

#	rm options
RM 			= @rm -rfv

# 	mkdir options
MKDIR 		= @mkdir -p

$(BIN_PATH)/$(TARGET): $(OBJS)
	$(MKDIR) $(BIN_PATH)
	$(CC) -o $(BIN_PATH)/$(TARGET) $(OBJS) $(LDFLAGS)

$(OBJ_PATH)/%.o: $(SRC_PATH)/%.cpp
	$(MKDIR) $(OBJ_PATH)
	$(CC) $(CXXFLAGS) $< -o $@

all : $(BIN_PATH)/$(TARGET)

#	dependency
dep :
	$(MKDIR) $(BUILD_PATH)
	$(CC) -M $(INCS) $(SRCS) > $(BUILD_PATH)/.depend

#	clean
clean:
	$(RM) $(BUILD_PATH)
	$(RM) $(TARGET)
	@echo "Done."

#	include dependency
ifeq (.depend,$(wildcard .depend))
include .depend
endif
