#
#	make file
#

.SUFFIXES : .c .cpp .o 

# 	GNU C++ compiler
CC = g++

TARGET      = $(notdir $(shell pwd))	## current foldername is target name

BUILD_PATH 	= ./build

BIN_PATH 	= $(BUILD_PATH)/bin
OBJ_PATH 	= $(BUILD_PATH)/obj

SRC_PATH 	= .

SRCS		= $(wildcard $(SRC_PATH)/*.cpp)
OBJS 		= $(SRCS:$(SRC_PATH)/%.cpp=$(OBJ_PATH)/%.o)

INCS	+= -I./include
INCS    += -I./lib/include
INCS	+= -I./usr/include

# 	Link Options
# pthread
LIBS	+= -lpthread
# udev
LIBS	+= -ludev
# video4linux2
LIBS	+= -lv4l2
	
# OpenCV default
OPENCV_CFLAGS  = $(shell pkg-config --cflags opencv)
OPENCV_LIBS	   = $(shell pkg-config --libs opencv)

#	set LDFLAGS
LDFLAGS     = $(LIBS) $(OPENCV_LIBS)

# 	set CFLAGS
#CXXFLAGS = $(INCS) $(OPENCV_CFLAGS) -c -std=c++11 -O3 -W -Wfatal-errors # -Wall
CXXFLAGS = $(INCS) $(OPENCV_CFLAGS) -c -std=c++11 -g  -fPIC -W -Wfatal-errors #-Wall

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
