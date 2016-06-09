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

CXXFLAGS = $(INCS) -c -O2 #-std=c++11 -W -Wall -O0
	
# 	Link Options
LIBS		+= -lpthread

LIVDIRS     += -L$(OPENCV_LIB_PATH)
LIBS 		+= -lopencv_calib3d
LIBS 		+= -lopencv_contrib
LIBS 		+= -lopencv_core
LIBS 		+= -lopencv_features2d
LIBS 		+= -lopencv_flann
LIBS 		+= -lopencv_gpu
LIBS 		+= -lopencv_highgui
LIBS 		+= -lopencv_imgproc
LIBS 		+= -lopencv_legacy
LIBS 		+= -lopencv_ml
LIBS 		+= -lopencv_nonfree
LIBS 		+= -lopencv_objdetect
LIBS 		+= -lopencv_ocl
LIBS 		+= -lopencv_photo
LIBS 		+= -lopencv_stitching
LIBS 		+= -lopencv_superres
LIBS 		+= -lopencv_ts
LIBS 		+= -lopencv_video
LIBS 		+= -lopencv_videostab

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
