# enable pretty build (comment to see full commands)
Q ?= @

# Set DEBUG to 1 for debugging
DEBUG := 0

# if only want to visualize saving results and don't do tracking,
# you could set VIS_ONLY to 1 and compile without ceres solver,
# but eigen3 is still needed.
# VIS_ONLY := 1

ifeq ($(DEBUG), 1)
	BUILD_DIR := build_debug
else
	BUILD_DIR := build
endif

VIS_ONLY ?= 0
ifeq ($(VIS_ONLY), 1)
	BUILD_DIR := $(BUILD_DIR)/PangaeaVis
else
	BUILD_DIR := $(BUILD_DIR)/PangaeaTracking
endif

LIB_BUILD_DIR := $(BUILD_DIR)/lib
BIN_BUILD_DIR := $(BUILD_DIR)/bin

# All the directories containing code
# SRC_DIRS := src/main_engine
MAIN_DIRS := $(shell find src/main_engine -type d -exec bash -c "find {} -maxdepth 1 \
	\( -name '*.cpp' \) | grep -q ." \; -print)
GUI_DIRS := $(shell find src/gui_app -type d -exec bash -c "find {} -maxdepth 1 \
	\( -name '*.cpp' \) | grep -q ." \; -print)
SRC_DIRS := $(MAIN_DIRS) $(GUI_DIRS)

ALL_BUILD_DIRS := $(addprefix $(BUILD_DIR)/, $(SRC_DIRS))
ALL_BUILD_DIRS += $(LIB_BUILD_DIR)

EIGEN_INCLUDE := -I/usr/include/eigen3/unsupported -I/usr/include/eigen3

WX_INCLUDE := `wx-config --cxxflags`

FLAGS_INCLUDE := $(EIGEN_INCLUDE) $(WX_INCLUDE) -I./include -I./include/third_party

# Library dependencies
GL_LIB := -lGL -lGLU -lX11 -lGLEW

WX_LIB := `wx-config --libs --gl-libs`

BOOST_LIB := -lboost_filesystem -lboost_system -lboost_thread

OPENCV_LIB := -lopencv_core -lopencv_highgui -lopencv_imgproc

CERES_LIB := -lceres -lglog -ltbb -ltbbmalloc -lcholmod -lccolamd \
	-lcamd -lcolamd -lamd -lsuitesparseconfig -llapack -lf77blas -latlas

LIBRARY_DIRS += $(LIB_BUILD_DIR)
LDFLAGS := $(WX_LIB) $(BOOST_LIB) $(OPENCV_LIB) $(CERES_LIB) $(GL_LIB)
LDFLAGS += $(foreach library_dir, $(LIBRARY_DIRS), -L$(library_dir))

# Setting compiler and building flags
CXX := g++
#CXXFLAGS += -std=c++11 -fopenmp -fPIC $(FLAGS_INCLUDE)
CXXFLAGS += -std=c++11 -fopenmp $(FLAGS_INCLUDE)

# Debugging
ifeq ($(DEBUG), 1)
	CXXFLAGS += -DDEBUG -g -O0
else
	CXXFLAGS += -DNDEBUG -O2 -ffast-math -Wno-unused-result
endif

# Automatic dependency generation
CXXFLAGS += -MMD -MP

# Get all source files
MAIN_ENGINE_SRCS := $(shell find src/main_engine -name "*.cpp" ! -name "Deform*.cpp")
#MAIN_ENGINE_SRCS := $(shell find src/main_engine -name "*.cpp" )
GUI_SRCS := $(shell find src/gui_app -name "*.cpp" ! -name "PangaeaTracking.cpp")
GUI_APP_SRCS := $(shell find src/gui_app -name "PangaeaTracking.cpp")
CONSOLE_APP_SRCS := $(shell find src/console_app -name "PangaeaTracking_console.cpp")

CONSOLE_BIN := $(BIN_BUILD_DIR)/PangaeaTracking_console

ifeq ($(VIS_ONLY), 0)
	MAIN_ENGINE_SRCS += src/main_engine/tracker/DeformNRSFMTracker.cpp
	MAIN_LIB := $(LIB_BUILD_DIR)/libmain_engine.a
	GUI_BIN := $(BIN_BUILD_DIR)/PangaeaTracking
	LDFLAGS := -lmain_engine $(LDFLAGS)
else
	CXXFLAGS += -DVIS_ONLY
	MAIN_LIB := $(LIB_BUILD_DIR)/libmain_engine_vis.a
	GUI_BIN := $(BIN_BUILD_DIR)/PangaeaTracking_vis
	LDFLAGS := -lmain_engine_vis $(LDFLAGS)
endif

# The objects corresponding to the source files
MAIN_OBJS := $(addprefix $(BUILD_DIR)/, ${MAIN_ENGINE_SRCS:.cpp=.o})
GUI_OBJS := $(addprefix $(BUILD_DIR)/, ${GUI_SRCS:.cpp=.o})

# Output files for automatic dependency generation
DEPS := $(MAIN_OBJS:.o=.d) $(GUI_OBJS:.o=.d)

ifeq ($(VIS_ONLY), 1)
	APP := $(GUI_BIN)
else
	APP := $(CONSOLE_BIN) $(GUI_BIN)
endif

#ORIGIN := \$$ORIGIN
#.PHONY:

.PHONY: all

all: $(APP)

$(CONSOLE_BIN): $(MAIN_LIB) | $(BIN_BUILD_DIR)
	@ echo CXX/LD -o $@
	$(Q) $(CXX) $(CONSOLE_APP_SRCS) -o $@ $(CXXFLAGS) $(LINKFLAGS) $(LDFLAGS)

$(GUI_BIN): $(GUI_OBJS) $(MAIN_LIB) | $(BIN_BUILD_DIR)
#	@ echo $(MAIN_ENGINE_SRCS)
#	@ echo $(MAIN_OBJS)
	@ echo CXX/LD -o $@
	$(Q) $(CXX) $(GUI_APP_SRCS) -o $@ $(GUI_OBJS) $(CXXFLAGS) $(LINKFLAGS) $(LDFLAGS)

# $(CXX) $(GUI_APP_SRCS) $(CXXFLAGS) -o $@ $(GUI_OBJS) $(LINKFLAGS) $(WX_PATH) $(LIBS_LINKER) -L./build/lib -Wl,-rpath,$(ORIGIN)/../lib -lmain_engine_vis
# $(CXX) $(GUI_APP_SRCS) $(CXXFLAGS) -o $@ $(GUI_OBJS) -lmain_engine_vis $(LINKFLAGS) $(LDFLAGS)
# $(Q)$(CXX) $< -o $@ $(LINKFLAGS) $(LDFLAGS) \
# -Wl,-rpath, -lmain_engine_vis

#@ echo AR -o $@
#$(Q)ar rcs $@ $(MAIN_OBJS)
#@ echo $(MAIN_OBJS)

$(MAIN_LIB): $(MAIN_OBJS) | $(LIB_BUILD_DIR)
	@ echo AR -o $@
	$(Q)ar rcs $@ $(MAIN_OBJS)
#	@ echo $(MAIN_OBJS)
#	@ echo LD -o $@
#	$(Q)$(CXX) -shared -o $@ $(MAIN_OBJS) $(LINKFLAGS) $(LDFLAGS)
#	$(Q)$(CXX) -o $@ $(MAIN_OBJS) $(LINKFLAGS) $(LDFLAGS)

# Define build targets
$(BUILD_DIR)/%.o: %.cpp | $(ALL_BUILD_DIRS)
	@ echo CXX $<
	$(Q)$(CXX) $< $(CXXFLAGS) -c -o $@

$(ALL_BUILD_DIRS):
	@ mkdir -p $@

$(BIN_BUILD_DIR):
	@ mkdir -p $@

-include $(DEPS)
