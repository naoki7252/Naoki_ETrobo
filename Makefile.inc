mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += device_io.o etrc_info.o driving_type.o utils.o end_condition.o driving_manager.o game_play.o state_manager.o

SRCLANG := c++

INCLUDES += -I$(ETROBO_HRP3_WORKSPACE)/etroboc_common

ifdef CONFIG_EV3RT_APPLICATION
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile
endif
