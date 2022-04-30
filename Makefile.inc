APPL_CXXOBJS += device_io.o etrc_info.o utils.o game_play.o driving_type.o end_condition.o driving_manager.o state_manager.o

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile
endif


