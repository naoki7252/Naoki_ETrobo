<<<<<<< HEAD
mkfile_path := $(dir $(lastword $(MAKEFILE_LIST)))

APPL_CXXOBJS += device_io.o etrc_info.o driving_type.o utils.o end_condition.o driving_manager.o game_play.o state_manager.o
=======
APPL_CXXOBJS += device_io.o etrc_info.o utils.o game_play.o driving_type.o end_condition.o driving_manager.o state_manager.o
>>>>>>> 301acb76addfcd4224db2bd3d36a69b702655897

SRCLANG := c++

ifdef CONFIG_EV3RT_APPLICATION
include $(EV3RT_SDK_LIB_DIR)/libcpp-ev3/Makefile
endif
<<<<<<< HEAD
=======


>>>>>>> 301acb76addfcd4224db2bd3d36a69b702655897
