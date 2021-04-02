
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = epuck-artist

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = lib/e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./modules/mod_communication.c \
		./modules/mod_data.c \
		

#Header folders to include
INCDIR += modules/include

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
