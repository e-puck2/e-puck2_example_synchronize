
#This is a template to build your own project with the e-puck2_main-processor folder as a library.
#Simply adapt the lines below to be able to compile

# Define project name here
PROJECT = e-puck2_example_synchronize

#Define path to the e-puck2_main-processor folder
GLOBAL_PATH = ../e-puck2_main-processor

#Source files to include
CSRC += ./main.c \
		./synchronize.c \

#Header folders to include
INCDIR += 

#Jump to the main Makefile
include $(GLOBAL_PATH)/Makefile
