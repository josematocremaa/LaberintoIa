###
### Makefile for Grupo_N Webots controller (snap install on Linux)
###

# Webots snap installation path
WEBOTS_HOME ?= /snap/webots/current/usr/share/webots

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH?=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
