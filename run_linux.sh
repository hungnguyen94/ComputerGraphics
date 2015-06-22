#!/bin/sh

#  run.sh
#
g++ -std=gnu++11 -lpthread -lGL -lGLU -lglut -I . *.cpp

./a.out
