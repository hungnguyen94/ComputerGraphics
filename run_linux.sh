#!/bin/sh

#  run.sh
#
g++ -lpthread -lGL -lGLU -lglut -I . *.cpp -pthread -std=gnu++11
./a.out
