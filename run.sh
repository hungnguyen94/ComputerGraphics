#!/bin/sh

#  run.sh
#  
#
#  Created by Ruben Wiersma on 04-06-15.
#
g++ -lpthread -framework OpenGL -framework GLUT -I. *.cpp -pthread -std=gnu++11
./a.out
