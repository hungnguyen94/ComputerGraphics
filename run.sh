#!/bin/sh

#  run.sh
#  
#
#  Created by Ruben Wiersma on 04-06-15.
#
g++ -framework OpenGL -framework GLUT -I. *.cpp
./a.out
