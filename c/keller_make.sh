#!/bin/bash

echo -e "\nCompiling Keller Acculevel SN: 201549 data acquisition code using RS485 . . . \c"
gcc  keller_acculevel_201549_daq.c -g -Wall -lm -o swl7_daq  `pkg-config --cflags --libs libmodbus`

echo -e "done!\n"

rm -f *~ > /dev/null
