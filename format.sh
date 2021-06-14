#!/usr/bin/bash

value_dir='application bsp Core/Src/main.c'

find $value_dir -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format --verbose -style=file -i {} \;
