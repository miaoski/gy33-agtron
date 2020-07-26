# -*- coding: utf8 -*-
# Convert weights JSON to C

import json
import sys

wx = json.load(open(sys.argv[1]))

print('#ifndef GY33_PARAM_H')
print('#include <MatrixMath.h>')
print()

for i in range(len(wx)):
    if type(wx[i][0]) == type([]):
        x, y = len(wx[i]), len(wx[i][0])
        print('mtx_type W{}[{}][{}] = {{'.format(i, x, y))
        for j in range(x):
            print('{%s},' % repr(wx[i][j])[1:-1])
        print('};')
    else:
        x = len(wx[i])
        print('mtx_type W{}[{}] = {{{}}};'.format(i, x, repr(wx[i])[1:-1]))

print('#endif	// GY33_PARAM_H')
