#!/usr/bin/env python

import transforms3d

mat = [
        [0.5073531540773762, 0.030307666804938516, -0.8612050989053506, -0.24565361512451578],
        [-0.0550239220297451, 0.998481323393445, 0.0027230201863000306, 0.008125398997600489],
        [0.8599797352567085, 0.04600534933365134, 0.5082502954061966, -0.13496362040050916],
        [0.0, 0.0, 0.0, 1.0]
        ]


al, be, ga = transforms3d.euler.mat2euler(mat, 'syxz')
Tdash, Rdash, Zdash, Sdash = transforms3d.affines.decompose(mat)

print(al, be, ga)
print(Tdash)

