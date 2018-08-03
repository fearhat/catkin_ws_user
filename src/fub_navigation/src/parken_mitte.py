#!/usr/bin/env python2

with open('parkenmitte.txt', 'w') as the_file:
    # the_file.write('Hello\n')
    startx = 2.0
    startx2 = 4.0
    r = 21
    for i in range(20, 41):
        the_file.write(
            "1.2." + str(i - 20) + " " + str(startx) + " 1.9\n"
        )
        startx += 0.1
    for i in range(40, 19, -1):
        the_file.write(
            "1.2." + str(r) + " " + str(startx2) + " 2.0\n"
        )
        startx2 -= 0.1
        r += 1
