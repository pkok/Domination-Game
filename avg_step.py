#!/usr/bin/env python
import sys
if __name__ == "__main__":
    if len(sys.argv) > 1:
        f = open(sys.argv[1])
        total = 0
        count = 0
        for line in f.readlines()[1:]:
            total += int(line.split(',')[8])
            count += 1
        print "Average step count: " + str(total) + "/" + str(count) + " = " + str(float(total)/count)
