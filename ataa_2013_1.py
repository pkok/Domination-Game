#!/usr/bin/env python

import math
import sys

from domination import core, scenarios

FIELD = """
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ w W W W w w w w w w w w w w w _ _ _ _ _ _ w
w _ _ _ w _ _ w _ _ _ _ _ _ _ _ _ _ A _ _ _ _ _ W _ _ _ w
w R _ _ w _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ W _ _ B w
w R _ _ w _ _ w _ _ _ _ w w w w w _ _ _ _ w _ _ W _ _ B w
w R _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ W _ _ B w
w _ _ _ w _ _ _ _ _ A _ _ _ _ _ _ _ _ _ _ w _ _ W _ _ _ w
w _ _ _ _ _ _ w w w w w w w w w w w W W W w _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
"""

class Tournament1(scenarios.Scenario):
    REPEATS   = 1000
    GENERATOR = None
    FIELD     = core.Field.from_string(FIELD)
    SETTINGS  = core.Settings(max_steps=300,
                              max_score=100,
                              spawn_time=10,
                              ammo_amount=1,  
                              ammo_rate=9,
                              max_range=60,
                              max_see=80,
                              max_turn=math.pi/4,
                              think_time=0.06,)

if __name__ == "__main__":
    Tournament1.test(red="my_agent.py", blue="domination/agent.py")