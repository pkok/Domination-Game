#!/usr/bin/env python

import sys
import math
import os
import datetime
import argparse
import cPickle as pickle
from shutil import copyfile
from domination import core, scenarios

FIELD1 = """
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ w _ C _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ w _ _ _ w w w w w w w w w w w _ _ _ _ _ _ w
w _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ w
w R _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ B w
w R _ w _ _ _ w _ A _ _ w w w w w _ _ A _ w _ _ _ w _ B w
w R _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ B w
w _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ w
w _ _ _ _ _ _ w w w w w w w w w w w _ _ _ w _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ C _ w _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
"""

FIELD2 = """
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ w w w w w w w w w w w w w w w _ _ _ _ _ _ _ w
w _ _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ A _ _ _ _ _ _ w _ _ _ w
w R _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ B w
w R _ _ w _ _ _ w _ _ _ _ w w w w w _ _ _ _ w _ _ _ w _ _ B w
w R _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ B w
w _ _ _ w _ _ _ _ _ _ A _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ _ w
w _ _ _ _ _ _ _ w w w w w w w w w w w w w w w _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
"""

FIELD3 = """
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ w w w w w w w w w w w w w w w _ _ _ _ _ _ _ w
w _ _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ A _ _ _ _ _ _ w _ A A w
w R _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ A B w
w R _ _ w _ _ _ w _ _ _ _ w w w w w _ _ _ _ w _ _ _ w _ A A w
w R _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ A B w
w _ _ _ w _ _ _ _ _ _ A _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ A A w
w _ _ _ _ _ _ _ w w w w w w w w w w w w w w w _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
"""

class Tournament1(scenarios.Scenario):
    REPEATS   = 1000
    GENERATOR = None
    FIELD     = core.Field.from_string(FIELD1)
    SETTINGS  = core.Settings(max_steps=300,
                              max_score=100,
                              spawn_time=10,
                              ammo_amount=100,  
                              ammo_rate=9,
                              max_range=60,
                              max_see=80,
                              max_turn=math.pi/4,
                              think_time=0.06,)

class Tournament2(scenarios.Scenario):
    REPEATS    = 1
    GENERATOR  = None
    SWAP_TEAMS = False
    FIELD      = core.Field.from_string(FIELD3)
    SETTINGS   = core.Settings(max_steps=300,
                              max_score=100,
                              spawn_time=10,
                              ammo_amount=10,
                              ammo_rate=9,
                              max_range=60,
                              max_see=80,
                              max_turn=math.pi/4,
                              think_time=0.06,
                              capture_mode=core.CAPTURE_MODE_MAJORITY)


# This is the code that is used for running a tournament, in order to run a 
# tournament in parallel, agents are temporarily copied, and blob data is not
# preserved. Please refer to "Running a Game" in the documentation for how
# to set up your own learning environment.
if __name__ == '__main__':

    # Use the -t flag to run a single rendered game. Otherwise run full tournament.
    now = datetime.datetime.now()
    parser = argparse.ArgumentParser(description='Process test flag.')
    parser.add_argument('-red', '--red_agent', type=str, default='my_agent_experiment.py')
    parser.add_argument('-blue', '--blue_agent', type=str, default='domination/enemy_agent_experiment.py')
    parser.add_argument('-s', '--save_blob', type=str, default = '')
    parser.add_argument('-n', '--new_blob', action='store_true')
    parser.add_argument('-t', '--test', action='store_true')
    parser.add_argument('-v', '--visuals', action='store_true')
    parser.add_argument('-r', '--repeats', type=int, default=100)  ## TODO: Dit doorgeven aan class Tournament2. Hoe?
    args = parser.parse_args(sys.argv[1:])

    if args.save_blob != '':
        copyfile('my_agent_blob', 'my_agent_blob_' + args.save_blob)
        "Saved old blob as: my_agent_blob_" + args.save_blob

    if args.new_blob == True:
        print "Deleted old blob. New blob added."
        state_action_pairs = {}
        blobfile = open("my_agent_blob", 'wb')
        pickle.dump(state_action_pairs, blobfile, pickle.HIGHEST_PROTOCOL)
        blobfile.close()

    if args.test == True:
        # TOD: Waarom savet de agents alleen zijn blobs in test mode? Should be fixed!
        tournament = Tournament2()
        match_id = hash((args.red_agent, args.blue_agent))
        tournament._single(red=args.red_agent, blue=args.blue_agent,
                matchinfo=scenarios.MatchInfo(1, 1, match_id, 1),
                rendered=True, verbose=True)
    else:
        now = datetime.datetime.now()
        folder = os.path.join('tournaments', now.strftime("%Y%m%d-%H%M"))
        Tournament2.tournament(agents=[args.red_agent, args.blue_agent], output_folder=folder, rendered=args.visuals)
