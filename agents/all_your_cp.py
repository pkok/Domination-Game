"""
UvA - MSc AI - ATAA

Domination Game Agent for team ATAAACK


(C) Michael Cabot, Chiel Kooijman, Steven Laan, Camiel Verschoor, Auke Wiggers
"""

import time
import math
import sys
from libs.munkres import Munkres
import copy
from random import sample, choice, randint
import numpy as np
import cPickle as pickle

from domination.utilities import point_dist, line_intersects_grid, angle_fix

# Visualization
DRAW = True
DRAW_MESH = True
DRAW_HEATMAP = True
DRAW_HYPOTHESES = True
DRAW_FOW = True
DRAW_GOAL = True
PRINT_RANKINGS = True
FRAME_BY_FRAME = True
DRAW_AMMO = True
MULTI_TANK_OBJECTIVE = True
COORDINATION_GRAPH = True
MAMDP = True

# So that pylint and pyflakes don't complain
TEAM_RED = 0
TEAM_BLUE = 1
TEAM_NEUTRAL = 2

# radii
CP_RADIUS = 12
AMMO_RADIUS = 8
TANK_RADIUS = 6
SAFE_MARGIN = 0.06

# Map with extra points of interest for the mesh
MAP_GRID = """
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ # _ _ _ _ _ C _ _ _ _ _ _ _ # _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ # _ w w w w w w w w w w w w w w w _ # _ _ _ _ _ w
w _ _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ A _ _ _ _ _ _ w _ _ _ w
w R _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ B w
w R _ _ w _ _ _ w _ _ _ _ w w w w w _ _ _ _ w _ _ _ w _ _ B w
w R _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ B w
w _ _ _ w _ _ _ _ _ _ A _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ _ w
w _ _ _ _ _ # _ w w w w w w w w w w w w w w w _ # _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ # _ _ _ _ _ _ _ C _ _ _ _ _ # _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
"""

ASCII_WOLVES = ['''
                     ,ood8888booo,
                  ,od8           8bo,
               ,od                   bo,
             ,d8                       8b,
            ,o                           o,    ,a8b
           ,8                             8,,od8  8
           8'                             d8'     8b
           8                           d8'ba     aP'
           Y,                       o8'         aP'
            Y8,                      YaaaP'    ba
             Y8o                   Y8'         88
              `Y8               ,8"           `P
                Y8o        ,d8P'              ba
           ooood8888888P"""'                  P'
        ,od                                  8
     ,dP     o88o                           o'
    ,dP          8                          8
   ,d'   oo       8                       ,8
   $    d$"8      8           Y    Y  o   8
  d    d  d8    od  ""boooooooob   d"" 8   8
  $    8  d   ood' ,   8        b  8   '8  b
  $   $  8  8     d  d8        `b  d    '8  b
   $  $ 8   b    Y  d8          8 ,P     '8  b
   `$$  Yb  b     8b 8b         8 8,      '8  o,
        `Y  b      8o  $$o      d  b        b   $o
         8   '$     8$,,$"      $   $o      '$o$$
          $o$$P"                 $$o$

''','''
                               __
                             .d$$b
                           .' TO$;\\
                          /  : TP._;
                         / _.;  :Tb|
                        /   /   ;j$j
                    _.-"       d$$$$
                  .' ..       d$$$$;
                 /  /P'      d$$$$P. |\\
                /   "      .d$$$P' |\^"l
              .'           `T$P^"""""  :
          ._.'      _.'                ;
       `-.-".-'-' ._.       _.-"    .-"
     `.-" _____  ._              .-"
    -(.g$$$$$$$b.              .'
      ""^^T$$$P^)            .(:
        _/  -"  /.'         /:/;
     ._.'-'`-'  ")/         /;/;
  `-.-"..--""   " /         /  ;
 .-" ..--""        -'          :
 ..--""--.-"         (\      .-(\\
   ..--""              `-\(\/;`
     _.                      :
                             ;`-
                            :\\
                            ;
''','''
                   ooo OOO OOO ooo
               oOO                 OOo
           oOO                         OOo
        oOO                               OOo
      oOO                                   OOo
    oOO                  _                    OOo
   oOO                  /.|                    OOo
  oOO                  /  |   _                 OOo
 oOO                  |   |  / |                 OOo
 oOO                 /    | /  |                 OOo
 oOO                / /|   V   |                 OOo
 oOO              _/          /                  OOo
 oOO           _-"           /                   OOo
  oOO         <______,      /                   OOo
   oOO              |      |                   OOo
    oOO             |       \                 OOo
      oOO          /         \              OOo
        oO        /           |          OOo
           oOO   /            |       OOo
               o/            /    OOo
                            / ooo
''']

class General():

    # Gamestates
    INIT = -2
    LOSING = -1
    EVEN = 0
    WINNING = 1

    NUM_AGENTS = 3

    # Killcount etc
    PWND = {3: ' is on a Killing Spree',  # 5 kills.
            4: ' just made a Mega Kill',
            5: ' is Dominating',          # 10 kills
            6: ' is Unstoppable',         # 20 kills.
            7: ' is on a Rampage',        # 15 kills.
            8: ' made a Monster Kill',
            9: ' is GODLIKE',
            'fb': ' drew First Blood',
            'br': ' - DO A BARREL ROLL',
            'sl': ', what does the scouter say about his power level?'
            }
    NAMES_POOL = ['Auke', 'Camiel', 'Chiel', 'Michael', 'Steven']

    def __init__(self, mesh, all_agents, team, settings, grid, blob):

        # Print a wolf!
        print choice(ASCII_WOLVES)
        print '\n             AHOOOOOOOOOOOOO!\n\n'

        # Read the binary blob, we're not using it though
        if blob is not None:
            # Remember the blob path so we can write back to it
            try:
                self.blobpath = blob.name
            except:
                pass
            try:
                self.blobcontent = pickle.loads(blob.read())
            except:
                print "Could not load blob."
            try:
                self.h_cache = self.blobcontent["h_cache"]
                print "Loaded h_cache from blob."
                print "Size: %d" % len(self.h_cache)
            except:
                self.h_cache = {}
                print "Could not load h_cache."
            try:
                self.c_cache = self.blobcontent["c_cache"]
                print "Loaded c_cache from blob."
                print "Size: %d" % len(self.c_cache)
            except:
                self.c_cache = {}
                print "Could not load c_cache."
            # Reset the file so other agents can read it too.
            try:
                blob.seek(0)
            except:
                pass
        else:
            print "No blob found."
            self.blobpath = None
            self.h_cache = {}
            self.c_cache = {}

        # Dynamic world variables
        self.ammo_info = {}         # Spawntime indexed by ammo_loc
        self.cps = []               # Controlpoint (x,y,state)
        self.foes = []              # Foes by location (x,y,r)
        self.foe_respawns = []      # Respawn times of foes
        # Agent variables
        self.ammos = []
        self.visible_targets = []
        self.spawn_info = []

        self.all_agents = all_agents
        self.team = team
        self.settings = settings
        self.grid = grid

        self.original_mesh = copy.deepcopy(mesh)
        self.mesh = self.init_mesh(mesh)
        self.state = self.INIT
        self.plan = self.plan_init

        # Add interesting nodes to the map, and set initial info
        for node in locs_from_map(MAP_GRID, '#'):
            self.add_to_mesh(mesh, node, max_dist=2*self.settings.max_speed)
        for ammo in [a for a in locs_from_map(MAP_GRID, 'A')]:
            self.ammo_info[ammo] = 0
            self.add_to_mesh(mesh, ammo)
        for cp in [c for c in locs_from_map(MAP_GRID, 'C')]:
            self.cps.append(cp + (0,))
            self.add_to_mesh(mesh, cp)

        self.h = len(grid)
        self.w = len(grid[0])

        # Create field of view map
        self.field_of_view = np.ones((self.h * self.settings.tilesize,
                                      self.w * self.settings.tilesize))
        self.field_of_view_decay = 6

        # Current kills, total kills, total deaths
        self.names = sample(self.NAMES_POOL, 3)
        self.kd_ratio = [[0, 0] for i in range(self.NUM_AGENTS)]

        # Initial orders are empty
        self.orders = {0: None, 1: None, 2: None}

        red_spawns = []
        for red_sp in [s for s in locs_from_map(MAP_GRID, 'R')]:
            red_spawns.append(red_sp)
        blue_spawns = []
        for blue_sp in [s for s in locs_from_map(MAP_GRID, 'B')]:
            blue_spawns.append(blue_sp)
        if team == TEAM_BLUE:
            red_spawns = [r for r in locs_from_map(MAP_GRID, 'R')]
            self.enemy_spawn_point = red_spawns[1] + (0.0,)
        else:
            blue_spawns = [b for b in locs_from_map(MAP_GRID, 'B')]
            self.enemy_spawn_point = blue_spawns[1] + (math.pi,)

        # error caused by bumping into wall or (enemy)agent
        self.dist_error = settings.tilesize
        self.angle_error = settings.max_turn/2
        # index is enemy_id, element is [enemy, last-update]
        self.enemy_IDs = []
        # enemies killed in last time step
        self.enemies_killed_IDs = []

    def strategize(self):
        """One iteration for the general class."""
        self.update_knowledge()
        self.plan()
        self.give_orders()

    def update_knowledge(self):
        self.visible_targets = list()
        self.spawn_info = list()
        self.ammos = list()
        self.ammopacks = set()
        self.foes = set()
        self.targetable_foes = set()

        # Update the field of view
        fov = self.field_of_view
        fov = fov + 1.0 / self.field_of_view_decay
        fov[fov > 1.0] = 1.0

        max_see = self.settings.max_see

        for agent in self.all_agents:
            obs = agent.observation

            # Seen foes, indexed by agentid, values are dicts:
            # vt[a_id] = {foe : angle_needed_for_kill}
            if obs.ammo:
                vt = get_vsbl_targets(agent.loc, obs.foes, obs.friends,
                                      agent.corners, self.settings.max_turn,
                                      self.settings.max_range,
                                      self.settings.tilesize, agent.grid,
                                      agent_radius=6)
                self.visible_targets.append(vt)
                self.targetable_foes |= set(vt.keys())
            else:
                self.visible_targets.append({})

            # Present ammopacks (x, y)
            packs = (x[:2] for x in
                     filter(lambda x: x[2] == 'Ammo', obs.objects))
            self.ammopacks |= set(packs)

            # Present foes (x, y, theta)
            self.foes |= set(obs.foes)

            self.ammos.append(obs.ammo)

            self.spawn_info.append(obs.respawn_in)

            self.field_of_view[obs.loc[1]-max_see:obs.loc[1]+max_see+1,
                               obs.loc[0]-max_see:obs.loc[0]+max_see+1] = 0.0

        for ammopack in self.ammopacks:
            self.ammo_info[ammopack] = 0

        game_score = self.all_agents[0].observation.score
        cps = self.all_agents[0].observation.cps
        cps_foe = len([c for c in cps if c[2] != self.team and c[2] != 2])
        cps_friend = len([c for c in cps if c[2] == self.team])
        cps_even = len(cps) - cps_foe - cps_friend
        self.cps = cps

        if cps_even == len(cps):
            if max(self.spawn_info) == -1 and game_score[0] == game_score[1] \
                    and max(self.ammos) < 1:
                self.state = self.INIT
            else:
                self.state = self.EVEN
        elif cps_foe > cps_friend:
            self.state = self.LOSING
        elif cps_foe == cps_friend:
            self.state = self.EVEN
        else:
            self.state = self.WINNING

        # Update enemy respawn timers
        for i, spawn_time in enumerate(self.foe_respawns):
            self.foe_respawns[i] = max(0, spawn_time-1)

        ammo_rate = self.settings.ammo_rate
        ammo_info = self.ammo_info

        # Update ammo timers
        for ammo, count in self.ammo_info.iteritems():
            ammo_info[ammo] = max(0, count - 1)
            if count == 0 and not ammo in self.ammopacks:
                for a in self.all_agents:
                    if point_dist(a.loc, ammo) < max_see:
                        # assume it's gone
                        # TODO This is worst case only
                        ammo_info[ammo] = ammo_rate

        self.who_shoots_who()

    def plan_init(self):
        """Since the start is hardcoded, might as well give it its own
           function.
        """
        if self.state == self.INIT:
            best_ammo, best_time, best_diff \
                    = self.all_agents[0].\
                      get_best_ammo_loc(self.ammo_info)

            points_of_interest = [cp[:2] for cp in self.cps] + [best_ammo[:2]]
            orders = [(self.cp_attack, cp) for cp in self.cps] + \
                     [(self.get_ammo, self.ammo_info)]
            cost_matrix = [[], [], []]
            for i, p in enumerate(points_of_interest):
                for a in self.all_agents:
                    if p == best_ammo:
                        cost = a.paths[p][1] * 2
                    else:
                        cost = a.paths[p][1]
                    cost_matrix[i].append(cost)
            munkres = Munkres()
            indices = munkres.compute(cost_matrix)
            for p_id, a_id in indices:
                self.orders[a_id] = orders[p_id]
        else:
            self.plan = self.plan_other
            self.plan()

    def plan_other(self):
        """Plan meta-actions for all agents."""

        cps_to_get = [cp[:2] for cp in self.cps if cp[2] != self.team]

        for a_id, (order, goal) in self.orders.iteritems():
            agent = self.all_agents[a_id]
            goal_reached = agent.goal_reached()

            if order == self.get_ammo:
                self.orders[a_id] = (self.get_ammo, self.ammo_info)

                if self.spawn_info[a_id] > -1 or agent.ammo > 1:
                    best_ammo, best_time, best_diff = \
                        agent.get_best_ammo_loc(self.ammo_info)

                    agents = [a for a in self.all_agents if a.id != a_id]
                    new_ammohogger = self.closest_to_point(best_ammo,
                                                           agents)
                    __, __, new_diff = new_ammohogger.get_best_ammo_loc(self.ammo_info)
                    if new_diff < 4:
                        self.switch_roles(a_id, new_ammohogger.id)

            elif order == self.cp_defend:
                if goal[:2] in cps_to_get:
                    self.orders[a_id] = (self.cp_attack, goal)

            elif order == self.cp_attack:
                if agent.ammo and goal_reached:
                    self.orders[a_id] = (self.cp_defend, goal)


    def give_orders(self):
        for agent in self.all_agents:
            (command, arg) = self.orders[agent.id]
            command(agent, arg)

    def switch_roles(self, id1, id2):
        temp = self.orders[id1]
        self.orders[id1] = self.orders[id2]
        self.orders[id2] = temp

    ### META-ACTION FUNCTIONS ###
    def cp_defend(self, agent, cp):
        spot, d = agent.closest_object(agent.campspots)
        if spot and point_dist(spot[:2], agent.loc[:2]) < \
                agent.settings.max_range and \
                point_dist(spot[:2], cp[:2]) < \
                agent.settings.max_range:
            agent.goal = spot
        else:
            agent.goal = cp[:2]

    def cp_attack(self, agent, cp):
        agent.goal = cp[:2]

    def get_ammo(self, agent, ammo_info):
        best_ammo, best_time, best_diff= agent.get_best_ammo_loc(ammo_info)
        agent.goal = best_ammo


    def closest_to_point(self, point, agentlist):
        best_dist = float('inf')
        best_agent = None
        agentlist = agentlist if agentlist else self.all_agents

        userealdist = True
        for agent in agentlist:
            if point[:2] not in agent.paths:
                userealdist = False

        for agent in agentlist:
            if userealdist:
                dist = agent.paths[point[:2]][1]
            else:
                dist = point_dist(agent.loc[:2], point[:2])
            if dist < best_dist:
                best_dist = dist
                best_agent = agent
        return best_agent

    def who_shoots_who(self):
        # Variables needed to call visible_targets
        max_turn = self.settings.max_turn
        max_range = self.settings.max_range
        tilesize = self.settings.tilesize
        corners = self.all_agents[0].corners
        grid = self.all_agents[0].grid

        # Get all involved agents and some of their info
        involved_agents = [a for a in self.all_agents if \
                           self.visible_targets[a.id]]

        if not involved_agents:
            return

        agent_locs = [a.loc for a in involved_agents]
        involved_targets = [self.visible_targets[a.id] \
                            for a in involved_agents]
        targetable_foes = list(self.targetable_foes)
        all_foes = self.foes
        cost_dict = {a: {f: 100 for f in targetable_foes} for a in agent_locs}
        for i, foe_loc in enumerate(targetable_foes):
            foe_friends = [f for f in all_foes if foe_loc != f]
            vt = get_vsbl_targets(foe_loc, agent_locs, foe_friends, corners,
                                 max_turn, max_range, tilesize, grid,
                                 agent_radius=6)

            for agent_loc in vt.keys():
                cost_dict[agent_loc][foe_loc] -= 100

        for a_id, foe_dict in enumerate(involved_targets):
            for foe_loc in foe_dict:
                cost_dict[agent_locs[a_id]][foe_loc] -= 10

        cost_matrix = [[0 for f in targetable_foes] for a in agent_locs]
        for i, agent_loc in enumerate(agent_locs):
            agent = involved_agents[i]

            for j, foe_loc in enumerate(targetable_foes):
                cost = cost_dict[agent_loc][foe_loc]
                cost -= agent.ammo
                cost_matrix[i][j] = cost
        munkres = Munkres()
        indices = munkres.compute(cost_matrix)
        for agent_id, foe_id in indices:
            real_agent_id = involved_agents[agent_id].id
            foe_loc = targetable_foes[foe_id]
            self.visible_targets[real_agent_id] = \
                (foe_loc,) + tuple(self.visible_targets[real_agent_id][foe_loc])

        for agent_id, agent in enumerate(involved_agents):
            if not agent_id in [a_id for a_id,f_id in indices]:
                real_agent_id = agent.id
                self.visible_targets[real_agent_id] = {}

    def init_mesh(self, mesh):
        """Recompute costs as multitudes of the max_speed of the agent,
        so that we can derive the number of turns needed for the movement.

        Mesh is a dict of dicts mesh[(x1, y1)][(x2, y2)] = cost
        """
        # Nodes are given equal cost if in same movement range
        max_speed = self.settings.max_speed
        for node1 in mesh:
            for node2 in mesh[node1]:
                # Get the old movement cost
                cost = int(math.ceil(mesh[node1][node2] / float(max_speed)))
                mesh[node1][node2] = cost * max_speed
        return mesh

    def add_to_mesh(self, mesh, node, bonus=0, max_dist=1e4):
        """ Add one node to the mesh, connect to every possible
            present node, give it bonus (so agents have an incentive
            to reach it). Permanent effect, and only adds to the generals
            mesh (not the local ones the agents have).

            This overrides previous values completely.
        """
        mesh[node] = dict()
        for n in mesh:
            d = point_dist(node, n)
            if d < max_dist and\
               not agent_intersects_grid(node, n, self.all_agents[0].grid,
                                         self.settings.tilesize):

                cost = (int(math.ceil(d / float(self.settings.max_speed))) *
                        self.settings.max_speed - bonus)

                mesh[node][n] = cost
                mesh[n][node] = cost
        return mesh

    def foe_killed(self, foe, agent_id):
        """Signal that an enemy has been killed. Resets the respawn counter."""
        for i, respawn in enumerate(self.foe_respawns):
            if respawn == 0:
                self.foe_respawns[i] = self.settings.spawn_time
                break

    def finalize(self):
        try:
            if self.blobpath is not None:
                # We simply write the same content back into the blob.
                # in a real situation, the new blob would include updates to
                # your learned data.
                blobfile = open(self.blobpath, 'wb')
                self.blobcontent = {}
                self.blobcontent["h_cache"] = self.h_cache
                self.blobcontent["c_cache"] = self.c_cache
                pickle.dump(self.blobcontent, blobfile,
                            pickle.HIGHEST_PROTOCOL)
                print "Cache written to blob."
        except:
            # We can't write to the blob, this is normal on AppEngine since
            # we don't have filesystem access there.
            print "Could not write to blob."

        if PRINT_RANKINGS:
            """Print necessary matchinfo, handle stuff."""
            toprint = '===== RANKINGS ======'
            maxlen = max(len(k) for k in [a.name for a in self.all_agents])
            toprint += '\n{0}   Kills\tDeaths'.format('Agent'.ljust(maxlen))
            for a in self.all_agents:
                toprint += '\n{0} :  {1}\t {2}'.format(a.name.ljust(maxlen),
                                                       self.kd_ratio[a.id][0],
                                                       self.kd_ratio[a.id][1])
            print toprint + '\n'

    def __str__(self):
        """ Tostring method prints all items of the General object.
        """
        items = sorted(self.__dict__.items())
        maxlen = max(len(k) for k, v in items)
        return '== General ==\n' + '\n'.join(('%s : %r' %
                                              (k.ljust(maxlen), v))
                                             for (k, v) in items)

class Agent(object):
    NAME = "All your cp"

    def __init__(self, id, team, settings=None, field_rects=None,
                 field_grid=None, nav_mesh=None, blob=None, **kwargs):
        """Each agent is initialized at the beginning of each game.
        The first agent (id==0) can use this to set up global variables. Note
        that the properties pertaining to the game field might not be given for
        each game.
        """
        self.id = id
        self.team = team
        self.grid = field_grid
        self.corners = get_corners(field_grid)
        self.campspots = get_campspots(field_grid)
        self.settings = settings
        self.goal = None
        self.callsign = '%s-%d' % (('BLU' if team == TEAM_BLUE else 'RED'), id)

        # Recommended way to share variables between agents.
        if id == 0:
            all_agents = [self]
            self.general = self.__class__.general = \
                General(nav_mesh, all_agents, team, settings, self.grid, blob)
        else:
            self.general.all_agents.append(self)

        # Set the agents personal navmesh
        self.mesh = copy.deepcopy(self.general.mesh)

        # Set name for killcount
        self.name = self.general.names[self.id]
        self.streak = 0

    def debug(self, surface):
        pass

    def observe(self, observation):
        """ Each agent is passed an observation using this function,
            before being asked for an action. You can store either
            the observation object or its properties to use them
            to determine your action. Note that the observation object
            is modified in place.
        """
        time_observe = time.time()

        self.observation = observation
        self.selected = self.observation.selected
        self.loc = self.observation.loc + (self.observation.angle,)
        self.ammo = self.observation.ammo

        # Update k/d
        if self.observation.respawn_in == self.settings.spawn_time - 1:
            self.streak = 0
            self.general.kd_ratio[self.id][1] += 1

        # Initialize the agent mesh by adding self.loc
        self.set_mesh()

        # Precompute paths to several points
        self.paths = dict()
        for goal in (self.general.cps + self.general.ammo_info.keys()):
            self.paths[goal[:2]] = self.find_optimal_path(goal=goal)

        # Additional info on each observed foe for easy later use
        foes = {}
        for foe in observation.foes:
            self.paths[foe[:2]] = self.find_optimal_path(goal=foe)

            rel_angle_to_foe = get_rel_angle(self.loc, foe)
            rel_angle_from_foe = get_rel_angle(foe, self.loc)
            point_dist_to_foe = point_dist(self.loc, foe[:2])
            foes[foe] = (rel_angle_to_foe,
                         rel_angle_from_foe,
                         point_dist_to_foe)
        self.foes = foes

        # All agent observations are present
        if self.id == len(self.general.all_agents) - 1:
            self.general.strategize()

        self.time = time.time() - time_observe

    def action(self):
        """ This function is called every step and should
            return a tuple in the form: (turn, speed, shoot)
        """
        time_action = time.time()

        # Initialize shoot
        shoot = False

        # Drive to where the user clicked
        # Clicked is a list of tuples of (x, y, shift_down, is_selected)
        if self.selected and self.observation.clicked:
            self.goal = self.observation.clicked[0][0:2]

        # Find the optimal path through the mesh
        if self.goal[:2] in self.paths:
            path, dist = self.paths[self.goal[:2]][:2]
        else:
            path, dist = self.find_optimal_path()[:2]

        # Convert path to useful movement info
        if path:
            node = path[0]
            dx = node[0] - self.loc[0]
            dy = node[1] - self.loc[1]

            if len(path) > 1 and abs(dx) < 1 and abs(dy) < 1:
                path.remove(node)
                node = path[0]
                dx = node[0] - self.loc[0]
                dy = node[1] - self.loc[1]

            angle = get_rel_angle(self.loc, node)

            if self.goal_reached():
                speed = 0
                turn = 0

                if len(self.goal) > 2:
                    turn = angle_fix(self.goal[2] - self.loc[2])

                elif self.ammo:
                    if self.foes:
                        closest_foe, dist = self.closest_foe()

                        if self.selected:
                            print closest_foe, dist
                        if dist / 40 < 5:
                            turn = get_rel_angle(self.loc, closest_foe)
                        else:
                            turn = angle_fix((self.team == TEAM_BLUE) * math.pi
                                    - self.loc[2])
                    else:
                        turn = angle_fix((self.team == TEAM_BLUE) * math.pi
                                - self.loc[2])

            elif abs(angle_fix(angle - math.pi)) < self.settings.max_turn:
                speed = -(dx**2 + dy**2)**0.5
                turn = angle_fix(angle - math.pi)
            else:
                speed = (dx**2 + dy**2)**0.5
                turn = angle_fix(angle)
        else:
            print 'ERROR: No path found to goal {0}'.format(self.goal)
            turn = 0
            speed = 0

        # Always go for targets!
        foe = None
        visible_targets = self.general.visible_targets[self.id]
        if self.ammo and visible_targets:
            foe, foe_a1, foe_a2 = visible_targets

            foe_a1 += SAFE_MARGIN
            foe_a2 -= SAFE_MARGIN
            if abs(turn) > self.settings.max_turn:
                turn = math.copysign(self.settings.max_turn, turn)
            if foe_a2 <= turn:
                turn = foe_a2
            elif foe_a1 >= turn:
                turn = foe_a1
            elif foe_a1 <= turn <= foe_a2:
                pass        # Free move: Use original turn
            else:
                print 'ERROR: Foe_angles {0}, {1} for turn {2}'.format(foe_a1,
                                                                       foe_a2,
                                                                       turn)

            # And shoot the selected target
            speed = self.recompute_speed(speed, turn, dist)
            shoot = True

            # Update kill streak
            self.streak += 1
            self.general.kd_ratio[self.id][0] += 1
            if self.streak in self.general.PWND:
                print 'Agent {0}{1}'.format(self.general.names[self.id],
                                                self.general.PWND[self.streak])

        if abs(turn) > self.settings.max_turn:
            max_turn = math.copysign(self.settings.max_turn, turn)
            speed = self.recompute_speed(speed, max_turn, dist)
            shoot = False

        if shoot:
            self.general.foe_killed(foe, self.id)

        # Remove any added tempnodes
        self.reset_mesh()

        self.time += time.time() - time_action
        #print 'Agent {0} takes {1} seconds'.format(self.id, self.time)

        return (turn, speed, shoot)
    def finalize(self, interrupted=False):
        """This function is called after the game ends, either due to
        time/score limits, or due to an interrupt (CTRL+C) by the user. Use it
        to store any learned variables and write logs/reports.
        """
        if self.id == len(self.general.all_agents) - 1:
            self.general.finalize()

    def __str__(self):
        return 'Agent ' + self.id

    def step_cost(self, frm, to):
        """ Get the cost for a movement, cost of turning is slightly
            higher.
        """
        max_turn = self.settings.max_turn
        max_speed = self.settings.max_speed
        pi = math.pi

        straight = int(math.ceil(point_dist(frm, to) / float(max_speed)))

        angle = get_rel_angle(frm, to)
        turn = min(abs(angle), abs(angle_fix(angle - pi)))
        turn_times = 0
        if turn > max_turn:
            turn_times = int(turn/max_turn)
            if turn % max_turn == 0:
                turn_times -= 1
        turn_c = turn_times * max_speed
        straight_c = straight

        add_cost = 0
        return straight_c+turn_c, add_cost

    def recompute_speed(self, speed, turn, dist):
        """ Check if we need to slow down given our old and new turn, and
            the current path_dist to the goal.
        """
        tilesize = self.settings.tilesize

        # Whether we should reset speed depends on the agents next position
        next_angle = angle_fix(self.loc[2] + turn)
        next_loc = (self.loc[0] + math.cos(next_angle) * speed,
                    self.loc[1] + math.sin(next_angle) * speed,
                    next_angle)

        # If the next location is within boundaries of the board and legit
        if 0 <= next_loc[0] <= self.general.w * tilesize - 1 and \
            0 <= next_loc[1] <= self.general.h * tilesize - 1 and \
            not agent_intersects_grid(self.loc[:2], next_loc[:2],
                                      self.grid, tilesize):

            next_path, next_dist, add_c = self.find_optimal_path(loc=next_loc)
            # If the next path is longer than the old one, we should brake
            if next_dist < dist:
                return speed

        return 0

    def set_mesh(self):
        """ Initializes the agent mesh by adding all direct connections from
            the current position to neighbouring nodes. Note that extra
            connections are also added if we DO stand on a node!
        """
        mesh = self.mesh
        loc = self.loc[:2]
        to_remove_from_mesh = set()
        grid = self.grid
        tilesize = self.settings.tilesize

        if loc not in mesh:
            mesh[loc] = dict()
            self.remove_loc_from_mesh = True

        for node in (n for n in mesh if not
                     agent_intersects_grid(loc, n, grid, tilesize)):
            if node not in mesh[loc]:
                mesh[loc][node] = self.step_cost(self.loc, node)[0]
                to_remove_from_mesh.add((loc, node))

        self.to_remove_from_mesh = to_remove_from_mesh

    def reset_mesh(self):
        """ Reset the mesh to the 'general mesh' it was before we added any
            custom nodes.
        """

        mesh = self.mesh
        if self.remove_loc_from_mesh:
            del(mesh[self.loc[:2]])
            self.remove_loc_from_mesh = False
        else:
            for n1, n2 in self.to_remove_from_mesh:
                del(mesh[n1][n2])

    def find_optimal_path(self, loc=None, goal=None):
        """ Pathplanning! Uses navmesh nodes as astar nodes, cost adjusted
            to match the real cost of the movement (including turns).
        """
        # For readability
        grid = self.grid
        mesh = self.mesh
        tilesize = self.settings.tilesize
        max_turn = self.settings.max_turn
        max_speed = self.settings.max_speed

        # Define current and goal location
        loc = loc if loc else self.loc
        start = loc[:2]
        goal = goal if goal else self.goal
        end = goal[:2]

        # If there is a straight line, just return the end point
        if not agent_intersects_grid(start, end, grid, tilesize):
            # step_cost includes temporal information
            return ([goal],) + self.step_cost(loc, goal)

        # Add the loc to the mesh, if necessary
        remove_loc = False
        if start not in mesh:
            remove_loc = True
            # If this is a tempnode, remove it at the end
            mesh[start] = dict([(n, self.step_cost(loc, n)[0])
                                for n in mesh if not
                                agent_intersects_grid(start, n, grid,
                                                      tilesize)])
        # Add temp nodes for end:
        if end not in mesh:
            endconns = ((n, point_dist(end, n)) for n in mesh if not
                        agent_intersects_grid(end, n, grid, tilesize))

            for n, dst in endconns:
                cost = int(math.ceil(dst / float(max_speed))) * max_speed
                mesh[n][end] = cost

        neighbours = lambda n: [n2 + (get_angle(n, n2),)
                                for n2 in mesh[n[:2]].keys()]
        general = self.general

        # Goal disregards orientation (TODO)
        goal = lambda n: n[:2] == end

        # Heuristic: Euclidean distance
        heuristic = lambda n: ((n[0]-end[0])**2 + (n[1]-end[1])**2) ** 0.5

        output = astar(loc, neighbours, goal, 0.0, heuristic, mesh,
                       general, max_speed, max_turn, end)

        # Remove tempnodes from mesh again, if needed
        if remove_loc:
            del(mesh[start])
        if end not in mesh:
            i = 0
            for n in (n1 for n1, n2 in mesh.iteritems() if end in n2):
                i += 1
                del(mesh[n][end])

        # Heuristics cached on the fly
        d_loc = discretize(start)
        d_end = discretize(end)
        if (start, end) not in self.general.h_cache:
            self.general.h_cache[(d_loc, d_end)] = output[-2]
            # Correct for rotation on first node
            turn = get_rel_angle(loc, output[0][0])
            turn_times = 0
            if turn > max_turn:
                turn_times = int(turn/max_turn)
                if turn % max_turn == 0:
                    turn_times -= 1
            self.general.h_cache[(d_loc, d_end)] = (output[-2] -
                                                    max_speed * turn_times)
        return output

    def goal_reached(self):
        """Return if goal has been reached for this agent.

        An agent has reached its goal if it is sufficiently close.
        """
        if self.goal[:2] in self.general.ammo_info.keys(): # ammo
            max_dist_from_goal = TANK_RADIUS + AMMO_RADIUS - 1
        elif self.goal[:2] in [x[:2] for x in self.general.cps]: # capture
            max_dist_from_goal = TANK_RADIUS + CP_RADIUS - 1
        else:
            max_dist_from_goal = 2*TANK_RADIUS - 1

        return self.goal is not None and (point_dist(self.goal, self.loc) <
                                          max_dist_from_goal)

    ### BESTSOFAR FUNCTIONS ###
    def get_best_ammo_loc(self, ammo_info):
        """Returns the best place to go to for ammo plus time until it is
        spawned. Best place is based on time to spawn or turns needed to reach
        it.
        """
        best_time = float("inf")
        best_ammo = None
        best_diff = float("inf")
        for ammo, spawn_time in ammo_info.iteritems():
            d = self.paths[ammo][1]
            turnsneeded = int(math.ceil(d / self.settings.max_speed))

            # Currently, time needed to take ammo used to determine best
            time_needed = max(turnsneeded, spawn_time)

            if time_needed <= best_time:
                best_ammo = ammo
                best_time = time_needed
                best_diff = abs(turnsneeded - spawn_time)

            # TODO perhaps incorporate close opponents/danger?
            # TODO use diff between turns_needed and spawn_time
        return best_ammo, best_time, best_diff

    def closest_object(self, objectlist):
        """Get the closest objects from a given list of objects using a
        best-so-far method.
        """
        best_dist = float('inf')
        best_object = None
        for o in objectlist:
            if o[:2] in self.paths:
                dist = self.paths[o[:2]][1]
            else:
                dist = point_dist(self.loc[:2], o[:2])
            if dist < best_dist:
                best_dist = dist
                best_object = o
        return best_object, best_dist

    def closest_foe(self, foelist=None):
        """Get the closest foe from a given list or all foes."""
        foelist = foelist if foelist else self.general.foes
        return self.closest_object(foelist)

    def closest_cp(self, cplist=None):
        """Get the closest cp from a given list or all cp."""
        cplist = cplist if cplist else self.cps
        return self.closest_object(cplist)

# AUX FUNCTIONS

def get_campspots(walls, tilesize=16):
    pi = math.pi
    angle = [pi*-3/4, pi*3/4, -pi/4, pi/4]
    offset = [(-.5, -.5), (.5, -.5), (-.5, .5), (.5, .5)]
    spots = []
    for i in xrange(len(walls) - 1):
        for j in xrange(len(walls[0]) - 1):
            a = walls[i][j]
            b = walls[i + 1][j]
            c = walls[i][j + 1]
            d = walls[i + 1][j + 1]
            if a + b + c + d == 3:
                cornertype = 6 - (b + 2 * c + 3 * d)
                spots.append((tilesize * (j+1+offset[cornertype][1]),
                             tilesize * (i+1+offset[cornertype][0]),
                             angle[cornertype]))
    return spots


def get_corners(walls, tilesize=16):
    corners = []
    for i in xrange(len(walls) - 1):
        for j in xrange(len(walls[0]) - 1):
            a = walls[i][j]
            b = walls[i + 1][j]
            c = walls[i][j + 1]
            d = walls[i + 1][j + 1]
            if a + b + c + d == 1:
                cornertype = b + 2 * c + 3 * d
                corners.append((tilesize * (i + 1),
                                tilesize * (j + 1), cornertype))
    return corners


def corners_in_range(corners, loc, rng=60):
    for corner in corners:
        dx = corner[0] - loc[0]
        dy = corner[1] - loc[1]
        dist = (dx ** 2 + dy ** 2) ** 0.5
        # If dx > 0, corner_x > loc_x, so the corner
        # is right of the agent.

        # If a corner is right and above or left and down
        if (dx > 0 and dy > 0) or (dx < 0 and dy < 0):
            cornertypes = (1, 2)
        else:
            cornertypes = (0, 3)
        if dist <= rng and corner[2] in cornertypes:
            yield corner + (dist,)


def get_rel_angle(a1, a2):
    """Positive angle indicates a right turn. Relative call."""
    return angle_fix(math.atan2(a2[1]-a1[1], a2[0]-a1[0]) - a1[2])


def get_angle(a1, a2):
    """Absolute call."""
    return math.atan2(a2[1]-a1[1], a2[0]-a1[0])


def shootable(p1, r1, p2, r2):
    dist = point_dist(p1, p2)
    angle_to_center_p2 = get_rel_angle(p1, p2)

    angle_to_tangent = math.asin(r2 / dist)
    left = angle_to_center_p2 - angle_to_tangent
    right = angle_to_center_p2 + angle_to_tangent

    if -r2 < dist - r1 < r2:
        d_tangent = (dist**2 - r2**2)**0.5

        if d_tangent > r1:
            x = (dist**2 - (r2**2) + r1**2) / (2 * dist)
            y = (r1**2 - x**2)**0.5
            a = math.atan(y/x)
            left = angle_to_center_p2 - a
            right = angle_to_center_p2 + a
    elif dist - r1 < -r2:
        pass
    else:
        return False

    return (left, right, dist)


def agent_intersects_grid((x1, y1), (x2, y2), grid, tilesize, radius=4):
    dx = x2 - x1
    dy = y2 - y1

    theta_1 = angle_fix(math.atan2(dy, dx))
    theta_2 = 0.5 * math.pi - theta_1

    dx2 = math.cos(theta_2) * radius
    dy2 = math.sin(theta_2) * radius

    l_x1 = x1 - dx2
    l_y1 = y1 + dy2

    l_x2 = x2 - dx2
    l_y2 = y2 + dy2

    u_x1 = x1 + dx2
    u_y1 = y1 - dy2

    u_x2 = x2 + dx2
    u_y2 = y2 - dy2

    return (line_intersects_grid((l_x1, l_y1), (l_x2, l_y2), grid, tilesize) or
            line_intersects_grid((u_x1, u_y1), (u_x2, u_y2), grid, tilesize))


def locs_from_map(map_grid, sig='#', tilesize=16):
    """Yields all x, y coordinates of a certain character on the map."""
    for x, line in enumerate(map_grid.split('\n')[1:-1]):
        for y, char in enumerate(line[::2]):
            if char.lower() == sig.lower():
                yield int(tilesize * (y + .5)), int(tilesize * (x + .5))


def discretize(point, tilesize=16):
    """Return the discretised location of a position (to the middle of the
    corresponding tile).
    """
    x, y = point
    return (int(tilesize * (int(x / tilesize) + .5)),
            int(tilesize * (int(y / tilesize) + .5)))


def to_tile_loc(pos, tilesize=16):
    """Return the tile location of a position."""
    return (int(pos[0] / tilesize),
            int(pos[1] / tilesize))

def get_vsbl_targets(loc, foes, friends, corners, max_turn, max_range,
                    tilesize, grid, agent_radius=6):
    visible_targets = {}
    foes_in_range = []
    for foe in foes:
        if abs(get_rel_angle(loc, foe)) < 2 * max_turn:
            target = shootable(loc, max_range, foe, agent_radius)
            if target:
                foes_in_range.append((foe, target))
    if not foes_in_range:
        return visible_targets
    friends_in_range = []
    for friend in friends:
        if abs(get_rel_angle(loc, friend)) < 2 * max_turn:
            target = shootable(loc, max_range, friend, agent_radius + 1)
            if target:
                friends_in_range.append(target)
    corners = corners_in_range(corners, loc)
    for foe_loc, target in foes_in_range:
        obstacle_agents = [t for f,t in foes_in_range if f != foe] + \
                          [t for t in friends_in_range]

        foe_a1, foe_a2 = determine_fov(loc, target, obstacle_agents,
                                       corners, max_turn)
        if foe_a1 is not None and foe_a2 - foe_a1 > 2 * SAFE_MARGIN:
            foe_dist = target[2]
            real_angle1 = angle_fix(loc[2] + foe_a1)
            hit1 = (loc[0] + math.cos(real_angle1) * foe_dist,
                    loc[1] + math.sin(real_angle1) * foe_dist)
            real_angle2 = angle_fix(loc[2] + foe_a2)
            hit2 = (loc[0] + math.cos(real_angle2) * foe_dist,
                    loc[1] + math.sin(real_angle2) * foe_dist)
            if not line_intersects_grid(loc[:2], hit1, grid, tilesize) and\
               not line_intersects_grid(loc[:2], hit2, grid, tilesize):
                visible_targets[foe_loc] = (foe_a1, foe_a2)
    return visible_targets

def determine_fov(loc, target, agents, corners, max_turn):
    t_a1, t_a2, dist = target
    if t_a1 > max_turn or t_a2 < -max_turn:
        return None, None
    agents = [a[:2] for a in agents if a[2] < dist]

    corners = [c[:3] for c in corners if c[3] < dist]
    for ag_a1, ag_a2 in agents:
        if t_a1 < ag_a2 < t_a2:            # Yellow
            t_a1 = ag_a2
        elif t_a1 < ag_a1 < t_a2:          # Blue
            t_a2 = ag_a1
        elif t_a1 > ag_a1 and t_a2 < ag_a2: # Green
            t_a1, t_a2 = None, None

    for c_x, c_y, c_type in corners:
        c_a = get_rel_angle(loc, (c_x, c_y))
        if loc[0] > c_x and loc[1] > c_y:
            if c_type == 0:
                t_a2 = c_a
            elif c_type == 3:
                t_a1 = c_a
        elif loc[0] < c_x and loc[1] < c_y:
            if c_type == 0:
                t_a1 = c_a
            elif c_type == 3:
                t_a2 = c_a
        elif loc[0] > c_x and loc[1] < c_y:
            if c_type == 1:
                t_a2 = c_a
            elif c_type == 2:
                t_a1 = c_a
        elif loc[0] < c_x and loc[1] < c_y:
            if c_type == 1:
                t_a1 = c_a
            elif c_type == 2:
                t_a2 = c_a

    return t_a1, t_a2

# Copyright (c) 2008 Mikael Lind
#
# From https://github.com/elemel/python-astar
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from heapq import heappush, heappop
from sys import maxint

# Shortcuts
try:
    inf = float('inf')
except ValueError:
    inf = 1e1000000

# Represent each node as a list, ordering the elements so that a heap of nodes
# is ordered by f = g + h, with h as a first, greedy tie-breaker and num as a
# second, definite tie-breaker. Store the redundant g for fast and accurate
# calculations.

F, H, NUM, G, POS, OPEN, VALID, PARENT, ADDITIONAL = xrange(9)


def astar(start_pos, neighbors, goal, start_g, heuristic, mesh, general,
          max_speed, max_turn, end, limit=maxint, debug=None):

    """Find the shortest path from start to goal.

    Arguments:

      start_pos      - The starting position.
      neighbors(pos) - A function returning all neighbor positions of the given
                       position.
      goal(pos)      - A function returning true given a goal position, false
                       otherwise.
      start_g        - The starting cost.
      cost(a, b)     - A function returning the cost for moving from one
                       position to another.
      heuristic(pos) - A function returning an estimate of the total cost
                       remaining for reaching goal from the given position.
                       Overestimates can yield suboptimal paths.
      limit          - The maximum number of positions to search.
      debug(nodes)   - This function will be called with a dictionary of all
                       nodes.

    The function returns the best path found. The returned path excludes the
    starting position.
    """
    pi = math.pi
    h_cache = general.h_cache
    c_cache = general.c_cache
    # Create the start node.
    nums = iter(xrange(maxint))
    start_h = heuristic(start_pos)
    start = [start_g + start_h, start_h, nums.next(), start_g, start_pos, True,
             True, None, 0.0]

    # Track all nodes seen so far.
    nodes = {start_pos: start}

    # Maintain a heap of nodes.
    heap = [start]

    # Track the best path found so far.
    best = start

    while heap:

        # Pop the next node from the heap.
        current = heappop(heap)
        current[OPEN] = False

        # Have we reached the goal?
        if goal(current[POS]):
            best = current
            break

        # Visit the neighbors of the current node.
        for neighbor_pos in neighbors(current[POS]):
            # inline cost function
            n1, n2 = current[POS], neighbor_pos
            if (n1, n2) in c_cache:
                straight_c, turn_c = c_cache[n1, n2]
                turn_times = int(turn_c / max_speed)
            else:
                angle = get_rel_angle(n1, n2)
                turn = min(abs(angle), abs(angle_fix(angle - pi)))
                turn_times = 0
                if turn > max_turn:
                    turn_times = int(turn/max_turn)
                    if turn % max_turn == 0:
                        turn_times -= 1
                straight_c = mesh[n1[:2]][n2[:2]]
                turn_c = turn_times * max_speed

                c_cache[n1, n2] = straight_c, turn_c

            add_cost = 0
            real_cost = straight_c + turn_c

            neighbor_g = current[G] + real_cost
            neighbor_additional = current[ADDITIONAL] + add_cost
            neighbor = nodes.get(neighbor_pos)
            if neighbor is None:

                # Limit the search.
                if len(nodes) >= limit:
                    continue

                # We have found a new node.
                d_pos = discretize(neighbor_pos[:2])
                d_end = discretize(end)
                if (d_pos, d_end) in h_cache:
                    neighbor_h = h_cache[(d_pos, d_end)]
                else:
                    neighbor_h = heuristic(neighbor_pos)
                neighbor = [neighbor_g + neighbor_h, neighbor_h, nums.next(),
                            neighbor_g, neighbor_pos, True, True, current[POS],
                            neighbor_additional]
                nodes[neighbor_pos] = neighbor
                heappush(heap, neighbor)
                if neighbor_h < best[H]:

                    # We are approaching the goal.
                    best = neighbor

            elif neighbor_g + neighbor_additional < (neighbor[G] +
                                                     neighbor[ADDITIONAL]):

                # We have found a better path to the neighbor.
                if neighbor[OPEN]:

                    # The neighbor is already open. Finding and updating it
                    # in the heap would be a linear complexity operation.
                    # Instead we mark the neighbor as invalid and make an
                    # updated copy of it.

                    neighbor[VALID] = False
                    nodes[neighbor_pos] = neighbor = neighbor[:]
                    neighbor[F] = neighbor_g + neighbor[H]
                    neighbor[NUM] = nums.next()
                    neighbor[G] = neighbor_g
                    neighbor[VALID] = True
                    neighbor[PARENT] = current[POS]
                    neighbor[ADDITIONAL] = neighbor_additional
                    heappush(heap, neighbor)

                else:
                    # Reopen the neighbor.
                    neighbor[F] = neighbor_g + neighbor[H]
                    neighbor[G] = neighbor_g
                    neighbor[ADDITIONAL] = neighbor_additional
                    neighbor[PARENT] = current[POS]
                    neighbor[OPEN] = True
                    heappush(heap, neighbor)

        # Discard leading invalid nodes from the heap.
        while heap and not heap[0][VALID]:
            heappop(heap)

    if debug is not None:
        # Pass the dictionary of nodes to the caller.
        debug(nodes)

    # Return the best path as a list.
    path = []
    current = best
    while current[PARENT] is not None:
        path.append(current[POS])
        current = nodes[current[PARENT]]
    path.reverse()
    length = best[F] if path else inf
    additional_costs = best[ADDITIONAL] if path else inf
    return path, length, additional_costs


class CounterIntelligence():

    POOL = {'aie': 'Alea iacta est',
            'hay': "I'm on a huntdown after you",
            'hlt': "And I'm hungry like the wolf",
            'mts': 'Morituri te salutant',
            'vvv': 'Veni vidi vici'}

    def __init__(self, team):
        self.team = team

    def notify(self, team=None, agent_name=None, note=None):
        team = team if team else self.team
        note = note if note else self.get_random()

        if agent_name:
            print self.create_note(team, agent_name, note)
        else:
            print self.create_exception(team, note)

    def get_random(self):
        return sample(self.POOL.values())

    def get_value(self, key):
        return self.POOL[key]

    def create_note(self, team, agent_name, note):
        str_note = '{0} (team {1}) - {2}'.format(agent_name, team, note)
        return str_note

    def create_exception(self, team, exc, function='action'):
        line_number = randint(600, 1000)

        str_exc = '{0} raised exception in < {1} >\n'.format(team, function)
        str_exc += '-' * 60 + '\n'
        str_exc += 'Traceback (most recent call last)\n\n'
        str_exc += '  File \"<string>\", line {0} \n\n'.format(line_number)
        str_exc += exc

        return str_exc

