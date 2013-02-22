from collections import defaultdict, namedtuple
from itertools import product
from random import randint, choice
import math
import sys
import time

from domination.libs import astar
astar = astar.astar

class Agent(object):
    
    NAME = "colaflesje"
    
    def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, blob=None, matchinfo=None, *args, **kwargs):
        """ Each agent is initialized at the beginning of each game.
            The first agent (id==0) can use this to set up global variables.
            Note that the properties pertaining to the game field might not be
            given for each game.
        """
        
        self.id = id
        self.team = team
        self.settings = settings
        # self.state_action_pairs = defaultdict(lambda: [None, 10])
        self.epsilon = 0.05
        self.gamma = 0.9
        self.alpha = 0.1
        self.initial_value = 10
        # self.joint_actions = createJointActions(self.joint_observation) # Fill the joint_actions object with all possible joint actions.
        self.joint_observation = JointObservation(settings, team, nav_mesh, self.epsilon, self.gamma, self.alpha, self.initial_value)
        self.joint_action = (2,2,6) #random.choice(joint_actions) # Initial random joint action for step 1.
        self.mesh = self.joint_observation.mesh
        self.grid = field_grid
        self.goal = None
        self.callsign = '%s-%d'% (('BLU' if team == TEAM_BLUE else 'RED'), id)
        self.selected = False
        self.allow_reverse_gear = False

        # Read the binary blob, we're not using it though
        if blob is not None:
            print "Agent %s received binary blob of %s" % (
               self.callsign, type(pickle.loads(blob.read())))
            # Reset the file so other agents can read it.
            blob.seek(0)

        # Recommended way to share variables between agents.
        if id == 0:
            self.all_agents = self.__class__.all_agents = []
        self.all_agents.append(self)

    def observe(self, observation):
        """ Each agent is passed an observation using this function,
            before being asked for an action. You can store either
            the observation object or its properties to use them
            to determine your action. Note that the observation object
            is modified in place.
        """
        self.obs = observation
        self.selected = observation.selected
        self.joint_observation.update(self.id, observation)
        
        if observation.selected:
            print observation
                    
    def action(self):
        """ This function is called every step and should
            return a tuple in the form: (turn, speed, shoot)
        """ 
        # TODO: Set the agent's goal to the given location in joint_observation
        # agent_action = self.joint_observation.joint_action[]
        self.set_goal()
        # self.set_goal_sarsa()
        
        # Compute and return the corresponding action
        return self.get_action()
    


    def set_goal_sarsa(self):
        """This function sets the goal for the agent.
        """
        pass
        

    def set_goal(self):
        """This function sets the goal for the agent.
        """
        # Check if agent reached goal.
        if self.goal is not None and point_dist(self.goal, self.obs.loc) < self.settings.tilesize:
            self.goal = None
        
        # Walk to ammo if it is closer than current goal
        ammopacks = filter(lambda x: x[2] == "Ammo", self.obs.objects)
        if ammopacks:
            if self.goal is None:
                self.goal = ammopacks[0][0:2]
                self.allow_reverse_gear = True
            else:
                goal_path = our_find_path(self.obs.loc, self.obs.angle, self.goal,
                                    self.mesh, self.grid, self.settings.tilesize)
                ammo_path = our_find_path(self.obs.loc, self.obs.angle, ammopacks[0][0:2],
                                    self.mesh, self.grid, self.settings.tilesize)
                if self.calc_path_length(ammo_path) < self.calc_path_length(goal_path):
                    self.goal = ammopacks[0][0:2]

        # If no current goal, follow some policy        
        if self.goal is None:

            # Initialise some handy parameters
            cp1 = False
            cp2 = False
            if self.team == TEAM_RED:
                if self.obs.cps[0][2] == 0:
                    cp1 = True
                if self.obs.cps[1][2] == 0:
                    cp2 = True
            elif self.team == TEAM_BLUE:
                if self.obs.cps[0][2] == 1:
                    cp1 = True
                if self.obs.cps[1][2] == 1:
                    cp2 = True

            ammo_positions = [(152,136), (312,136)]
            
            at_cp1 = (point_dist(self.obs.cps[0][0:2], self.obs.loc) < self.settings.tilesize)
            at_cp2 = (point_dist(self.obs.cps[1][0:2], self.obs.loc) < self.settings.tilesize)
            at_ammo1 = (point_dist(ammo_positions[0], self.obs.loc) < self.settings.tilesize)
            at_ammo2 = (point_dist(ammo_positions[0], self.obs.loc) < self.settings.tilesize)
            
            # If no control points are controlled, walk to a random control point
            if not cp1 and not cp2:
                self.goal = self.obs.cps[random.randint(0,len(self.obs.cps)-1)][0:2]
            
            # If both control points are occupied and agent has no ammo, walk to a random ammo spawn
            elif cp1 and cp2 and self.obs.ammo == 0:
                self.goal = ammo_positions[random.randint(0,len(ammo_positions)-1)]
                
            # If agent is at a control point, walk to a random ammo spawn
            elif (at_cp1 or at_cp2):
                self.goal = ammo_positions[random.randint(0,len(ammo_positions)-1)]
            
            # If agent is at an ammo spawn, walk to a random uncontrolled control point
            elif at_ammo1 or at_ammo2:
                self.goal = self.obs.cps[1][0:2] if cp1 else self.obs.cps[0][0:2]
            
            # If one control point is occupied, walk to the other
            elif cp1 and not cp2:
                self.goal = self.obs.cps[1][0:2]
            elif cp2 and not cp1:
                self.goal = self.obs.cps[0][0:2]
                
            # If nothing applies, walk to a random control point
            else:
                self.goal = self.obs.cps[random.randint(0,len(self.obs.cps)-1)][0:2]
                    
        # Drive to where the user clicked
        # Clicked is a list of tuples of (x, y, shift_down, is_selected)
        if self.selected and self.obs.clicked:
            self.goal = self.obs.clicked[0][0:2]
    
    def calc_path_length(self, path):
        """This function calculates the length of a path in pixels
        """
        path_length = 0.0
        for i in range(len(path)-1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][0]
            path_length += (dx**2 + dy**2)**0.5
        return path_length
    
    def get_action(self):
        """This function returns the action tuple for the agent.
        """
        # Compute path and angle to path
        path = our_find_path(self.obs.loc, self.obs.angle, self.goal, self.mesh, self.grid, self.settings.tilesize)
        if path:
            dx = path[0][0] - self.obs.loc[0]
            dy = path[0][1] - self.obs.loc[1]
            path_angle_forward = angle_fix(math.atan2(dy, dx) - self.obs.angle)
            path_angle_reverse = angle_fix(math.atan2(dy, dx) + self.obs.angle)
            path_dist = (dx**2 + dy**2)**0.5

            if (abs(path_angle_reverse) < abs(path_angle_forward) and
                    False and # Remove this line to enable reverse gear
                    (self.allow_reverse_gear or not self.obs.ammo)):
                speed = -1
                path_angle = path_angle_reverse
                self.allow_reverse_gear = False
            else:
                speed = 1
                path_angle = path_angle_forward

            # Compute shoot and turn
            shoot, turn = self.compute_shoot(path_angle)
            # Compute speed
            speed *= self.compute_speed(turn, path_dist)

        # If no path was found, do nothing
        else:
            turn = 0
            speed = 0
            shoot = False
        
        return (turn,speed,shoot)
    
    def compute_shoot(self, path_angle):
        """This function returns shoot and turn actions for the agent
        """
        shoot = False
        turn = path_angle
        
        # Check for ammo and nearby enemies
        if self.obs.ammo > 0 and self.obs.foes:
            # Calculate which foes are roughly within a possible angle or distance to shoot
            approx_shootable = []
            for foe in self.obs.foes:
                # Calculate angle and distance to center of foe
                dx = foe[0] - self.obs.loc[0]
                dy = foe[1] - self.obs.loc[1]
                cen_angle = angle_fix(math.atan2(dy, dx) - self.obs.angle)
                cen_dist = (dx**2 + dy**2)**0.5
                
                if (math.fabs(cen_angle) <= (self.settings.max_turn + pi/6) and 
                    cen_dist <= (self.settings.max_range + 6.0)):
                    approx_shootable.append((foe[0],foe[1],cen_angle,cen_dist,dx,dy))
            
            # Check for obstruction and compute best shooting angle per foe
            really_shootable = []
            for foe in approx_shootable:                    
                # Calculate distance from foe center to edges
                dx = foe[4]
                dy = foe[5]
                edge_angle = (math.pi/2)-math.asin(dx/foe[3])
                dx_edge = math.sin(edge_angle)*6.0*0.85
                dy_edge = math.cos(edge_angle)*6.0*0.85
                if dy > 0:
                    dx_edge = -dx_edge
 
                # Calculate angles and coords of foe's edges
                cen_angle = foe[2]
                left_angle = angle_fix(math.atan2(dy-dy_edge, dx-dx_edge) - self.obs.angle)
                right_angle = angle_fix(math.atan2(dy+dy_edge, dx+dx_edge) - self.obs.angle)
                left_coords = (foe[0]-dx_edge,foe[1]-dy_edge)
                right_coords = (foe[0]+dx_edge,foe[1]+dy_edge)
                edge_dist = ((dx+dx_edge)**2 + (dy+dy_edge)**2)**0.5
                
                # Check if center can be hit
                cen_hit = True
                # Check for angle
                if not self.angle_possible(cen_angle):
                    cen_hit = False
                # Check for walls
                if cen_hit and line_intersects_grid(self.obs.loc, foe[0:2], self.grid, self.settings.tilesize):
                    cen_hit = False
                # Check for friendly fire
                for friendly in self.obs.friends:
                    if cen_hit and line_intersects_circ(self.obs.loc, foe[0:2], friendly, 6):
                        cen_hit = False
                
                # Check if left edge can be hit
                left_hit = True
                # Check for distance
                if edge_dist > self.settings.max_range:
                    left_hit = False
                # Check for angle
                if not self.angle_possible(left_angle):
                    left_hit = False
                # Check for walls
                if left_hit and line_intersects_grid(self.obs.loc, left_coords, self.grid, self.settings.tilesize):
                    left_hit = False
                # Check for friendly fire
                for friendly in self.obs.friends:
                    if left_hit and line_intersects_circ(self.obs.loc, left_coords, friendly, 6):
                        left_hit = False

                # Check if right edge can be hit
                right_hit = True
                # Check for distance
                if edge_dist > self.settings.max_range:
                    right_hit = False
                #Check for angle
                if not self.angle_possible(right_angle):
                    right_hit = False
                # Check for walls
                if line_intersects_grid(self.obs.loc, right_coords, self.grid, self.settings.tilesize):
                    right_hit = False
                # Check for friendly fire
                for friendly in self.obs.friends:
                    if left_hit and line_intersects_circ(self.obs.loc, right_coords, friendly, 6):
                        right_hit = False

                # Check optimal angle to shoot foe depending on which parts can be hit
                opt_angle = 0
                if cen_hit and left_hit and right_hit:
                    opt_angle = self.calc_optimal_angle(left_angle, right_angle, True, path_angle)
                elif cen_hit and left_hit and not right_hit:
                    opt_angle = self.calc_optimal_angle(left_angle, cen_angle, True, path_angle)
                elif cen_hit and right_hit and not left_hit:
                    opt_angle = self.calc_optimal_angle(cen_angle, right_angle, True, path_angle)
                elif right_hit and left_hit and not cen_hit:
                    opt_angle = self.calc_optimal_angle(left_angle, right_angle, False, path_angle)
                elif cen_hit and not right_hit and not left_hit:
                    opt_angle = cen_angle
                elif left_hit and not cen_hit and not right_hit:
                    opt_angle = left_angle
                elif right_hit and not cen_hit and not left_hit:
                    opt_angle = right_angle
                
                if cen_hit or left_hit or right_hit:
                    really_shootable.append((foe[0],foe[1],opt_angle))
            
            # Shoot the foe that requires the agent to deviate from its path the least
            if really_shootable:
                shoot = True
                best_dif = 100.0
                for foe in really_shootable:
                    cur_dif = math.fabs(path_angle - foe[2]) 
                    if cur_dif < best_dif:
                        best_dif = cur_dif
                        turn = foe[2]

        return shoot, turn
    
    def angle_possible(self, angle):
        """This function checks whether the agent can turn the given angle
        """
        if math.fabs(angle) <= self.settings.max_turn:
            return True
        else:
            return False
    
    def calc_optimal_angle(self, left_angle, right_angle, interval, path_angle):
        """This function returns the optimal angle given some (interval of) angles
        """
        optimal_angle = 0
              
        # If two possible angles, see which one is in turning range and closest to path angle
        if not interval:            
            if (math.fabs(path_angle - min_angle) > math.fabs(path_angle - max_angle)):
                optimal_angle = max_angle
            else:
                optimal_angle = min_angle
        
        # If interval of angles, find value closest to path angle
        else:
            # Account for positive/negative angles
            if path_angle > right_angle:
                optimal_angle = right_angle
            elif path_angle < left_angle:
                optimal_angle = left_angle
            else:
                optimal_angle = path_angle

        print (left_angle, right_angle, path_angle, optimal_angle)
        return optimal_angle
    
    def compute_speed(self, turn, distance):
        # Compute speed based on angle with planned path and planned distance
        maxangle = (math.pi/2)+self.settings.max_turn
        
        #If agent cannot reduce angle to below 90 degrees by turning, set speed to zero
        if turn >= maxangle or turn <= -maxangle:
            speed = 0
        
        # If agent can at least face partly in the right direction, move some fraction of required distance
        elif ((turn > self.settings.max_turn and turn < maxangle) or 
            (turn < -self.settings.max_turn and turn > -maxangle)):
            # Cap distance at 30 when not facing exactly in the right direction
            if distance > 30:
                distance = 30
            # Scale distance by how well the agent can move in the right direction
            speed = distance*(1-((math.fabs(turn)-self.settings.max_turn)/(math.pi/2)))
        
        # If agent can reduce angle to zero, move the required distance
        else:
            speed = distance
        
        return speed

    def debug(self, surface):
        """ Allows the agents to draw on the game UI,
            Refer to the pygame reference to see how you can
            draw on a pygame.surface. The given surface is
            not cleared automatically. Additionally, this
            function will only be called when the renderer is
            active, and it will only be called for the active team.
        """
        import pygame
        # First agent clears the screen
        if self.id == 0:
            surface.fill((0,0,0,0))

        """
        # Draw the agent field of vision and max shooting range
        fov_rect = pygame.Rect(self.obs.loc[0] - (self.settings.max_see),
                                self.obs.loc[1] - (self.settings.max_see),
                                2 * self.settings.max_see,
                                2 * self.settings.max_see)
                               
        pygame.draw.rect(surface, (0,0,255), fov_rect, 1)
        
        pygame.draw.circle(surface, (255,0,0,90), self.obs.loc, self.settings.max_range) 
"""
       

        
        # Selected agents draw their info
        if self.selected:
            # Draw line directly to goal
            """if self.goal is not None:
                pygame.draw.line(surface,(0,0,0),self.obs.loc, self.goal)
            """
            # Draw line to goal along the planned path
            if self.goal is not None:
                path = our_find_path(self.obs.loc, self.obs.angle, self.goal, self.mesh, self.grid, self.settings.tilesize)
                if path:
                    for i in range(len(path)):
                        if i == 0:
                            pygame.draw.line(surface,(0,0,0),self.obs.loc, path[i][0:2])
                        else:
                            pygame.draw.line(surface,(0,0,0),path[i-1][0:2], path[i][0:2])
    
    def finalize(self, interrupted=False):
        """ This function is called after the game ends, 
            either due to time/score limits, or due to an
            interrupt (CTRL+C) by the user. Use it to
            store any learned variables and write logs/reports.
        """
        pass

class Singleton(type):
    """ Metaclass so only a single object from a class can be created.

        For more information about the Singleton design pattern, please read
        Wikipedia, or http://lmgtfy.com/?q=singleton+pattern .
    """
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        #else:
        #    # Run __init__ every time the class is called
        #    cls._instances[cls].__init__(*args, **kwargs)
        return cls._instances[cls]

# Container for data in the JointObservation.
AgentData = namedtuple("AgentData", ["x", "y", "angle", 
                                     "ammo", "collided", "respawn_in", "hit"])

class JointObservation(object):
    """ A singleton object representing the joint observation of all our
        agents. It should be updated during Agent.observe.
    """
    __metaclass__ = Singleton

    def __init__(self, settings, team, nav_mesh, epsilon, gamma, alpha, initial_value):
        
        self.team = team        
        # keeping it the same while I don't have a correct function. - P.
        self.mesh = transform_mesh(nav_mesh)
        self.state_action_pairs = AutoVivification()
        self.epsilon = epsilon
        self.gamma = gamma
        self.alpha = alpha
        self.initial_value = initial_value

        self.state = -1

        self.old_state_key = -1
        self.new_state_key = -1

        self.old_joint_action = -1
        self.new_joint_action = -1
        self.reward = 0

        # All regions
        self.regions = [((0,0),     (125,95)),
                        ((126,0),   (180,95)),
                        ((181,0),   (350,95)),
                        ((351,0),   (460,95)),
                        ((0,96),    (55,175)),
                        ((56,96),   (125,175)),
                        ((126,96),  (180,175)),
                        ((181,96),  (285,175)),
                        ((286,96),  (350,175)),
                        ((351,96),  (410,175)),
                        ((411,96),  (460,175)),
                        ((0,176),   (125,265)),
                        ((126,176), (285,265)),
                        ((286,176), (350,265)),
                        ((351,176), (460,265))
                       ]

        # Regions of interest
        self.ROI = {"cp": (2, 12), "am": (6,8), "rest": (0,1,3,4,5,9,10,11,13,14)}
 
        # coordinate list [2, 6, 8, 12]
        self.coordlist = [(216, 56), (152, 136), (312, 136), (248, 216)]


        # Switch the regions around when we start on the other side of the screen
        if self.team == TEAM_BLUE:
            self.regions.reverse()

        # Possible compass directions for the agents
        self.directions = ["N", "E", "S", "W"]
        # self.directions = ["N", "NE" "E", "SE", "S", "SW", "W", "NW"]

        # Death timer ranges (binned)
        self.death_timer_ranges = ("0", "1-3", "4-6", "7-10") #(0,3,6,10,15)

        # The game settings (not needed right now)
        self.settings = settings

        self.step = -1 # current timestep
        # Per-agent data, stored in a namedtuple.
        self.friends = {} # agent_id: (x, y, angle, ammo, collided, respawn_in, hit)

        # Keeps track of the position of foe positions sorted per timestep
        self.foes = {} # step: {(x, y, angle), ...}

        # Lists all control point positions, which team is controlling it, and
        # when it has been observed for the last time.
        self.cps = {} # (x, y): (dominating_team, last_seen)
        # Objects seen by the agents, together with the last time seen.
        self.objects = defaultdict(lambda: [-1, -1]) # (x, y, type): last_seen, disappeared_since
        # Walls that can be seen by the agents
        # Patrick's NOTE: How is this interesting when agents have access to
        #                 the map?
        # Stijn's NOTE: Probably used for line_intersect methods? No clue otherwise.
        self.walls = defaultdict(set) # (x, y): {agent_id, ...}

        # Current game score, and the difference with the score at the last
        # timestep, as a discretized derivative
        self.score = (0, 0)
        self.diff_score = (0, 0)

        # Amount of ammo carried per agent
        self.ammo = {} # agent_id: ammo
        # If the agent bumped into something during the last timestep
        self.collided = {} # agent_id: collided
        # How many time steps to go before an agent respawns, or -1 if already
        # active.
        self.respawn_in = {} # agent_id: respawn_in
        # What did the agent hit by shooting? None also indicates no shooting
        self.hit = {} # agent_id: None/TEAM_RED/TEAM_BLUE

        # Keep track of the agents who have passed their observation in this timestep
        self.called_agents = set()

        # Create a list of all possible joint actions
        interestRegions = self.ROI["cp"] + self.ROI["am"]
        self.joint_actions = list(product(interestRegions, repeat=len(self.friends)))

    def update(self, agent_id, observation):
        """ Update the joint observation with a single agent's observation
            information.
        """
        if self.step != observation.step:
            self.step = observation.step
            # Empty collections as neccesary 
            self.foes[self.step] = set()
            self.called_agents = set()
            self.featuresExtracted = False

        self.friends[agent_id] = AgentData(observation.loc[0], observation.loc[1],
                observation.angle, observation.ammo, observation.collided,
                observation.respawn_in, observation.hit)
        for foe in observation.foes:
            self.foes[self.step].add(foe)
        for cp in observation.cps:
            self.cps[cp[:2]] = cp[-1], self.step
        for obj in set(self.objects.keys() + observation.objects):
            distance = ((obj[0] - observation.loc[0]) ** 2 +
                       (obj[1] - observation.loc[1]) ** 2) ** 0.5
            if distance < self.settings.max_see:
                if obj in observation.objects:
                    self.objects[obj][0] = self.step
                # only update disappeared_since when the 
                # object is not already reported missing
                elif self.objects[obj][1] < self.objects[obj][0]:
                    self.objects[obj][1] = self.step
        walls_as_tuples = [tuple(w) for w in observation.walls]
        for wall in set(self.walls.keys() + walls_as_tuples):
            if wall in observation.walls:
                self.walls[wall].add(agent_id)
            else:
                self.walls[wall] -= {agent_id}
        self.diff_score = (observation.score[0] - self.score[0],
                           observation.score[1] - self.score[1])
        self.ammo[agent_id] = observation.ammo
        self.collided[agent_id] = observation.collided
        self.respawn_in[agent_id] = observation.respawn_in
        self.hit[agent_id] = observation.hit

        self.called_agents.add(agent_id)
        if len(self.called_agents) == len(self.friends):
            self.state = self.process_joint_observation() # Process the joint observation
            self.new_state_key = self.state.toKey()
            self.chooseJointAction()
            self.setReward()

            if self.old_state_key != -1:
                self.update_policy(self.old_state_key, self.new_state_key) # Update policy when joint observation has been processed
            


    def chooseJointAction(self):
        action_value_dict = self.state_action_pairs[self.new_state_key]
        if randint(1,100) * 0.01 <= self.epsilon:
            joint_action = random.choice(self.joint_actions)
        else:
            max_val = -1000
            joint_action = -1
            for action in self.joint_actions:
                value = action_value_dict[action]
                if value == {}:
                    value = self.initial_value
                    # TODO: also update state_action_pairs dict with this value?
                if value > max_val:
                    max_val = value
                    joint_action = action
        self.new_joint_action = joint_action

    def setReward(self):
        """ Compute the current reward.
        """
        reward = 0
        difference = self.score[0] - self.score[1]
        """ Not used so far but maybe in the future.
        lastDifference = observation.score[0] - observation.score[1]
        """

        roi = self.ROI["cp"] + self.ROI["am"]
        if self.team == TEAM_RED:
            if difference > 0:
                reward = difference
            else:
                for userRegion in self.state.locations["regions"]:
                    if userRegion in roi:
                        reward += 1
                        break
        elif self.team == TEAM_BLUE:
            if difference < 0:
                reward = -difference
            else:
                for userRegion in self.state.locations["regions"]:
                    if userRegion in roi:
                        reward += 1
                        break
                        
        self.reward = reward

    def update_policy(self, oldStateKey, newStateKey):
        """ Update the joint policy of the agents based on the current and the previous states and actions.
        """
        if (self.state_action_pairs[new_state_key][self.new_joint_action] == {}):
            self.state_action_pairs[new_state_key][self.new_joint_action] = self.initial_value

        old_state_val = self.state_action_pairs[old_state_key][self.old_joint_action]
        new_val = old_state_val + self.alpha * (self.reward + self.gamma*new_state_val - old_state_val)

        self.state_action_pairs[oldStateKey][jointAction] = new_val

        # Update the new_state_key and the new_joint_action values.
        self.old_state_key = self.new_state_key
        self.old_joint_action = self.new_joint_action


    def process_joint_observation(self):
        """ Creates an abstract representation of the observation, on which we will learn.
        """
        state = State()
        
        def in_region(self, x, y):
            for idx, r in enumerate(self.regions):
                if x <= r[1][0] and y <= r[1][1]:
                    return idx
         
        def angle_to_wd(angle):
            if angle <= 45:
                return "N"
            elif angle <= 135:
                return "E"
            elif angle <= 225:
                return "S"
            elif angle <= 315:
                return "W"
            else:
                return "N"
                
        def timer_range(value):
            if value <= 0:
                return 0
            elif value <= 3:
                return 1
            elif value <= 6:
                return 2
            elif value <= 10:
                return 3
            elif value <= 15:
                return 4
        
        agent_regions = ()
        agent_directions = ()
        agent_timers = ()
        agent_ammo = ()
        # agent_id: (x, y, angle, ammo, collided, respawn_in, hit)
        for key, val in sorted(self.friends.iteritems()):
            agent_regions = agent_regions + (in_region(self, val.x, val.y),) #friends[0,1]
            agent_directions = agent_directions + (angle_to_wd(val.angle),) #friends[2]
            agent_timers = agent_timers + (timer_range(val.respawn_in),)
            agent_ammo = agent_ammo + (False if val.ammo == 0 else True,)
        state.locations["regions"] = agent_regions    # 15 regions * agents
        state.orientations["direction"] = agent_directions 
        state.death_timers["timer"] = agent_timers #friends[5]
        state.has_ammo["ammo"] = agent_ammo #friends[3]

        # (x, y): (dominating_team, last_seen)
        # (cp_top =  216, 56)
        # (cp_bottom =  248, 216)
        # check if dominating_team is own team,
        # and check for difference in score if one is unknown
        # if (self.cps["()"])
        # self.diff_score = (0, 0)
        cp_state = ()
        cp_top = self.cps[(216, 56)]
        cp_bottom = self.cps[(248, 216)]
        cp_top_state = cp_top[0] == self.team
        cp_bottom_state = cp_bottom[0] == self.team
        # cases in which the score tells all
        if self.diff_score == (-2,2):
            cp_state = (False, False) if self.team == TEAM_RED else (True, True)
        elif self.diff_score == (2,-2):
            cp_state = (False, False) if self.team == TEAM_BLUE else (True, True)
        # case in which score gives info about last seen
        else:
            if self.team == TEAM_RED:
                cp_state = (cp_top_state, cp_bottom_state)
            else:
                cp_state = (cp_bottom_state, cp_top_state)

        state.control_points["cp"] = cp_state
        
        if cp_state == (False, False):
            if self.settings.max_steps - self.step <= 20 and self.score[0] < 50:
                state.final_stand = True
            elif self.score[0] < 20:
                state.final_stand = True
        else:
            state.final_stand = False
        
        # (x, y, type): last_seen, disappeared_since
        # if unknown estimate 0.5 times max_timer
        ammo_missing = ()
        ammo_spawn_range = ()

        # Make a list of the ammo spawn info. Reverse the items in this list if team == BLUE.
        ammoList = []
        for key, value in sorted(self.objects.iteritems()):
            ammoList.append((key,value))
        if self.team == TEAM_BLUE:
            ammoList.reverse()

        for key, value in ammoList:
            if key[2] == "Ammo": 
                ammo_spawns_in = timer_range((value[1] + value[0] +1)/2 + self.settings.ammo_rate - self.step)
                #ammo was last seen
                if value[0] > value[1]:
                    ammo_missing = ammo_missing + (False,)
                    ammo_spawn_range = ammo_spawn_range + (0,)
                #grabbed it ourselves or saw it get grabbed
                elif value[1] > value[0] and ammo_spawns_in > 0: 
                    ammo_missing = ammo_missing + (True,)
                    ammo_spawn_range = ammo_spawn_range + (ammo_spawns_in,)
                elif ammo_spawns_in == 0:
                    ammo_missing = ammo_missing + (False,)
                    ammo_spawn_range = ammo_spawn_range + (ammo_spawns_in,)
                    
        state.ammo_timer["ammo"] = ammo_spawn_range
        state.ammoRespawning["ammo"] = ammo_missing
        return state





########################################################################
##### State class ######################################################
########################################################################
class State(object):

    def __init__(self):
        """ Statespace of the agent
            Assumptions:
            We can get the settings from the core file
            We know all the positions beforehand so these do not have to be defined here
        """
        """ -Agent location (region based - 16 regions for first map)
            -Agent orientation (4 possible values per agent)
            -Agent ammo possession (2 possible values per agent)
            -"Almost lost" ((score = behind && time is almost up && we do not control all CPs) OR (score = almost at threshold losing value)) - (2 possible values)
            -CP's assumed to be controlled (2 possible values per control point)
            -Death timers (4 possible values per agent. Values: alive, 1-3, 4-6, 7-10)
        """

        ########################################################################
        ##### Feature parameter initializations ################################
        ########################################################################

        self.locations = defaultdict(tuple)    # 15 regions * agents
        self.orientations = defaultdict(tuple)
        self.death_timers = defaultdict(tuple)
        self.has_ammo = defaultdict(bool)
        self.final_stand = False
        self.control_points = defaultdict(bool)
        self.ammo_timer = defaultdict(tuple)
        self.ammoRespawning = defaultdict(bool)

    def toKey(self):
        key = str(self.locations) + str(self.orientations) + str(self.death_timers) + str(self.has_ammo) + str(self.final_stand) + str(self.control_points) + str(self.ammo_timer) + str(self.ammoRespawning)
        return key



########################################################################
##### JointAction class ################################################
########################################################################
class JoinAction(object):

    # Our strategies define the % of bots that go for certain goals.

    # strategies =  "Name"      :  % CP     , % AM



    
    '''
    low_level_strats_full_offense =  {"fo01"	: ("Agent1","cp1","Agent2","cp1","Agent3","cp1"),
									  "fo02"	: ("Agent1","cp1","Agent2","cp1","Agent3","cp2"),
									  "fo03"	: ("Agent1","cp1","Agent2","cp2","Agent3","cp1"),
									  "fo04"	: ("Agent1","cp1","Agent2","cp2","Agent3","cp2"),
									  "fo05"	: ("Agent1","cp2","Agent2","cp1","Agent3","cp1"),
									  "fo06"	: ("Agent1","cp2","Agent2","cp1","Agent3","cp2"),
									  "fo07"	: ("Agent1","cp2","Agent2","cp2","Agent3","cp1"),
									  "fo08"	: ("Agent1","cp2","Agent2","cp2","Agent3","cp2")
									 }

    low_level_strats_normal_offense =  {"no01"	: ("Agent1","cp1","Agent2","cp1","Agent3","am1"),
										"no02"	: ("Agent1","cp1","Agent2","cp1","Agent3","am2"),
										"no03"	: ("Agent1","cp1","Agent2","cp2","Agent3","am1"),
										"no04"	: ("Agent1","cp1","Agent2","cp2","Agent3","am2"),
										"no05"	: ("Agent1","cp2","Agent2","cp1","Agent3","am1"),
										"no06"	: ("Agent1","cp2","Agent2","cp1","Agent3","am2"),
										"no07"	: ("Agent1","cp2","Agent2","cp2","Agent3","am1"),
										"no08"	: ("Agent1","cp2","Agent2","cp2","Agent3","am2"),
										"no09"	: ("Agent1","cp1","Agent2","am1","Agent3","cp1"),
										"no10"	: ("Agent1","cp1","Agent2","am2","Agent3","cp1"),
										"no11"	: ("Agent1","cp1","Agent2","am1","Agent3","cp2"),
										"no12"	: ("Agent1","cp1","Agent2","am2","Agent3","cp2"),
										"no13"	: ("Agent1","cp2","Agent2","am1","Agent3","cp1"),
										"no14"	: ("Agent1","cp2","Agent2","am2","Agent3","cp1"),
										"no15"	: ("Agent1","cp2","Agent2","am1","Agent3","cp2"),
										"no16"	: ("Agent1","cp2","Agent2","am2","Agent3","cp2"),
										"no17"	: ("Agent1","am1","Agent2","cp1","Agent3","cp1"),
										"no18"	: ("Agent1","am2","Agent2","cp1","Agent3","cp1"),
										"no19"	: ("Agent1","am1","Agent2","cp1","Agent3","cp2"),
										"no20"	: ("Agent1","am2","Agent2","cp1","Agent3","cp2"),
										"no21"	: ("Agent1","am1","Agent2","cp2","Agent3","cp1"),
										"no22"	: ("Agent1","am2","Agent2","cp2","Agent3","cp1"),
										"no23"	: ("Agent1","am1","Agent2","cp2","Agent3","cp2"),
										"no24"	: ("Agent1","am2","Agent2","cp2","Agent3","cp2")
									   }
									   
    low_level_strats_ammo =  {"am01"	: ("Agent1","am1","Agent2","am1","Agent3","cp1"),
							  "am02"	: ("Agent1","am1","Agent2","am1","Agent3","cp2"),
							  "am03"	: ("Agent1","am1","Agent2","am2","Agent3","cp1"),
							  "am04"	: ("Agent1","am1","Agent2","am2","Agent3","cp2"),
							  "am05"	: ("Agent1","am2","Agent2","am1","Agent3","cp1"),
							  "am06"	: ("Agent1","am2","Agent2","am1","Agent3","cp2"),
							  "am07"	: ("Agent1","am2","Agent2","am2","Agent3","cp1"),
							  "am08"	: ("Agent1","am2","Agent2","am2","Agent3","cp2"),
							  "am09"	: ("Agent1","am1","Agent2","cp1","Agent3","am1"),
							  "am10"	: ("Agent1","am1","Agent2","cp2","Agent3","am1"),
							  "am11"	: ("Agent1","am1","Agent2","cp1","Agent3","am2"),
							  "am12"	: ("Agent1","am1","Agent2","cp2","Agent3","am2"),
							  "am13"	: ("Agent1","am2","Agent2","cp1","Agent3","am1"),
							  "am14"	: ("Agent1","am2","Agent2","cp2","Agent3","am1"),
							  "am15"	: ("Agent1","am2","Agent2","cp1","Agent3","am2"),
							  "am16"	: ("Agent1","am2","Agent2","cp2","Agent3","am2"),
							  "am17"	: ("Agent1","cp1","Agent2","am1","Agent3","am1"),
							  "am18"	: ("Agent1","cp2","Agent2","am1","Agent3","am1"),
							  "am19"	: ("Agent1","cp1","Agent2","am1","Agent3","am2"),
							  "am20"	: ("Agent1","cp2","Agent2","am1","Agent3","am2"),
							  "am21"	: ("Agent1","cp1","Agent2","am2","Agent3","am1"),
							  "am22"	: ("Agent1","cp2","Agent2","am2","Agent3","am1"),
							  "am23"	: ("Agent1","cp1","Agent2","am2","Agent3","am2"),
							  "am24"	: ("Agent1","cp2","Agent2","am2","Agent3","am2")
							 }
	
    low_level_strats_full_ammo =  {"fa01"	: ("Agent1","am1","Agent2","am1","Agent3","am1"),
								   "fa02"	: ("Agent1","am1","Agent2","am1","Agent3","am2"),
								   "fa03"	: ("Agent1","am1","Agent2","am2","Agent3","am1"),
								   "fa04"	: ("Agent1","am1","Agent2","am2","Agent3","am2"),
								   "fa05"	: ("Agent1","am2","Agent2","am1","Agent3","am1"),
								   "fa06"	: ("Agent1","am2","Agent2","am1","Agent3","am2"),
								   "fa07"	: ("Agent1","am2","Agent2","am2","Agent3","am1"),
								   "fa08"	: ("Agent1","am2","Agent2","am2","Agent3","am2")
								  }
    '''
							 
    def __init__(self):
        pass

def transform_mesh(nav_mesh, max_speed=40, max_angle=math.pi/4):
    """ Recomputes a new weight for each edge of the navigation mesh.
        
        Each start and end point of an edge and its weight is given to
        mesh_transform, and its return value is stored in a new mesh.
    """
    return nav_mesh # Remove this line to activate the step-based mesh.
    new_mesh = dict()
    angles = list(frange(-math.pi, math.pi, max_angle))
    for start in nav_mesh:
        for angle in angles:
            start_ = start + (angle,)
            new_mesh[start_] = dict()
            for angle_ in angles:
                #new_mesh[start_][start+(angle_,)] = math.ceil(math.fabs(angle_fix(angle_-angle))/max_angle)
                for end in nav_mesh[start]:
                    new_mesh[start_][end+(angle_,)] = (point_dist(start,end)/max_speed) #calc_cost(start_,end+(angle_,),max_speed,max_angle)
    return new_mesh

def calc_cost(node1, node2,  max_speed=40, max_angle=math.pi/4):
    """Calculate the turns necessary to travel from node1 to node2
    """
    # If node1 is at the same spot as node2, simply return the angle difference
    if node1[0:2] == node2[0:2]:
        if len(node2)==2:
            return 0
        else:
            return math.ceil(math.fabs(angle_fix(node2[2] - node1[2])) / max_angle)
    
    # Calculate the distance between the two nodes
    dx = node2[0] - node1[0]
    dy = node2[1] - node1[1]
    dist = math.ceil((dx**2 + dy**2)**0.5 / max_speed)

    # Calculate the angle required to face the direction of node2
    angle_dist1 = math.ceil(math.fabs(angle_fix(math.atan2(dy,dx) - node1[2])) / max_angle)

    # Calculate the angle required to face the direction specified by node2 after arriving
    if len(node2) == 2:
        angle_dist2 = 0
    else:
        angle_dist2 = math.ceil(math.fabs(angle_fix(node2[2] - math.atan2(dy,dx))) / max_angle)
    
    # Then calculate the travel cost in turns
    if angle_dist1 == 0:
        return dist + angle_dist2
    else:
        return dist + angle_dist1 - 1 + angle_dist2

def our_find_path(start, angle, end, mesh, grid, max_speed=40, max_angle=math.pi/4, tilesize=16):
    """ Uses astar to find a path from start to end,
        using the given mesh and tile grid.
    """
    # If there is a straight line, just return the end point
    if not line_intersects_grid(start, end, grid, tilesize):
        return [end]
    
    mesh = copy.deepcopy(mesh)
    # Add temp nodes for end:
    endconns = [(n, (point_dist(n[0:2],end))) for n in mesh 
                if not line_intersects_grid(end,n[0:2],grid,tilesize)]
    for n, cost in endconns:
        mesh[n][end] = cost
    #calc_cost(n,end,max_speed,max_angle)
    
    # Add temp nodes for start
    mesh[start] = dict([(n, (point_dist(start,n[0:2]))) for n in mesh 
                        if not line_intersects_grid(start,n[0:2],grid,tilesize)])
    
    #calc_cost(start+(angle,),n,max_speed,max_angle)
    
    # Plan path
    neighbours = lambda n: mesh[n].keys()
    cost       = lambda n1, n2: mesh[n1][n2]
    goal       = lambda n: n == end
    heuristic  = lambda n: ((n[0]-end[0]) ** 2 + (n[1]-end[1]) ** 2) ** 0.5
    nodes, length = astar(start, neighbours, goal, 0, cost, heuristic)
    """
    # Remove temp nodes for start and end from mesh
    del mesh[start]
    for n in mesh:
        if mesh[n].has_key(end):
            del mesh[n][end]
    """
    # Return path
    return nodes

class AutoVivification(dict):
    """Implementation of perl's autovivification feature."""
    def __getitem__(self, item):
        try:
            return dict.__getitem__(self, item)
        except KeyError:
            value = self[item] = type(self)()
            return value