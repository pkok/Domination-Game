from collections import defaultdict, namedtuple

class Agent(object):
    
    NAME = "stuiterbal"
    
    def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, blob=None, matchinfo=None, *args, **kwargs):
        """ Each agent is initialized at the beginning of each game.
            The first agent (id==0) can use this to set up global variables.
            Note that the properties pertaining to the game field might not be
            given for each game.
        """
        self.id = id
        self.team = team
        self.mesh = nav_mesh
        self.grid = field_grid
        self.settings = settings
        self.goal = None
        self.callsign = '%s-%d'% (('BLU' if team == TEAM_BLUE else 'RED'), id)
        self.selected = False
        self.joint_observation = JointObservation(settings)
        self.state_strategy_pairs = defaultdict(lambda: [None, None, 0])

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
        self.joint_observation.update(self.id, observation)

        self.observation = observation
        self.selected = observation.selected

        if observation.selected:
            print observation
                    
    def action(self):
        """ This function is called every step and should
            return a tuple in the form: (turn, speed, shoot)
        """ 
        # Set the goal of the action
        self.set_goal(self.observation)
        
        # Compute and return the corresponding action
        return self.get_action(self.observation)
    
    def set_goal(self, obs):
        """This function sets the goal for the agent.
           Nobert, Thomas, Fernando: this is your turf!
        """
        # Check if agent reached goal.
        if self.goal is not None and point_dist(self.goal, obs.loc) < self.settings.tilesize:
            self.goal = None
            
        # Walk to ammo
        ammopacks = filter(lambda x: x[2] == "Ammo", obs.objects)
        if ammopacks:
            self.goal = ammopacks[0][0:2]
            
        # Drive to where the user clicked
        # Clicked is a list of tuples of (x, y, shift_down, is_selected)
        if self.selected and self.observation.clicked:
            self.goal = self.observation.clicked[0][0:2]
        
        # Walk to random CP
        if self.goal is None:
            self.goal = obs.cps[random.randint(0,len(obs.cps)-1)][0:2]
    
    def get_action(self, obs):
        """This function returns the action for the agent.
           Patrick, Stijn: this is your turf!
        """
        # Compute path and angle to path
        path = find_path(obs.loc, self.goal, self.mesh, self.grid, self.settings.tilesize)
        if path:
            dx = path[0][0] - obs.loc[0]
            dy = path[0][1] - obs.loc[1]
            path_angle = angle_fix(math.atan2(dy, dx) - obs.angle)

        # Compute shoot and angle to enemies
        shoot = False
        if (obs.ammo > 0 and 
            obs.foes and 
            point_dist(obs.foes[0][0:2], obs.loc) < self.settings.max_range and
            not line_intersects_grid(obs.loc, obs.foes[0][0:2], self.grid, self.settings.tilesize)):
            
            # Calculate angle and distance to enemy center
            dxf = obs.foes[0][0] - obs.loc[0]
            dyf = obs.foes[0][1] - obs.loc[1]
            cen_angle = angle_fix(math.atan2(dyf, dxf) - obs.angle)
            cen_dist = (dxf**2 + dyf**2)**0.5
            
            # Calculate angle between enemy center and edge
            in_angle = math.asin(6.0 / cen_dist)
            
            # Calculate minimum angle the agent should turn to hit enemy
            if cen_angle > 0:
                if cen_angle >= in_angle:
                    req_turn = cen_angle - in_angle
                else:
                    req_turn = 0 #ACTUALLY, CALCULATE ANGLE TO GOAL AND TURN AS FAR AS POSSIBLE
            else:
                if cen_angle <= -in_angle:
                    req_turn = cen_angle + in_angle
                else:
                    req_turn = 0 #ACTUALLY, CALCULATE ANGLE TO GOAL AND TURN AS FAR AS POSSIBLE
                            
            if math.fabs(req_turn) <= self.settings.max_turn:
                shoot = True
                turn = req_turn
            
            #Check for friendly fire
            for friendly in obs.friends:
                if line_intersects_circ(obs.loc, obs.foes[0][0:2], friendly, 6):
                    shoot = False
        
        # Compute speed
        if path:
            if not shoot:
                turn = path_angle
            
            # Determine speed based on angle with planned path and planned distance
            maxangle = (math.pi/2)+self.settings.max_turn
            distance = (dx**2 + dy**2)**0.5
            
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
        
        # If no path was found, do nothing
        else:
            turn = 0
            speed = 0
        
        return (turn,speed,shoot)
    
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
        # Draw the agent field of vision
        fov_rect = pygame.Rect(self.observation.loc[0] - (self.settings.max_see),
                               self.observation.loc[1] - (self.settings.max_see),
                              2 * self.settings.max_see,
                               2 * self.settings.max_see)
                               
        pygame.draw.rect(surface,
                         (0,0,255),
                         fov_rect,
                         1)

        
        # Draw the agent shooting arc
        shoot_rect = pygame.Rect(self.observation.loc[0] - (self.settings.max_range),
                               self.observation.loc[1] - (self.settings.max_range),
                              2 * self.settings.max_range,
                               2 * self.settings.max_range)                       
            
        pygame.draw.arc(surface,
                           (255,0,0,90),
                           shoot_rect,
                           -self.settings.max_turn-self.observation.angle,
                           self.settings.max_turn-self.observation.angle,
                           1)
        """
        # Selected agents draw their info
        if self.selected:
            # Draw line directly to goal
            """if self.goal is not None:
                pygame.draw.line(surface,(0,0,0),self.observation.loc, self.goal)
            """
            # Draw line to goal along the planned path
            if self.goal is not None:
                path = find_path(self.observation.loc, self.goal, self.mesh, self.grid, self.settings.tilesize)
                if path:
                    for i in range(len(path)):
                        if i == 0:
                            pygame.draw.line(surface,(0,0,0),self.observation.loc, path[i])
                        else:
                            pygame.draw.line(surface,(0,0,0),path[i-1], path[i])
        
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

    def __init__(self, settings):

        # Regions are defined as ((topleft)(bottomright)). [((x1, y1), (x2, y2)), ...]
        regions = [((0,0),     (125,95)),
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
        # Possible compass directions for the agents
        directions = ["N", "E", "S", "W"]
        # self.directions = ["N", "NE" "E", "SE", "S", "SW", "W", "NW"]

        # Death timer ranges (binned)
        death_timer_ranges = ("0", "1-3", "4-6", "7-10") #(0,3,6,10,15)

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
                else:
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
            state = self.process_joint_observation() # Process the joint observation 
            self.update_policy() # Update policy when joint observation has been processed
        
    def process_joint_observation(self):
        """ Creates an abstract representation of the observation, on which we will learn.
        """
        state = State()
        # TODO: compute state values based on joint observation
        return state

    def update_policy(self):
        """ Update the joint policy of the agents based on the current state.
        """



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
            -Agent orientation (8 possible values per agent)
            -Agent ammo possession (2 possible values per agent)
            -"Almost lost" ((score = behind && time is almost up && we do not control all CPs) OR (score = almost at threshold losing value)) - (2 possible values)
            -CP's assumed to be controlled (2 possible values per control point)
            -Death timers (4 possible values per agent. Values: alive, 1-3, 4-6, 7-10)
        """

        ########################################################################
        ##### Feature parameter initializations ################################
        ########################################################################

        self.locations = defaultdict(tuple)
        self.orientations = defaultdict(tuple)
        self.death_timers = defaultdict(tuple)
        self.has_ammo = defaultdict(bool)
        self.final_stand = False
        self.control_points = defaultdict(bool)
        self.ammo_observed = defaultdict(bool)
        self.ammoRespawning = defaultdict(bool)



########################################################################
##### Strategy class ###################################################
########################################################################
class Strategy(object):

    # Our strategies define the % of bots that go for certain goals.

    # strategies =  "Name"      :  % offense  , % defensive , % get_ammo  
    strategies   = {"Offensive" : (2./3., 0    , 1./3.),
                    "Defensive" : (0.   , 2./3., 1./3.),
                    "Ammo_def"  : (0.   , 2./3., 2./3.),
                    "Ammo_off"  : (0.   , 2./3., 2./3.),
                    "Relentless": (1.   , 0    , 0)
                   }
    
    def __init__(self):
        pass
