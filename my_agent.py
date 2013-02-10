
class Agent(object):
    
    NAME = "default_agent"
    
    def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, blob=None):
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
        self.observation = observation
        self.selected = observation.selected
        
        if observation.selected:
            print observation
                    
    def action(self):
        """ This function is called every step and should
            return a tuple in the form: (turn, speed, shoot)
        """ 
        # Set the goal of the action
        self.setGoal(self.observation)
        
        # Compute and return the corresponding action
        return self.getAction(self.observation)
    
    def setGoal(self, obs):
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
    
        # Shoot enemies
        if (obs.ammo > 0 and 
            obs.foes and 
            point_dist(obs.foes[0][0:2], obs.loc) < self.settings.max_range and
            not line_intersects_grid(obs.loc, obs.foes[0][0:2], self.grid, self.settings.tilesize)):
            self.goal = obs.foes[0][0:2]
    
    def getAction(self, obs):
        """This function returns the action for the agent.
           Patrick, Stijn: this is your turf!
        """
        # Try to shoot enemies if they are within range
        shoot = False
        if (obs.ammo > 0 and 
            obs.foes and 
            point_dist(obs.foes[0][0:2], obs.loc) < self.settings.max_range and
            not line_intersects_grid(obs.loc, obs.foes[0][0:2], self.grid, self.settings.tilesize)):
            shoot = True

        # Compute path, angle and drive
        path = find_path(obs.loc, self.goal, self.mesh, self.grid, self.settings.tilesize)
        if path:
            dx = path[0][0] - obs.loc[0]
            dy = path[0][1] - obs.loc[1]
            turn = angle_fix(math.atan2(dy, dx) - obs.angle)
            
            # If target is outside the shooting angle, do not shoot
            if turn > self.settings.max_turn or turn < -self.settings.max_turn:
                shoot = False
            
            # If target requires more turning than possible, stand still, else move required distance
            slack = 1.1
            if turn > self.settings.max_turn*slack or turn < -self.settings.max_turn*slack:
                speed = 0
            else:
                speed = (dx**2 + dy**2)**0.5

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
        
