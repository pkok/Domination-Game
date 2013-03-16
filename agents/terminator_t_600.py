class Agent(object):

    NAME = "Terminator_T-600"

    def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, blob=None, matchinfo=None):
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


        #role for agent:
        '''
        x-Ammo
        O-controlpoint

          5-O-4
          |   |
         3x   0x
          |   |
          1-O-2
        '''
        self.role = 0

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


        #assign init roles:
        '''
        if (self.team==TEAM_BLUE):
            self.role=1
        else:
            self.role=4
        '''   
        if(self.id == 0):
            self.role = 4

        if(self.id == 1):
            if(self.team == TEAM_RED):          
                self.role = 4
            else:
                self.role = 1

        if(self.id == 2):
            #self.role = 4
            self.role = 1
        

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
        turn = 0
        speed = 0
        shoot = False


        obs = self.observation
        #print(self.role)



        #set goals according to roles:

        if(self.role == 0): #left ammo (152,136)
            self.goal = (152,136)#(152,151)
            '''
            #up
            if(point_dist((152,151), obs.loc) < self.settings.tilesize-3):
                self.goal = (152,111)
            #down
            if(point_dist((152,111), obs.loc) < self.settings.tilesize-3):
                self.goal = (152,151)
            '''
        if(self.role == 3): #right ammo (312,136)
            self.goal = (312,136)#(312,151)
            '''
            #up
            if(point_dist((312,151), obs.loc) < self.settings.tilesize-3):
                self.goal = (312,111)
            #down
            if(point_dist((312,111), obs.loc) < self.settings.tilesize-3):
                self.goal = (312,151)
            '''
        if(self.role == 1): #bot cp
            self.goal = (248, 216)
        if(self.role == 4):#top cp
            self.goal = (216, 56)

        if(self.role == 2): #go to right ammo
            self.goal = (312,136)
        if(self.role == 5): #go to left ammo
            self.goal = (152,136)

        if(self.role == 6): #go to left ammo
            self.goal = (320,250)
        if(self.role == 7): #go to left ammo
            self.goal = (150,136)
            #####
            #####
        #check if there:




        # Check if agent reached goal.
        #if self.goal is not None and point_dist(self.goal, obs.loc) < self.settings.tilesize:
        #    self.goal = None

        # Walk to ammo
        #ammopacks = filter(lambda x: x[2] == "Ammo", obs.objects)
        #if ammopacks:
        #    self.goal = ammopacks[0][0:2]

        # Drive to where the user clicked
        # Clicked is a list of tuples of (x, y, shift_down, is_selected)
        if self.selected and self.observation.clicked:
            self.goal = self.observation.clicked[0][0:2]

        # Walk to random CP
        #if self.goal is None:
        #    self.goal = obs.cps[random.randint(0,len(obs.cps)-1)][0:2]

        # Shoot enemies
        shoot = False
        if (obs.ammo > 0 and obs.foes and point_dist(obs.foes[0][0:2], obs.loc) < self.settings.max_range and not line_intersects_grid(obs.loc, obs.foes[0][0:2], self.grid, self.settings.tilesize)):
            self.goal = obs.foes[0][0:2]
            shoot = True



        #turn = math.pi/4
        speed = 0
        if (not point_dist(self.goal, obs.loc) < self.settings.tilesize):
            #goto goal!

            path = find_path(obs.loc, self.goal, self.mesh, self.grid, self.settings.tilesize)
            #print(self.id,"i keep movin movin movin")
            dx = path[0][0] - obs.loc[0]
            dy = path[0][1] - obs.loc[1]
            turn = angle_fix(math.atan2(dy, dx) - obs.angle)
            if turn > self.settings.max_turn or turn < -self.settings.max_turn:
                shoot = False
            if(not shoot):
                speed = (dx**2 + dy**2)**0.5



        else: #reached whatever goal it was:
            #just circle
            '''if(self.role < 5 and self.role >= 0):
                self.role += 1
            else:
                self.role = 0
            '''
            if (self.role == 0): ##2,4
                self.role = 3

            elif (self.role == 1): ##2,4
                self.role = 1

            elif (self.role == 2): ##2,4
                self.role = 4

            elif (self.role == 3):
                self.role = 0

            elif (self.role == 4):
                self.role = 4

            elif (self.role == 5):
                self.role = 1

            '''
            if(self.role == 1 or self.role == 4):#reached the cp
                self.role += 1
            elif(self.role == 5):#clap with other agent
                for i in range(len(self.all_agents)):
                    if(self.all_agents[i].role == 0): #change others agants role !
                        self.all_agents[i].role = 1
                self.role = 0

            elif(self.role == 2): #clap with other agent
                for i in range(len(self.all_agents)):
                    if(self.all_agents[i].role == 3):
                        self.all_agents[i].role = 4
                        break
                self.role = 3
            '''
        '''
        if(self.id == 1 and not point_dist(self.goal, obs.loc) < self.settings.tilesize):
            dx = path[0][0] - obs.loc[0]
            dy = path[0][1] - obs.loc[1]
            turn = angle_fix(math.atan2(dy, dx) - obs.angle)
            if turn > self.settings.max_turn or turn < -self.settings.max_turn:
                shoot = False
            speed = (dx**2 + dy**2)**0.5
        '''
        '''
        if(self.id == 2 and not point_dist(self.goal, obs.loc) < self.settings.tilesize):
            dx = path[0][0] - obs.loc[0]
            dy = path[0][1] - obs.loc[1]
            turn = angle_fix(math.atan2(dy, dx) - obs.angle)
            if turn > self.settings.max_turn or turn < -self.settings.max_turn:
                shoot = False
            speed = (dx**2 + dy**2)**0.5
        '''

        #if(self.id = 0 and not obs.loc == self.goal):
        #    pass


        #if(self.id == 0):
        #    print(turn,speed,self.id)
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
        # Selected agents draw their info
        if self.selected:
            if self.goal is not None:
                pygame.draw.line(surface,(0,0,0),self.observation.loc, self.goal)

    def finalize(self, interrupted=False):
        """ This function is called after the game ends,
            either due to time/score limits, or due to an
            interrupt (CTRL+C) by the user. Use it to
            store any learned variables and write logs/reports.
        """
        pass

