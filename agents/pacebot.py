"""
Created on Tue Feb 05 11:38:04 2013

@author: Steven
"""
from domination import utilities
from utilities import *

class SharedKnowledge:
    '''
    Class for the shared knowledge between agents.
    '''
    
    def __init__( self ):
        # Dictonary keyed by location tuple: location: timer
        self.ammo_info = dict()
        
    def add_ammo_loc( self, loc ):
        '''
        Adds the given location to the shared knowledge (if it isn't known)
        already.
        '''
        if loc not in self.ammo_info:
            self.ammo_info[loc] = 0
            
    def get_ammo_time( self, ammo_loc ):
        '''
        Returns the respawn time for the given ammo location.
        Note that this is based on the knowledge of this team only, so may be
        inaccurate.
        '''
        return self.ammo_info[ammo_loc]
        
    def set_ammo_time( self, ammo_loc, time ):
        '''
        Sets the respawn time for the given ammo location to the given amount.
        '''
        # TODO: retrieve respawn time from settings
        self.ammo_info[ammo_loc] = time
        
    def get_ammo_loc( self, my_loc ):
        '''
        Returns the best place to go to for ammo:
            if two locations have ammo, nearest one is returned
            if no locations have ammo, shortest spawn time is returned
        '''
        # TODO get max respawn time from settings and best dist
        best_time = 100
        best_ammo = None
        best_dist = None
        for ammo in self.ammo_info:
            time = self.ammo_info[ammo]
            if time < best_time:
                best_ammo = ammo
                best_time = time
                best_dist = point_dist( my_loc, ammo )
            elif time == best_time:
                d = point_dist( my_loc, ammo )
                if d < best_dist:
                    best_ammo = ammo
                    best_dist = d
        return best_ammo
        
    def update( self ):
        '''
        Updates the shared information. For now only the respawn times for ammo.
        '''
        for ammo in self.ammo_info:
            self.ammo_info[ammo] = max( 0, self.ammo_info[ammo] - 1 )
        
    
class Agent(object):

        NAME = "PaceBot" # Replay filenames and console output will contain this name.        
        
        def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, **kwargs):
            
            self.id = id
            self.team = team
            self.mesh = nav_mesh
            self.grid = field_grid
            self.settings = settings
            self.goal = None
            self.callsign = '%s-%d'% ( ( 'BLU' if team == TEAM_BLUE else 'RED'), id )
            
            self.relocate = False
            
            self.last_seen = []            
            
            # Recommended way to share variables between agents.
            if id == 0:
                self.all_agents = self.__class__.all_agents = []
                self.shared_knowledge = self.__class__.shared_knowledge = SharedKnowledge()
                
            self.all_agents.append(self)
            
        def observe(self, observation):
            self.observation = observation
            
            # Check for ammo            
            ammopacks = filter( lambda x: x[2] == "Ammo", observation.objects )
            
            for ammo in self.last_seen:
                ammo_loc = ammo[0:2]
                if point_dist( ammo_loc, observation.loc ) < self.settings.max_see:
                    # At this point we should see the ammo location, so if 
                    # we see no ammo, it has been taken
                    if ammo_loc not in ammopacks:
                        self.shared_knowledge.set_ammo_time( ammo_loc, self.settings.ammo_rate )
                        
            self.last_seen = ammopacks
            
            if ammopacks:
                self.shared_knowledge.add_ammo_loc( ammopacks[0][0:2] ) 
        
        def action(self):
            obs = self.observation
                        
            if self.goal is None:
                if self.team == TEAM_BLUE:
                    if self.id == 0:
                        self.goal = obs.cps[0][0:2]
                    elif self.id == 2:
                        self.goal = obs.cps[1][0:2]
                    elif self.id == 1:
                        self.goal = obs.cps[1][0:2] 
                else:
                    if self.id == 0:
                        self.goal = obs.cps[1][0:2]
                    elif self.id == 2:
                        self.goal = obs.cps[0][0:2]
                    elif self.id == 1:
                        self.goal = obs.cps[0][0:2]
            else:               
                if self.id == 1:
                    loc = self.shared_knowledge.get_ammo_loc( obs.loc )
                    if loc:
                        self.goal = loc
                
            path = find_path( obs.loc, self.goal, self.mesh, self.grid, self.settings.tilesize )
            if path:
                dx = path[0][0] - obs.loc[0]
                dy = path[0][1] - obs.loc[1]
                
                speed = ( dx ** 2 + dy ** 2 ) ** 0.5
                
                turn = angle_fix( math.atan2(dy, dx) - obs.angle )

                if abs( turn ) > self.settings.max_turn:
                    speed = 0
                
                #turn = median( [self.settings.max_turn, -self.settings.max_turn, turn] )
            else:
                turn = 1
                speed = 0

            if self.id == 0:
                self.shared_knowledge.update()

            return ( turn, speed, False )
        
        def debug(self, surface):
            import pygame
            
            # First agent clears the screen
            if self.id == 0:
                surface.fill( ( 0, 0, 0 ) )
                #surface.set_colorkey( ( 5, 5, 5 ) )
                
                
            # Selected agents draw their info
            #if self.observation.selected:
                
            #pygame.draw.circle( surface, ( 0, 0, 0, 0 ), self.observation.loc, self.settings.max_see )
            loc = self.observation.loc
            tile = self.settings.tilesize
            rect = pygame.Rect( loc[0] - tile * 5, loc[1] - tile * 5,tile * 10, tile * 10 )
            pygame.draw.rect( surface, (0,0,0,0), rect )
            #ammopacks = filter( lambda x: x[2] == "Ammo", self.observation.objects )
            if self.id == 0:
                mycolor = (255,0,0)
                myrange = 8
            elif self.id == 1:
                mycolor = (0,255,0)
                myrange = 7
            else:
                mycolor = (0,0,255)
                myrange = 6
            for obj in self.observation.objects:
                pygame.draw.circle( surface, mycolor, obj[0:2], myrange, 2 )
                
            for obj in self.observation.foes:
                pygame.draw.circle( surface, mycolor, obj[0:2], myrange, 2 )
            for obj in self.observation.friends:
                pygame.draw.circle( surface, mycolor, obj[0:2], myrange, 2 )
                #if self.path:
                #    ppp = self.path[0]
                #    for pp in self.path: 
                #        pygame.draw.line( surface, (0,0,0), ppp, pp )
                #        ppp = pp
        
        def finalize(self, interrupted=False):
            pass

