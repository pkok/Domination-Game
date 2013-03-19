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
        self.ids = Ids()
        self.ids.all_ids.append(id)
        self.team = team
        self.settings = settings
        self.mesh = nav_mesh
        self.grid = field_grid
        self.goal = None
        self.callsign = '%s-%d'% (('BLU' if team == TEAM_BLUE else 'RED'), id)
        self.selected = False

        """Blob handling
        self.blobpath = None
        self.blobcontent = None
        
        # Read the binary blob, we're not using it though
        if blob is not None:
            # Remember the blob path so we can write back to it
            self.blobpath = blob.name
            self.blobcontent = pickle.loads(blob.read())
            print "Agent %s received binary blob of %s" % (
               self.callsign, type(self.blobcontent))
            # Reset the file so other agents can read it too.
            blob.seek(0) 
        """

    def observe(self, observation):
        """ Each agent is passed an observation using this function,
            before being asked for an action. You can store either
            the observation object or its properties to use them
            to determine your action. Note that the observation object
            is modified in place.
        """
        self.obs = observation
        self.selected = observation.selected
        
        if observation.selected:
            print observation
                    
    def action(self):
        """ This function is called every step and should
            return a tuple in the form: (turn, speed, shoot)
        """ 
        # self.set_goal()
        if self.id == self.ids.all_ids[0]:
            self.goal = self.obs.cps[0][0:2]
        if self.id == self.ids.all_ids[1]:
            self.goal = self.obs.cps[1][0:2]
        # if self.id == self.ids.all_ids[2]:
        #     self.goal = self.obs.cps[1][0:2]
        # Compute and return the corresponding action
        return self.get_action()

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
            else:
                goal_path = find_path(self.obs.loc, self.goal,
                                    self.mesh, self.grid, self.settings.tilesize)
                ammo_path = find_path(self.obs.loc, ammopacks[0][0:2],
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

            ammo_positions = [(168,168), (296,104)]
            
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

        # Default values (stand still if not path found, or if goal already reached)
        turn = 0
        speed = 0
        shoot = False

        # If goal is not reached, compute path
        if (self.obs.loc != self.goal):
            # Compute path and angle to path
            path = find_path(self.obs.loc, self.goal, self.mesh, self.grid, self.settings.tilesize)
            if path:
                dx = path[0][0] - self.obs.loc[0]
                dy = path[0][1] - self.obs.loc[1]
                path_angle = angle_fix(math.atan2(dy, dx) - self.obs.angle)
                path_dist = (dx**2 + dy**2)**0.5

                # Compute shoot and turn
                shoot, turn = self.compute_shoot(path_angle)
                # Compute speed
                speed = self.compute_speed(turn, path_dist)

        # If goal reached, only move to shoot opponent agents
        else:
            shoot, turn = self.compute_shoot(0)

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
                    if right_hit and line_intersects_circ(self.obs.loc, right_coords, friendly, 6):
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
        
        # Selected agents draw their info
        if self.selected:
            # Draw line directly to goal
            """if self.goal is not None:
                pygame.draw.line(surface,(0,0,0),self.obs.loc, self.goal)
            """
            # Draw line to goal along the planned path
            if self.goal is not None:
                path = find_path(self.obs.loc, self.goal, self.mesh, self.grid, self.settings.tilesize)
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

########################################################################
##### Singleton class ##################################################
########################################################################
    
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


class Ids(object):
    """ A singleton object representing the joint observation of all our
        agents. It should be updated during Agent.observe.
    """
    __metaclass__ = Singleton

    def __init__(self):
        self.all_ids = []