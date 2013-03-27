from collections import defaultdict, namedtuple
from itertools import product
from random import randint, choice
import math
import sys
import time
import cPickle as pickle

from domination.libs import astar
astar = astar.astar

DRAW_MESH_POINTS = False
DRAW_MESH_POINT_COORDS = False
DRAW_AGENT_DOT = True
DRAW_AGENT_VIEW = True
DRAW_AGENT_AMMO = True
DRAW_AGENT_GOAL = True
DRAW_PF_ALL = True
DRAW_PF_PROMINENT = False

class Agent(object):

    NAME = "Dalek Thay"
    RADIUS = 6.0 # Radius in pixels of an agent.
    INTEREST_POINTS = {'cp1':(232, 56), 'cp2':(264, 216), 'am1':(184,168), 'am2':(312,104)}
    
    def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, blob=None, matchinfo=None, *args, **kwargs):
        """ Each agent is initialized at the beginning of each game.
            The first agent (id==0) can use this to set up global variables.
            Note that the properties pertaining to the game field might not be
            given for each game.
        """
        
        self.id = id #NamedInt(id)
        self.team = team
        self.settings = settings
        self.epsilon = 0.05
        self.gamma = 0.9
        learning_weighting = ((matchinfo.num_games - matchinfo.current + 1) / (matchinfo.num_games)) # decrease learning rate over time
        self.alpha = 0.2 * learning_weighting
        # WoLF
        self.deltaWin = 0.1 * learning_weighting
        self.deltaLose = self.deltaWin * 2
        self.useWoLF = True
        # /WoLF
        self.initial_value = 10
        self.number_of_agents = 3
        self.joint_observation = JointObservation(settings, field_grid, team, nav_mesh, self.epsilon, self.gamma, self.alpha, self.deltaWin, self.deltaLose, self.useWoLF, self.initial_value, self.number_of_agents, Agent.INTEREST_POINTS)
        self.mesh = self.joint_observation.mesh
        self.sharia_point = []
        self.haram_point = []
        for point in self.mesh:
            if point[0] in (121, 151, 345, 375):
                self.sharia_point.append(point)
            if point[0] >= 184 and point[0] <= 312:
                self.haram_point.append(point)
        self.grid = field_grid
        self.goal = None
        #self.callsign = '%s-%d'% (('BLU' if team == TEAM_BLUE else 'RED'), id)
        self.callsign = '%s-%d'% (('DECEPTICON' if team == TEAM_BLUE else 'AUTOBOT'), id)
        self.selected = False

        self.blobpath = None
        
        # Read the binary blob, we're not using it though
        if not self.joint_observation.state_action_pairs and blob is not None:
            # Remember the blob path so we can write back to it
            self.blobpath = blob.name
            self.joint_observation.state_action_pairs = pickle.loads(blob.read())
            print "%s received binary blob of %s" % (
                self.id, type(self.joint_observation.state_action_pairs))
            # Reset the file so other agents can read it too.
            blob.seek(0)

    def observe(self, observation):
        """ Each agent is passed an observation using this function,
            before being asked for an action. You can store either
            the observation object or its properties to use them
            to determine your action. Note that the observation object
            is modified in place.
        """
        self.obs = observation
        self.selected = observation.selected
        
        #start_time = time.time()
        self.joint_observation.update(self.id, observation)
        #print 'Time to update Joint Obs: {}\n'.format(time.time() - start_time)
        
        if observation.selected:
            print observation
        
    def action(self):
        """ This function is called every step and should
            return a tuple in the form: (turn, speed, shoot)
        """ 
        # Compute the goal
        # self.set_goal_sarsa()
        self.set_goal_hardcoded()
        
        # Compute and return the corresponding action
        action = self.get_action()
        self.joint_observation.update_action(self.id, action)
        return action

    def set_goal_sarsa(self):
        """This function sets the goal for the agent.
        """
        index = sorted(self.joint_observation.friends.keys()).index(self.id)
        goal_region = self.joint_observation.new_joint_action[index]
        self.goal = self.joint_observation.coords[goal_region]

        # Drive to where the user clicked
        # Clicked is a list of tuples of (x, y, shift_down, is_selected)
        if self.selected and self.obs.clicked:
            self.goal = self.obs.clicked[0][0:2]

    def set_goal_hardcoded(self):
        """This function sets the goal for the agent.
        """

        # If the agent is not agent 0, its goal has already been computed
        if self.id != 0:
            self.goal = self.joint_observation.goals[self.id]
            return
        
        # Intinialise some handy variables
        goals = [(0,0), (0,0), (0.0)]
        assigned = []       
        have_ammo = []
        for agent_id, agent in self.joint_observation.friends.items():
            if agent[3] > 1:
                have_ammo.append(agent_id)
        
        cp1_loc = Agent.INTEREST_POINTS['cp1']
        cp2_loc = Agent.INTEREST_POINTS['cp2']
        am1_loc = Agent.INTEREST_POINTS['am1']
        am2_loc = Agent.INTEREST_POINTS['am2']
            
        cp1_dist = []
        cp2_dist = []
        am1_dist = []
        am2_dist = []
        
        am1 = False
        am2 = False
        am1_timer = 0
        am2_timer = 0
        
        # placeholder variables
        # positions on the field of interest points cp1=(13, 3) cp2=(15, 13) am1=(18, 6) am2=(10, 10)
        cp1_sqr = self.joint_observation.get_tile_coords(cp1_loc)
        cp2_sqr = self.joint_observation.get_tile_coords(cp2_loc)
        am1_sqr = self.joint_observation.get_tile_coords(am1_loc)
        am2_sqr = self.joint_observation.get_tile_coords(am2_loc)
        pf_probs = self.joint_observation.mc_probs
        # DEBUG 
        #print str(pf_probs)
        
        cp1_prob = max( [ pf_probs[0][cp1_sqr], pf_probs[1][cp1_sqr], pf_probs[2][cp1_sqr] ] )
        cp2_prob = max( [ pf_probs[0][cp2_sqr], pf_probs[1][cp2_sqr], pf_probs[2][cp2_sqr] ] )
        am1_prob = max( [ pf_probs[0][am1_sqr], pf_probs[1][am1_sqr], pf_probs[2][am1_sqr] ] )
        am2_prob = max( [ pf_probs[0][am2_sqr], pf_probs[1][am2_sqr], pf_probs[2][am2_sqr] ] )
        
        am_enemy_loc = am1_loc if self.team == TEAM_BLUE else am2_loc
        am_enemy_dist = am1_dist if self.team == TEAM_BLUE else am2_dist
        am_enemy = am1 if self.team == TEAM_BLUE else am2
        
        
        for obj in self.joint_observation.objects:
            if obj[0:2] == am1_loc:
                am1_timer = self.joint_observation.step - self.joint_observation.objects[obj][0]
                if am1_timer != 0:
                    am1_timer = max(0, self.settings.ammo_rate-am1_timer+1)
                else:
                    am1 = True
            if obj[0:2] == am2_loc:
                am2_timer = self.joint_observation.step - self.joint_observation.objects[obj][0]
                if am2_timer != 0:
                    am2_timer = max(0, self.settings.ammo_rate-am2_timer+1)
                else:
                    am2 = True

        for id in range(3):
            # Add respawn timer to distance calculation
            if self.joint_observation.friends[id].respawn_in > 0:
                respawn_time = self.joint_observation.friends[id].respawn_in
            else:
                respawn_time = 0
            # Calculate distances
            cp1_dist.append(self.joint_observation.paths[id]['cp1'][1] + respawn_time)
            cp2_dist.append(self.joint_observation.paths[id]['cp2'][1] + respawn_time)
            am1_dist.append(self.joint_observation.paths[id]['am1'][1] + respawn_time)
            am2_dist.append(self.joint_observation.paths[id]['am2'][1] + respawn_time)
        
        team = int(self.team == TEAM_BLUE)
        controlling_cps = map(lambda cp: cp[2] == team, self.obs.cps)

        if all(controlling_cps):
            danger_zone = min([self.settings.max_range,
                self.settings.max_speed])
            def cp_under_pressure(cp):
                for foe in self.joint_observation.foes[self.joint_observation.step]:
                    dx = foe[0] - cp[0]
                    dy = foe[1] - cp[1]
                    distance = (dx**2 + dy**2) ** 0.5
                    if distance < danger_zone:
                        return True
                return False
            safe_cps = filter(lambda x: not cp_under_pressure(x), self.obs.cps)
            if safe_cps:
                cp_distance = {}
                nearby_agent_id = {}
                paths = {}
                for cp in safe_cps:
                    cp = cp[:2]
                    paths[cp] = dict()
                    cp_distance[cp] = float("inf")
                    # is in joint_observation, recode!
                    for agent_id, agent in self.joint_observation.friends.items():
                        paths[cp][agent_id] = find_single_path(agent[:2], agent[2], cp[:2], self.mesh, self.grid,
                                self.settings.max_speed, self.settings.max_turn, self.settings.tilesize)
                    for agent_id, agent_path in paths[cp].items():
                        if len(agent_path) < cp_distance[cp]:
                            cp_distance[cp] = len(agent_path)
                            nearby_agent_id[cp] = agent_id
                random.shuffle(safe_cps)
                chosen_cp = min(safe_cps, key=lambda cp:
                        self.joint_observation.friends[nearby_agent_id[cp[:2]]].ammo)
                chosen_agent_id = nearby_agent_id[cp]
                chosen_agent = self.joint_observation.friends[chosen_agent_id]
                dx = Agent.INTEREST_POINTS['am1'][0] - chosen_agent[0]
                dy = Agent.INTEREST_POINTS['am1'][1] - chosen_agent[1]
                distance_am1 = (dx**2 + dy**2) ** 0.5
                dx = Agent.INTEREST_POINTS['am2'][0] - chosen_agent[0]
                dy = Agent.INTEREST_POINTS['am2'][1] - chosen_agent[1]
                distance_am2 = (dx**2 + dy**2) ** 0.5
                if distance_am1 < distance_am2: 
                    chosen_am = Agent.INTEREST_POINTS['am1']
                else:
                    chosen_am = Agent.INTEREST_POINTS['am2']
                # Agents not "associated" with a CP
                selectable_agents = filter(lambda friend: 
                        friend not in nearby_agent_id.values(), 
                        self.joint_observation.friends)
                if len(selectable_agents) >= 1:
                    # Select the nearest to the CP
                    # Move this one to the CP
                    # Move chosen_agent to the chosen_am
                    #return after setting the goals, with commands such as...
                    #     self.joint_observation.goals = goals
                    #     self.goal = goals[self.id]
                    pass

        #If no agent has ammo, follow this policy
        if len(have_ammo) == 0:
            # If there is a much higher probability of an enemy present at one cp, 
            # then only send one agent to the cp with very low probability
            if cp1_prob > cp2_prob *2:
                if not controlling_cps[1]:
                    # Send agent closest to cp2 to cp2
                    min_dist, min_id = 10000.0, 0
                    for id in range(3):
                        if (id not in assigned) and cp2_dist[id] < min_dist:
                            min_dist = cp2_dist[id]
                            min_id = id
                    goals[min_id] = cp2_loc
                    assigned.append(min_id)
            elif cp1_prob*2< cp2_prob:
                if not controlling_cps[0]:
                    # Send agent closest to cp1 to cp1
                    min_dist, min_id = 10000.0, 0
                    for id in range(3):
                        if cp1_dist[id] < min_dist:
                            min_dist = cp1_dist[id]
                            min_id = id
                    goals[min_id] = cp1_loc
                    assigned.append(min_id)
            # if enemy presence is low at both, then just go for both
            else:
                # Send agent closest to cp1 to cp1
                min_dist, min_id = 10000.0, 0
                for id in range(3):
                    if cp1_dist[id] < min_dist:
                        min_dist = cp1_dist[id]
                        min_id = id
                goals[min_id] = cp1_loc
                assigned.append(min_id)
            
                # Send agent closest to cp2 to cp2
                min_dist, min_id = 10000.0, 0
                for id in range(3):
                    if (id not in assigned) and cp2_dist[id] < min_dist:
                        min_dist = cp2_dist[id]
                        min_id = id
                goals[min_id] = cp2_loc
                assigned.append(min_id)

            # Send the other agent(s) to collect ammo
            for id in range(3):
                if id not in assigned:
                    # Go to closest ammo
                    if am1 and am2 and am1_loc not in goals and am2_loc not in goals:
                        goals[id] = am1_loc if (am1_dist[id] < am2_dist[id]) else am2_loc
                    elif am1 and am1_loc not in goals:
                        goals[id] = am1_loc
                    elif am2 and am2_loc not in goals:
                        goals[id] = am2_loc
                    else:
                        am1_delay = max(am1_dist[id], am1_timer)
                        am2_delay = max(am2_dist[id], am2_timer)
                        
                        if am1_delay < am2_delay:
                            goals[id] = am1_loc
                        else:
                            goals[id] = am2_loc
                    assigned.append(id)

        # If one agent has ammo, follow this policy
        elif len(have_ammo) == 1:
            # Send the agent with ammo to nearest uncontrolled cp, and one to the other cp, and one for ammo

            # Assign the agent with ammo to the nearest cp with highest enemy probability
            ammo_id = have_ammo[0]
            if cp1_prob > cp2_prob*2:
                goals[ammo_id] = cp1_loc
            elif cp1_prob*2 < cp2_prob:
                goals[ammo_id] = cp2_loc
            # if the probabilities are the same then pick the closest one
            else:
                goals[ammo_id] = cp1_loc if (cp1_dist[ammo_id] < cp2_dist[ammo_id]) else cp2_loc
            assigned.append(ammo_id)
            
            # Send agent closest to other cp to other cp
            if cp1_loc in goals:
                other_cp_dist, other_cp_loc = cp2_dist, cp2_loc
            else:
                other_cp_dist, other_cp_loc = cp1_dist, cp1_loc
                
            min_dist, min_id = 10000.0, 0
            for id in range(3):
                if (id not in assigned) and other_cp_dist[id] < min_dist:
                    min_dist = other_cp_dist[id]
                    min_id = id
            goals[min_id] = other_cp_loc
            assigned.append(min_id)
            
            # Send the other agent(s) to collect ammo
            for id in range(3):
                if id not in assigned:
                    # Go to closest ammo
                    if am1 and am2 and am1_loc not in goals and am2_loc not in goals:
                        goals[id] = am1_loc if (am1_dist[id] < am2_dist[id]) else am2_loc
                    elif am1 and am1_loc not in goals:
                        goals[id] = am1_loc
                    elif am2 and am2_loc not in goals:
                        goals[id] = am2_loc
                    else:
                        am1_delay = max(am1_dist[id], am1_timer)
                        am2_delay = max(am2_dist[id], am2_timer)
                        
                        if am1_delay < am2_delay:
                            goals[id] = am1_loc
                        else:
                            goals[id] = am2_loc
                    assigned.append(id)
        
        # If two agents have ammo, follow this policy
        elif len(have_ammo) == 2:
            # Both agents with ammo go for a closest control point each, the other gets ammo
            
            # Assign the agent with ammo to the nearest cp
            ammo_id = have_ammo[0]
            goals[ammo_id] = cp1_loc if (cp1_dist[ammo_id] < cp2_dist[ammo_id]) else cp2_loc
            assigned.append(ammo_id)
        
            # Send other agent with ammo to other cp
            for id in have_ammo:
                if id not in assigned:
                    goals[id] = cp2_loc if (cp1_loc in goals) else cp1_loc
            
            # Send the other agent(s) to collect ammo
            for id in range(3):
                if id not in assigned:
                    # Go to closest ammo
                    if am1 and am2 and am1_loc not in goals and am2_loc not in goals:
                        goals[id] = am1_loc if (am1_dist[id] < am2_dist[id]) else am2_loc
                    elif am1 and am1_loc not in goals:
                        goals[id] = am1_loc
                    elif am2 and am2_loc not in goals:
                        goals[id] = am2_loc
                    else:
                        am1_delay = max(am1_dist[id], am1_timer)
                        am2_delay = max(am2_dist[id], am2_timer)
                        
                        if am1_delay < am2_delay:
                            goals[id] = am1_loc
                        else:
                            goals[id] = am2_loc
                    assigned.append(id)
            
        # If three agents have ammo, and at least one cp is uncontrolled, follow this policy
        elif len(have_ammo) == 3:
            # Send two agents to one unheld cp, and one agent to the other cp
            
            # Send agent closest to cp1 to cp1
            min_dist, min_id = 10000.0, 0
            for id in range(3):
                if cp1_dist[id] < min_dist:
                    min_dist = cp1_dist[id]
                    min_id = id
            goals[min_id] = cp1_loc
            assigned.append(min_id)
            
            # Send agent closest to cp2 to cp2
            min_dist, min_id = 10000.0, 0
            for id in range(3):
                if (id not in assigned) and cp2_dist[id] < min_dist:
                    min_dist = cp2_dist[id]
                    min_id = id
            goals[min_id] = cp2_loc
            assigned.append(min_id)
            
            # If at least one cp is uncontrolled, send other agent to closest cp
            if not all(controlling_cps):
                for id in range(3):
                    if id not in assigned:
                        goals[id] = cp1_loc if (cp1_dist[id] < cp2_dist[id]) else cp2_loc
                        assigned.append(id)
        
            # If both cps are controlled, send other agent to ammo spawn closest to enemy base
            else:
                for id in range(3):
                    if id not in assigned:
                        goals[id] = am1_loc if (self.team == TEAM_BLUE) else am2_loc
                        assigned.append(id)
        
        # Set the joint goal
        self.joint_observation.goals = goals
        
        # Instantiate goal for agent 0
        self.goal = goals[0]
    
    def find_path_recursively(self, start, angle, ends, mesh, grid, waypoints=[], return_type="dict"):
        """ Uses astar to find a path from start to each point in end, with optional waypoints in between
            using the given mesh and tile grid.
        """ 
        if return_type == "dict":
            path_return = {}
        else:
            path_return = []
        for end in ends:
            end_point = ()
            if type(ends) is dict:
                end_point = ends[end]
            else:
                end_point = end
            print "endpoint: "+str(end_point)
            path = [start]
            for pt in waypoints:
                print "waypoint: " + str(pt)
                path.extend( find_single_path(path[-1], angle, pt, mesh, grid,
                    self.settings.max_speed, self.settings.max_turn, self.settings.tilesize) )
                print "path extended with waypoint: " + str(path)
            print "last element in path: " + str(path[-1])
            path.extend( find_single_path(path[-1], angle, end_point, mesh, grid, 
                self.settings.max_speed, self.settings.max_turn, self.settings.tilesize) )
            del path[0]
            if return_type == "dict":
                path_return[end_point] = path
            else:
                path_return.append(path)
            print "path: " + str(path)
        return path_return


    def find_detour(self, goal):
        """ This function is used to calculate the paths to a goal
            for a certain agent while including waypoints to enemies in it's path
        """
        # get all known enemy positions and the estimation from the Particle Filter
        enemy_pos = self.get_enemy_pos( (0, 0), (460, 260) )
        enemy_locs = []
        for enemy in enemy_pos:
            if enemy != []:
                enemy_locs.append(enemy[0][1])
        
        # calculate each path to the enemies        
        enemy_paths = find_all_paths(self.obs.loc, self.obs.angle, enemy_locs, self.mesh, self.grid)
        enemy_dist = []
        for path in enemy_paths.values():
            enemy_dist.append(path[1])
        
        path_goal = find_all_paths(self.obs.loc, self.obs.angle, [goal], self.mesh, self.grid)
        path = path_goal[goal]
        # select the shortest path to an enemy
        min_dist, closest_enemy = path_goal[goal][1], goal
        for i in range(len(enemy_dist)):
            if enemy_dist[i] < min_dist:
                min_dist = enemy_dist[i]
                closest_enemy = enemy_locs[i]
                path = enemy_paths[closest_enemy]
        
        # takes the shortest path to either an enemy or a goal
        return path
        
        
    def get_enemy_pos(self, topleft, bottomright):
        """ This function uses the observation to pinpoint enemy positions 
            if it cannot then it will use the particle filter information to
            estimate the positions
        """
        
        
        enemy_positions = []
        """
        #check observations for enemies within the region
        print str(self.joint_observation.foes[self.joint_observation.step]) + " foes at current timestep"
        for foe in self.joint_observation.foes[self.joint_observation.step]:
            pos = self.joint_observation.get_tile_coords((foe[0], foe[1])) 
            if pos >= topleft and pos <= bottomright:
                enemy_positions.append([(1, pos)])
        
        # exclude lists that has the highest probability of the observed enemies
        exclude = []
        for enemy in enemy_positions:
            highest_prob = -1
            list_nr = -1
            for i in range(len(self.joint_observation.mc_probs)):
                if not( i in exclude ) and enemy[0][1] in self.joint_observation.mc_probs[i]:
                    if self.joint_observation.mc_probs[i][enemy[0][1]] > highest_prob:
                        highest_prob = self.joint_observation.mc_probs[i][enemy[0][1]]
                        list_nr = i
            exclude.append(list_nr)
        """
        
        # of the remaining lists
        # make a list of the probabilities within the region topleft to bottomright
        prob_list = []
        for i in range(len(self.joint_observation.mc_probs)):
            #if not( i in exclude ):
                prob_list.append([])
                for x in range(int(math.floor(topleft[0]/16.0)), int(math.floor(bottomright[0]/16.0))+1):
                    for y in range(int(math.floor(topleft[1]/16.0)), int(math.floor(bottomright[1]/16.0))+1):
                        x_coord = x * 16
                        y_coord = y * 16
                        if (x_coord, y_coord) in self.joint_observation.mc_probs[i]:
                            prob_list[-1].append( (self.joint_observation.mc_probs[i][(x_coord, y_coord)], (x_coord, y_coord)) )
        
        threshold = 0
        # sort the remaining lists
        sorted_prob_list = []
        
        for i in range(len(prob_list)):
            temp_list = sorted(prob_list[i], reverse=True)
            # pick items with a probability above the threshold
            item_list = []
            for item in temp_list:
                if item[0] > threshold:
                    item_list.append(item)
            sorted_prob_list.append(item_list)
        # DEBUG 
        # print str(sorted_prob_list)
        # results are the observed enemy positions and the picked PF positions
        enemy_positions.extend(sorted_prob_list)
        
        return enemy_positions
    
    
    def get_action(self):
        """This function returns the action tuple for the agent.
        """
        path = []
        for ip in Agent.INTEREST_POINTS:
            if self.goal == Agent.INTEREST_POINTS[ip]:
                if self.obs.ammo:
                    # set path to enemy or goal whichever is closer
                    self.joint_observation.paths[self.id][ip] = self.find_detour(self.goal)
                path = self.joint_observation.paths[self.id][ip][0]
        
        if not path:
            # Compute path and angle to path
            path = find_single_path(self.obs.loc, self.obs.angle, self.goal, self.mesh, self.grid,
                                self.settings.max_speed, self.settings.max_turn, self.settings.tilesize)
        if path:
            dx = path[0][0] - self.obs.loc[0]
            dy = path[0][1] - self.obs.loc[1]
            path_angle = angle_fix(math.atan2(dy, dx) - self.obs.angle)
            path_reverse_angle = angle_fix(math.atan2(dy, dx) - self.obs.angle + math.pi)
            speed = 1
            if self.allow_reverse_gear(self.obs.loc, path, path_angle, path_reverse_angle):
                path_angle = path_reverse_angle
                speed = -1
            path_dist = (dx**2 + dy**2)**0.5

            # Compute shoot and turn
            shoot, turn = self.compute_shoot(path_angle)
            # Compute speed
            speed *= self.compute_speed(turn, path_angle, path_dist)

        # If no path was found, do nothing
        else:
            turn = 0
            speed = 0
            shoot = False
        
        # If the agent's goal is where he is already at, posture to face the right way
        if not shoot and path[0] == self.goal and speed == 0.0:
            turn = self.defend()
        
        return (turn,speed,shoot)


    def allow_reverse_gear(self, loc, path, forward_angle, reverse_angle):
        """ Is it safe/smart to drive backwards?
        """
        switch_reverse = False
        for point in self.sharia_point:
            dx = point[0] - loc[0]
            dy = point[1] - loc[1]
            dist = (dx**2 + dy**2) ** 0.5
            if dist < 10:
                switch_reverse = True
                break
        switch_reverse = switch_reverse and path[0] in self.haram_point
        reverse_angle_criterion = math.ceil(abs(reverse_angle) / self.settings.max_turn)
        forward_angle_criterion = math.ceil(abs(forward_angle) / self.settings.max_turn)
        return ((switch_reverse or not self.obs.ammo) and
                reverse_angle_criterion < forward_angle_criterion)
        return abs(reverse_angle) < abs(forward_angle)
    
    def compute_shoot(self, path_angle):
        """This function returns shoot and turn actions for the agent
        """
        shoot = False
        turn = path_angle
        
        # Check for ammo and nearby enemies
        if self.obs.ammo > 0 and self.obs.foes:
            # Calculate which foes are roughly within a possible angle or distance to shoot
            approx_shootable = []
            targeted_foes = self.joint_observation.targeted_foes()
            for foe in self.obs.foes:
                # Calculate angle and distance to center of foe
                dx = foe[0] - self.obs.loc[0]
                dy = foe[1] - self.obs.loc[1]
                cen_angle = angle_fix(math.atan2(dy, dx) - self.obs.angle)
                cen_dist = (dx**2 + dy**2) ** 0.5
                
                if (abs(cen_angle) <= (self.settings.max_turn + math.pi/6) and 
                    cen_dist <= (self.settings.max_range + Agent.RADIUS)):
                    approx_shootable.append((foe[0], foe[1], cen_angle, cen_dist, dx, dy))
            
            # Check for obstruction and compute best shooting angle per foe
            really_shootable = []
            for foe in approx_shootable:                    
                # Calculate distance from foe center to edges
                dx = foe[4]
                dy = foe[5]
                edge_angle = (math.pi/2)-math.asin(dx/foe[3])
                dx_edge = math.sin(edge_angle) * Agent.RADIUS * 0.85
                dy_edge = math.cos(edge_angle) * Agent.RADIUS * 0.85
                if dy > 0:
                    dx_edge = -dx_edge
 
                # Calculate angles and coords of foe's edges
                center_angle = foe[2]
                left_angle = angle_fix(math.atan2(dy - dy_edge, dx - dx_edge) - self.obs.angle)
                right_angle = angle_fix(math.atan2(dy + dy_edge, dx + dx_edge) - self.obs.angle)
                left_coords = (foe[0] - dx_edge, foe[1] - dy_edge)
                right_coords = (foe[0] + dx_edge, foe[1] + dy_edge)
                edge_distance = ((dx + dx_edge)**2 + (dy + dy_edge)**2) ** 0.5

                center_hit = self.can_hit_target(edge_distance, center_angle, foe[0:2], center=True)
                left_hit = self.can_hit_target(edge_distance, left_angle, left_coords)
                right_hit = self.can_hit_target(edge_distance, right_angle, right_coords)

                # Check optimal angle to shoot foe depending on which parts can be hit
                opt_angle = 0
                if center_hit and left_hit and right_hit:
                    opt_angle = self.calc_optimal_angle(left_angle, right_angle, True, path_angle)
                elif center_hit and left_hit and not right_hit:
                    opt_angle = self.calc_optimal_angle(left_angle, center_angle, True, path_angle)
                elif center_hit and right_hit and not left_hit:
                    opt_angle = self.calc_optimal_angle(center_angle, right_angle, True, path_angle)
                elif right_hit and left_hit and not center_hit:
                    opt_angle = self.calc_optimal_angle(left_angle, right_angle, False, path_angle)
                elif center_hit and not right_hit and not left_hit:
                    opt_angle = center_angle
                elif left_hit and not center_hit and not right_hit:
                    opt_angle = left_angle
                elif right_hit and not center_hit and not left_hit:
                    opt_angle = right_angle
                
                if center_hit or left_hit or right_hit:
                    really_shootable.append((foe[0], foe[1], opt_angle))
            
            # Shoot the foe that requires the agent to deviate from its path the least
            if really_shootable:
                best_difference = 100.0
                # Compute minimal angle to rotate
                for foe in really_shootable:
                    current_difference = abs(path_angle - foe[2]) 
                    if current_difference < best_difference:
                        best_difference = current_difference
                        turn = foe[2]
                        shoot = foe[:2]
                # Compute which agent you will hit
                bullet_angle = turn + self.obs.angle
                bullet_path = (math.cos(bullet_angle), math.sin(bullet_angle))
                bullet_path = point_mul(bullet_path, self.settings.max_range)
                smallest_distance = 1.0
                for foe in self.obs.foes:
                    intersection = line_intersects_circ(self.obs.loc, bullet_path, foe[0:2], Agent.RADIUS)
                    if intersection and intersection[0] < smallest_distance:
                        smallest_distance = intersection[0]
                        shoot = foe[:2]
        return shoot, turn


    def can_hit_target(self, edge_distance, target_angle, target_coords, center=False):
        """ Check if you can hit a target at a given distance, with a certain
            angle.  
        """
        # Check if edge can be hit
        can_hit = True
        # Check for distance
        if edge_distance > self.settings.max_range and not center:
            can_hit = False
        # Check for angle
        if abs(target_angle) > self.settings.max_turn:
            can_hit = False
        # Check for walls
        if can_hit and line_intersects_grid(self.obs.loc, target_coords, self.grid, self.settings.tilesize):
            can_hit = False
        # Check for friendly fire
        for friendly in self.obs.friends:
            if can_hit and line_intersects_circ(self.obs.loc, target_coords, friendly, Agent.RADIUS*1.05):
                can_hit = False
        return can_hit

    
    def calc_optimal_angle(self, left_angle, right_angle, interval, path_angle):
        """This function returns the optimal angle given some (interval of) angles
        """
        optimal_angle = 0
              
        # If two possible angles, see which one is in turning range and closest to path angle
        if not interval:            
            if (abs(path_angle - min_angle) > abs(path_angle - max_angle)):
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
    
    def compute_speed(self, turn, old_path_angle, distance):
        """ Compute speed based on angle with planned path and planned distance
        """
        if turn > self.settings.max_turn:
            turn = self.settings.max_turn
        elif turn < -self.settings.max_turn:
            turn = -self.settings.max_turn
        path_angle = angle_fix(old_path_angle - turn)
        
        if abs(path_angle) >= math.pi/2:
            speed = 0
        
        # If agent can at least face partly in the right direction, move some fraction of required distance
        elif abs(path_angle) > 0.0 and abs(path_angle) < math.pi/2:
            # Cap distance at 30 when not facing exactly in the right direction
            if distance > 30:
                distance = 30
            # Scale distance by how well the agent can move in the right direction
            speed = distance*(1-(abs(path_angle)/(math.pi/2)))
        
        # If agent can reduce angle to zero, move the required distance
        else:
            speed = distance
        
        return speed

    def defend(self):
        """ Returns the turn an agent should to to defend a point on the map
        """
        # Determine new angle the agent should be at
        new_angle = 0.0
        
        # If no foes are nearby, just face the direction of the other team's base
        if (self.team == TEAM_RED):
            new_angle = 0.0
        else:
            new_angle = -math.pi
        
        # For the foes that either
        #     - have a line of sight and can reach you within 3 steps, or
        #     - can reach you with the least steps,
        # face the foe that requires the least turning to face.
        req_turn = 2*math.pi
        foe_path = {}
        for foe in self.joint_observation.foes[self.joint_observation.step]:
            foe_path[foe] = find_single_path(foe[:2], foe[2], self.obs.loc, self.mesh, self.grid,
                    self.settings.max_speed, self.settings.max_turn, self.settings.tilesize)
        foe_path_ = sorted(foe_path.items(), key=lambda item: len(item[1]))
        nearest_foes = filter(lambda item: item[1] == foe_path_[0][1], foe_path)
        dangerous_foe = []
        for foe in self.joint_observation.foes[self.joint_observation.step]:
            if not line_intersects_grid(self.obs.loc, foe[0:2], self.grid, self.settings.tilesize):
                if len(foe_path[foe]) < 3:
                    dangerous_foe.append(foe)
        for foe in set(dict(nearest_foes).keys()).union(dangerous_foe):
            dx = foe[0] - self.obs.loc[0]
            dy = foe[1] - self.obs.loc[1]
            foe_angle = math.atan2(dy, dx)
            req_turn_foe = angle_fix(foe_angle - self.obs.angle)
            
            if abs(req_turn_foe) < abs(req_turn):
                req_turn = req_turn_foe
                new_angle = foe_angle 
        
        # Calculate how the agent should turn to face the new angle
        return angle_fix(new_angle - self.obs.angle)

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
        font = pygame.font.SysFont("monospace", 10)
        if self.id == 0:
            surface.fill((0,0,0,0))    

            if DRAW_MESH_POINTS:
                for point in self.mesh:
                    if point in self.sharia_point:
                        pygame.draw.circle(surface, (169, 32, 62), point, 3)
                    elif point in self.haram_point:
                        pygame.draw.circle(surface, (236, 242, 69), point, 3)
                    else:
                        pygame.draw.circle(surface, (255, 128, 0), point, 3)
                    if DRAW_MESH_POINT_COORDS:
                        label = font.render(str(point), True, (0,0,0))
                        label_pos = (point[0] - label.get_width()/2., point[1] + 5)
                        #surface.blit(label, label_pos)
                        
            if DRAW_PF_ALL:
                for enemy_pf, color in zip(self.joint_observation.mc_points, [(169, 32, 62), (236, 242, 69), (255, 128, 0)]):
                    for point in enemy_pf:
                        pygame.draw.circle(surface, color, (int(point[0]), int(point[1])), 1)
        
        # Details of agents are drawn in their respective colors:
        #     Agent 0 = red
        #     Agent 1 = green
        #     Agent 2 = blue
        # Each agent has a dot on top to mark it with its own color
        color = ((200, 0, 0), (0, 200, 0), (0, 0, 200))
        agent_color = color[self.id % len(color)]
        if DRAW_AGENT_DOT or self.selected:
            pygame.draw.circle(surface, agent_color, self.obs.loc, 1)
        # This section draws its viewing range
        if DRAW_AGENT_VIEW or self.selected:
            view_range = self.settings.max_see
            rect = pygame.Rect(self.obs.loc[0] - view_range, 
                    self.obs.loc[1] - view_range, 
                    2 * view_range, 
                    2 * view_range)
            pygame.draw.rect(surface, agent_color, rect, 1)
        if DRAW_AGENT_AMMO or self.selected:
            # Write a small ammo counter south of the agent (in black for readability)
            label = font.render(str(self.obs.ammo), True, (0, 0, 0))
            label_pos = (self.obs.loc[0] - label.get_width()/2., self.obs.loc[1] + 5)
            surface.blit(label, label_pos)

        # Draw line to goal along the planned path
        if self.goal is not None and (DRAW_AGENT_GOAL or self.selected):
            path = None
            for ip in Agent.INTEREST_POINTS:
                if self.goal == Agent.INTEREST_POINTS[ip]:
                    path = self.joint_observation.paths[self.id][ip][0]
            if path is None:
                # Compute path and angle to path
                path = find_single_path(self.obs.loc, self.obs.angle, self.goal, self.mesh, self.grid,
                                    self.settings.max_speed, self.settings.max_turn, self.settings.tilesize)
            if path:
                for i in range(len(path)):
                    if i == 0:
                        pygame.draw.line(surface, agent_color, self.obs.loc, path[i][0:2])
                    else:
                        pygame.draw.line(surface, agent_color, path[i-1][0:2], path[i][0:2])

        # Selected agents draw their info
        if self.selected:
            pass
    
    def finalize(self, interrupted=False):
        """ This function is called after the game ends, 
            either due to time/score limits, or due to an
            interrupt (CTRL+C) by the user. Use it to
            store any learned variables and write logs/reports.
        """
        if self.id == 0 and hasattr(self, 'blobpath') and self.blobpath is not None:
            try:
                # We simply write the same content back into the blob.
                # in a real situation, the new blob would include updates to 
                # your learned data.
                blobfile = open(self.blobpath, 'wb')
                pickle.dump(self.joint_observation.state_action_pairs, blobfile, pickle.HIGHEST_PROTOCOL)
                print "Blob saved.\n"
            except:
                # We can't write to the blob, this is normal on AppEngine since
                # we don't have filesystem access there.        
                print "%s can't write blob." % self.id


########################################################################
##### Pathfinding functions ############################################
########################################################################

def transform_mesh(nav_mesh, interest_points, grid, tilesize, max_speed=40, max_angle=math.pi/4):
    """ Recomputes a new weight for each edge of the navigation mesh.
        
        Each start and end point of an edge and its weight is given to
        mesh_transform, and its return value is stored in a new mesh.
    """
    
    # Add interest points to mesh
    for point in interest_points:
        nav_mesh[interest_points[point]] = 0
    
    # Interconnect all points that may be connected in the original mesh
    full_mesh = {}
    for start in nav_mesh:
        full_mesh[start] = {}
        for end in nav_mesh:
            if not line_intersects_grid(start, end, grid, tilesize):
                full_mesh[start][end] = calc_cost(start, end)
    
    ridiculous_lines = {
            (57, 89): [(232, 56)],
            (87, 89): [(232, 56), (184, 168)],
            (57, 183): [(264, 216), (184, 168), (295, 151), (201, 151), (121,
                169)],
            (87, 183): [(295, 151)],
            (439, 183): [(264, 216)],
            (409, 183): [(264, 216)],
            (409, 89): [(312, 104), (201, 121)],
            (439, 89): [(232, 56), (312, 104), (201, 121), (295, 121), (375,
                103)]}
    for start in ridiculous_lines:
        for end in ridiculous_lines[start]:
            try:
                del full_mesh[start][end]
            except KeyError:
                pass
            try:
                del full_mesh[end][start]
            except KeyError:
                pass

    # Add in angles and calculate new cost for transitions
    #new_mesh = {}
    #angles = list(frange(-math.pi, math.pi, max_angle*(8/4.0)))
    #for start in full_mesh:
    #    for angle in angles:
    #        start_ = start + (angle,)
    #        new_mesh[start_] = {}
    #        for angle_ in angles:
    #            for end in full_mesh[start]:
    #                new_mesh[start_][end+(angle_,)] = calc_cost(start_,end+(angle_,),max_speed,max_angle)
    
    return full_mesh

def calc_cost(node1, node2,  max_speed=40, max_angle=math.pi/4):
    #Calculate the turns necessary to travel from node1 to node2
 
    # Calculate the distance between the two nodes
    dx = node2[0] - node1[0]
    dy = node2[1] - node1[1]
    dist = ((dx**2 + dy**2)**0.5 / max_speed)
    
    # If no angle is specified, just return the distance in turns
    if len(node1) == 2 and len(node2) == 2:
        return dist
        
    # If angle is specified for the first node, calculate the turn cost
    angle1_weight = 0.0
    angle1_dist = 0.0
    if len(node1) == 3:
        angle1_dist = (abs(angle_fix(math.atan2(dy,dx) - node1[2])) / max_angle) * angle1_weight

    # If angle is specified for the second node, calculate the turn cost
    angle2_weight = 0.0
    angle2_dist = 0.0
    if len(node2) == 3:
        angle2_dist = (abs(angle_fix(node2[2] - math.atan2(dy,dx))) / max_angle) * angle2_weight
    
    # Then calculate the travel cost
    if angle1_dist < 1.0:
        return dist + angle2_dist
    else:
        return dist + (angle1_dist - 1.0) + angle2_dist    

def find_single_path(start, angle, end, mesh, grid, max_speed=40, max_angle=math.pi/4, tilesize=16):
    """ Uses astar to find a path from start to end,
        using the given mesh and tile grid.
    """
    # If there is a straight line, just return the end point
    if not line_intersects_grid(start, end, grid, tilesize):
        return [end]
    
    # Add temp nodes for end:
    if not end in mesh:
        endconns = [(n, calc_cost(n,end,max_speed,max_angle)) for n in mesh 
                    if not line_intersects_grid(n[0:2],end,grid,tilesize)]
        for n, cost in endconns:
            mesh[n][end] = cost
    
    # Add temp nodes for start
    start_list = []
    start_angle = start+(angle,)
    for n in mesh:
        if not start == n and line_intersects_grid(start,n[0:2],grid,tilesize):
            start_list.append((n, calc_cost(start_angle,n,max_speed,max_angle)))
    if start_angle not in mesh:
        remove_start_node = True
    else:
        remove_start_node = False
    mesh[start_angle] = dict(start_list)
    
    # Plan path
    neighbours = lambda n: mesh[n].keys()
    cost       = lambda n1, n2: mesh[n1][n2]
    goal       = lambda n: n == end
    heuristic  = lambda n: ((n[0]-end[0])**2 + (n[1]-end[1])**2)**0.5 / max_speed
    nodes, length = astar(start_angle, neighbours, goal, 0, cost, heuristic)

    # Remove temp nodes for start and end from mesh
    if remove_start_node:
        del mesh[start_angle]
    if not end in mesh:
        for n in mesh:
            if mesh[n].has_key(end):
                del mesh[n][end]

    # Return path
    return nodes

def find_all_paths(start, angle, ends, mesh, grid, max_speed=40, max_angle=math.pi/4, tilesize=16):
    """ Uses astar to find a path from start to each point in end,
        using the given mesh and tile grid.
    """
    
    # Add temp nodes for start
    start_list = []
    start_angle = start+(angle,)
    for n in mesh:
        if not n == start and not line_intersects_grid(start,n[0:2],grid,tilesize):
            start_list.append((n, calc_cost(start_angle,n,max_speed,max_angle)))
    if start_angle not in mesh:
        remove_start_node = True
    else:
        remove_start_node = False
    mesh[start_angle] = dict(start_list)
    
    path_dict = {}
    for end_point in ends:
        
        if type(ends) is dict:
            end = ends[end_point]
        else:
            end = end_point
        
        # If there is a straight line, just return the end point
        if not line_intersects_grid(start, end, grid, tilesize):
            path_dict[end_point] = ([end], calc_cost(start_angle,end,max_speed,max_angle))
        else:
            # Plan path
            neighbours = lambda n: mesh[n].keys()
            cost       = lambda n1, n2: mesh[n1][n2]
            goal       = lambda n: n == end+(0.0,)
            heuristic  = lambda n: ((n[0]-end[0])**2 + (n[1]-end[1])**2)**0.5 / max_speed
            nodes, length = astar(start_angle, neighbours, goal, 0, cost, heuristic)
            
            # Save path
            path_dict[end_point] = (nodes, length)
        
    # Remove temp nodes for start
    if remove_start_node:
        del mesh[start_angle]

    # Return path dictionary
    return path_dict


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


########################################################################
##### JointObservation class ###########################################
########################################################################

class JointObservation(object):
    """ A singleton object representing the joint observation of all our
        agents. It should be updated during Agent.observe.
    """
    __metaclass__ = Singleton
    ascii_red = """
                        +                             .                         
  M                     .M                           I.                     M   
  MM                     MM                         MM.                    MM   
  NMM.                   DMM.                      MMD                    MMN   
  .MMM                    MMM                     MMM,                  ,MMM.   
  .MMMMN                 .MMMM..                 MMMM.                 IMMMM.   
   MMMMMM                .MMMMMI.               MMMMM                 MMMMMM.   
   DMMMMMM                MMMMMMO              MMMMMD                MMMMMMM    
    MMMMMMM.              :MMMMMMM           .MMMMMM.               MMMMMMMM    
    MMMMMMMM.             .MMMMMMMM.       .,MMMMMMM.             .MMMMMMMM$    
    MMMMMMMMMM. .         .MMMMMMMMMMMMMMMMMMMMMMMMM.          .:MMMMMMMMMM.    
    MMMMMMMMMMMMMMMZ..     $MMMMMMMMMMMMMMMMMMMMMMMM     .. 8MMMMMMMMMMMMMM.    
    ~MMMMMMMMMMMMMMMMMMMN   MMMMMMMMMMMMMMMMMMMMMMM=  .7MMMMMMMMMMMMMMMMMMM     
     MMMMMMMMMMMMMMMMMMMMMM.MMMMMMMMMMMMMMMMMMMMMMM. MMMMMMMMMMMMMMMMMMMMMN     
     MMMMMMMMMMMMMMMMMMMMMM.MMMMMMMMMMMMMMMMMMMMMMM OMMMMMMMMMMMMMMMMMMMMM.     
     MMMMMMMMMMMMMMMMMMMMMM.$MMMMMMMMMMMMMMMMMMMMMM MMMMMMMMMMMMMMMMMMMMMM      
     .MMMMMMMMMMMMMMMMMMMMMD MMMMMMMMZ    MMMMMMMM: MMMMMMMMMMMMMMMMMMMMMM      
     .MMMMMMMMO  .  MMMMMMMM.MMMMMMMMM   .MMMMMMMM. MMMMMMMM ... DMMMMMMMM      
      MMMMMMMMMMMZ. .  ..MMM MMMMMMMMM.  MMMMMMMMM.$MM:.  .. .DMMMMMMMMMM8      
      MMMMMMMMMMMMMMMMO.    .=MMMMMMMMM .MMMMMMMMM.    .NMMMMMMMMMMMMMMMM,      
      IMMMMMMMMMMMMMMMMMMMM8. MMMMMMMMM.MMMMMMMMMI.MMMMMMMMMMMMMMMMMMMMMM.      
      .MMMMMMMMMMMMMMMMMMMMMM.MMMMMMMMMMMMMMMMMMM.,MMMMMMMMMMMMMMMMMMMMMM       
       MMMMMMMMMMMMMMMMMMMMMM.MMMMMMMMMMMMMMMMMMM.NMMMMMMMMMMMMMMMMMMMMMM       
       MMMMMMMMM.. OMMMMMMMMM.+MMMMMMMMMMMMMMMMMM MMMMMMMMMM ..MMMMMMMMM~       
       ,MMMMMMMMMMMM....+MMMM8.MMMMMMMMMMMMMMMMM, MMMMM .  MMMMMMMMMMMMM.       
        MMMMMMMMMMMMMMMM7   .   MMMMMMMMMMMMMMM$ :.   ~MMMMMMMMMMMMMMMMM.       
        MMMMMMMMMMMMMMMMMMMMM+ ..MMMMMMMMMMMMM.. .MMMMMMMMMMMMMMMMMMMMM,        
        .MMMMMMMMMMMMMMMMMMMMMMM. MMMMMMMMMMM. MMMMMMMMMMMMMMMMMMMMMMMO         
          MMMMMMMMMMMMMMMMMMMMMMM8.OMMMMMMMM .MMMMMMMMMMMMMMMMMMMMMMMM          
        . .MMMMMMMMIMMMMMMMMMMMMMMM $MMMMMM. MMMMMMMMMMMMMMMMMMMMMMMN           
        M. .MMMMMMMM    MMMMMMMMMMMM..MMMM.NMMMMMMMMMMMMM= MMMMMMMMM..M         
        .M  IMMMMMMMM     ..MMMMMMMMM. MO.MMMMMMMMM.      MMMMMMMMM. M.         
        .MO .MMMMMMMMM       MMMMMMMMM,  MMMMMMMMM       MMMMMMMMM. MM.         
        .MMI  MMMMMMMMM.      MMMMMMMMMMMMMMMMMMMD    ..MMMMMMMMM  +MM.         
         MMM.  MMMMMMMMMMM.    MMMMMMMMMMMMMMMMMM    8MMMMMMMMMM  ,MMM.         
          MMM.  MMMMMMMMMMMMM. NMMMMMMMMMMMMMMMM   MMMMMMMMMMMM?. MMMM          
          MMMM  .MMMMMMMMMMMMMMOMMMMMMMMMMMMMMMM:MMMMMMMMMMMMMD. MMMM7          
         .MMMMM  .MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM  MMMMM           
         .MMMMMM  7MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM .MMMMMM           
          8MMMMMM  8MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM .MMMMMMM           
           MMMMMMN. MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM .7MMMMMMM           
          .MMMMMMM,  MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM. ,MMMMMMM,           
           MMMMMMMM.  MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM.  MMMMMMMM            
           DMMMMMMMM. .MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM, .MMMMMMMMM            
           .MMMMMMMMM.  MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM  MMMMMMMMMM.           
           .MMMMMMMMMM. =MMMMMMMMMMMMMMMMMMMMMMMMMMMMMM  MMMMMMMMMMM            
           .MMMMMMMMMMM..8MMMMMMMMMMMMMMMMMMMMMMMMMMMM  NMMMMMMMMMMI            
            MMMMMMMMMMMM .MMMMMMMMMMMMMMMMMMMMMMMMMMM  =MMMMMMMMMMM.            
            ~MMMMMMMMMMM7  MMMMMMMMMMMMMMMMMMMMMMMMM  :MMMMMMMMMMMM.            
            .MMMMMMMMMMMM= .MMMMMMMMMMMMMMMMMMMMMMM  .MMMMMMMMMMMMM             
            .MMMMMMMMMMMMM  .MMMMMMMMMMMMMMMMMMMMM. .MMMMMMMMMMMMMM             
             MMMMMMMMMMMMMM.  MMMMMMMMMMMMMMMMMMM=. MMMMMMMMMMMMMM.             
             ~MMMMMMMMMMMMMM .+MMMMMMMMMMMMMMMMMI. MMMMMMMMMMMMMMM.             
              MMMMMMMMMMMMMMM  $MMMMMMMMMMMMMMMM  MMMMMMMMMMMMMMMM              
                 MMMMMMMMMMMMM  MMMMMMMMMMMMMMM. IMMMMMMMMMMMMMM..              
                   :MMMMMMMMMMO  MMMMMMMMMMMMM  :MMMMMMMMMMMM+..                
                     .MMMMMMMMM7  MMMMMMMMMMM ..MMMMMMMMMM~                     
                       . MMMMMMM.  MMMMMMMMM   MMMMMMMM~                        
                            MMMMM...MMMMMMM   MMMMMM..                          
                              .MMM  .MMMMM   MMMM.                              
                                  M   MMMZ .NM~                                 
                                      :M8  ..                                   
                                       .
"""
    ascii_blue = """
                          ZZZZZZZZZZZZZZZZZZZZZZZZZZZ.                          
                    ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ                     
    .ZZZZZZZZZ  ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ  ZZZZZZZZZ?     
     ZZZZZZZZZZ ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ ZZZZZZZZZZ      
     ZZZZZZZZZZ ?ZZZZZZZZZZZZZZZZZZZZ:   .ZZZZZZZZZZZZZZZZZZZZZ ZZZZZZZZZZ      
      ZZZZZZZZZZ ZZZZZZZZZZZZZZ.                ZZZZZZZZZZZZZZ  ZZZZZZZZZ       
      ZZZZZZZZZZ =ZZZZZZZZZZZZZZZZZ         ZZZZZZZZZZZZZZZZZZ ZZZZZZZZZZ       
      ZZZZZZZZZZZ   7ZZZZZZZZZZZZZZZZZ  .ZZZZZZZZZZZZZZZZZZ   7ZZZZZZZZZZ       
       ZZZZZ+ZZZZZZZ    ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ    ZZZZZZZ7ZZZZZ        
       ZZZZZZ   ZZZZZZZZ    ZZZZZZZZZZZZZZZZZZZZZZZ    ZZZZZZZZ   $ZZZZZ        
       IZZZZZZZZ   ,ZZZZZZZ.    ZZZZZZZZZZZZZZZ     ZZZZZZZZ   $ZZZZZZZZ        
        ZZZZZZZZZZZ   .ZZZZZZZ,    ,ZZZZZZZZ     ZZZZZZZ=   ZZZZZZZZZZZ+        
        ZZZZZZ=ZZZZZZZ,   ZZZZZ ZZZ    =    ZZZ ZZZZZ   .ZZZZZZZZZZZZZZ         
        ZZZZZZ~   ZZZZZZZZ ZZZZ  ZZZZZZ ZZZZZZ. ZZZZIZZZZZZZZ    ZZZZZZ         
         ZZZZZZZZ=   ZZZZZZZZZZZ ZZZZZZZZZZZZZ  ZZZZZZZZZZ   ,ZZZZZZZZ          
         ZZZZZZZZZZZZ   ,ZZZZZZZ ZZZZZZZZZZZZZ ZZZZZZZ=   ZZZZZZZZZZZZ          
           ZZZZZZZZZZZZZ=  ~ZZZZ  ZZZZZZZZZZZ  ZZZZZ  .ZZZZZZZZZZZZZ            
          Z  ZZZZZZZZZZZZZZZZZZZ  ZZZZZZZZZZZ  ZZZZ.ZZZZZZZZZZZZZZ  Z           
          ZZZ  ZZZZZZZZZZZZZZZZZZ ZZZZZZZZZZZ ZZZZZZZZZZZZZZZZZZ  ZZZ           
          ZZZZZ  ZZZZZZZZZZZZZZZZ  ZZZZZZZZZ  ZZZZZZZZZZZZZZZZ  ZZZZZ           
          ZZZZZZ                   ZZZZZZZZZ                   ZZZZZZ           
          ZZZZZZ                   ZZZZZZZZZ                   ZZZZZZ           
          ZZZZZZZ                  ZZZZZZZZZ                  ZZZZZZZ           
          :ZZZZZZ               ZZ ZZZZZZZZZ ZZ               ZZZZZZZ           
           ZZZZZZZZZ          ZZZZ ZZZZZZZZZ ZZZZ          7ZZZZZZZZ:           
           ZZZZZZZZZZZZ     ZZZZZZ ZZZZZZZZZ ZZZZZZ     ZZZZZZZZZZZZ            
           ZZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZZ            
           ZZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZZ            
           ZZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZZ            
           ZZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZZ            
           ?ZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZZ            
            ZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZZ            
            ZZZZZZZZZZZZZ= ZZZZZZZ ZZZZZZZZZ ZZZZZZZ  ZZZZZZZZZZZZZ=            
            ZZZZZZZZZZZZZ= ZZZZZZZ           ZZZZZZZ  ZZZZZZZZZZZZZ             
            ZZZZZZZZZZZZZ= ZZZZZZZZZZZZZZZZZZZZZZZZZ  ZZZZZZZZZZZZZ             
            ZZZZZZZZZZZZZ= ZZZZZZZZZZZZZZZZZZZZZZZZZ  ZZZZZZZZZZZZZ             
            ZZZZZZZZZZZZZ= ZZZZZZZZZZZZZZZZZZZZZZZZZ  ZZZZZZZZZZZZZ             
             ZZZZZZZZZZZZ= ZZZZZZ             ZZZZZZ  ZZZZZZZZZZZZ              
                .ZZZZZZZZ= ZZZZZZ +ZZZZZZZZZZ ZZZZZZ  ZZZZZZZZ,                 
                    ZZZZZ= ZZZZZZ ZZZZZZZZZZZ ZZZZZZ  ZZZZZ
                       =Z= ZZZZZZ ZZZZZZZZZZZ ZZZZZZ  ZZ
                           ZZZZZ  ZZZZZZZZZZZ  ZZZZZ
                               Z ,ZZZZZZZZZZZZ Z
                                 ZZZZZZZZZZZZZ
"""
    
    def __init__(self, settings, grid, team, nav_mesh, epsilon, gamma, alpha, deltaWin, deltaLose, useWoLF, initial_value, number_of_agents, interest_pts):
        # printing out the proper transformer logo
        if team == TEAM_BLUE:
            print JointObservation.ascii_blue
        else:
            print JointObservation.ascii_red
        
        # Game constants
        self.interest_points = interest_pts
        self.settings = settings
        self.team = team        
        self.grid = grid
        self.mesh = transform_mesh(nav_mesh, interest_pts, grid, settings.tilesize, settings.max_speed, settings.max_turn)
        self.number_of_agents = number_of_agents
        
        # State space value representation
        self.state_action_pairs = {}
        self.WoLF_policy = {}
        self.avg_policy = {}
        
        # Learning constants
        self.epsilon = epsilon
        self.gamma = gamma
        self.alpha = alpha
        self.deltaWin = deltaWin
        self.deltaLose = deltaLose
        self.useWoLF = useWoLF
        self.initial_value = initial_value
        
        # Learning variables
        self.state = -1
        self.new_state_key = -1
        self.old_state_key = -1
        self.locked_agent = [False] * self.number_of_agents
        self.new_joint_action = -1
        self.old_joint_action = -1
        self.available_joint_actions = []
        self.old_available_joint_actions = []
        self.reward = 0

        # Regions of interest; placeholders for the cp and ammo points in the current learning setting
        self.ROI = {"cp": (2, 12), "am": (6,8)}
 
        # coordinate list [2, 6, 8, 12]
        self.coordlist = [(232, 56), (184, 168), (312, 104), (264, 216)]
        
        # Switch the regions around when we start on the other side of the screen
        if self.team == TEAM_BLUE:
            self.coordlist.reverse()

        self.coords = {2: self.coordlist[0], 6: self.coordlist[1], 8: self.coordlist[2], 12: self.coordlist[3]}

        # The game settings (not needed right now)
        self.settings = settings

        self.step = -1 # current timestep

        # Per-agent data, stored in a namedtuple.
        self.friends = {} # agent_id: (x, y, angle, ammo, collided, respawn_in, hit)

        # Keeps track of the position of foe positions sorted per timestep
        self.foes = {} # step: {(x, y, angle), ...}

        # Lists all control point positions, and which team is controlling them.
        self.cps = {} # (x, y, dominating_team)

        # Objects seen by the agents, together with the last time seen.
        self.objects = defaultdict(lambda: [-1, -1]) # (x, y, type): last_seen, disappeared_since

        # Walls that can be seen by the agents
        self.walls = defaultdict(set) # (x, y): {agent_id, ...}

        # Current game score, and the difference with the score at the last
        # timestep, as a discretized derivative
        self.score = (0, 0)
        self.diff_score = (0, 0)

        # Keep track of the agents who have passed their observation in this timestep
        self.called_agents = set()

        # Create a list of all possible joint actions
        interestRegions = self.ROI["cp"] + self.ROI["am"]
        self.joint_actions = list(product(interestRegions, repeat=self.number_of_agents))

        # Keep track of paths to interest points for each agent at each timestep
        self.paths = {} # agent_id: {'cp1':(path,length), ..}
        # Keep track of each agent's goal
        self.goals = [] # [agent0_goal, agent1_goal, ...]
        # Keep track of actions from other agents during each timestep
        self.actions = {} # agent_id: (turn, speed, shoot)

        # Set enemy spawn points
        if self.team == TEAM_BLUE:
            self.enemy_spawn_pos = [(24,120), (24,136), (24,142)]
        else:
            self.enemy_spawn_pos = [(472,120), (472,136), (427,142)]
        
        # Monte Carlo localisation to track enemy positions
        self.mc_count = 1000
        self.foe_ids = {} # {id1:loc1, ...}
        self.mc_points = self.init_MC_points() # [[loc1,loc2,loc3...], ...]
        self.mc_probs = self.init_MC_probs() # [{tile1 : prob1, tile2 : prob2, ...}, ...]

    def update(self, agent_id, observation):
        """ Update the joint observation with a single agent's observation
            information.
        """
        
        # Initialize to empty when the first agent calls update
        if self.step != observation.step:
            self.step = observation.step
            # Empty collections as neccesary 
            self.foes[self.step] = set()
            self.called_agents = set()
            self.featuresExtracted = False
            self.chosen_goals = {}
            self.paths = {}

        # Unlocks an agent's action if it has reached it's objective or if it is respawning
        if (not self.old_joint_action == -1):
            objective = self.coords[self.old_joint_action[agent_id]]
            if (point_dist(objective, observation.loc) < self.settings.tilesize) or observation.respawn_in > 0:
                #print "Agent " + str(agent_id) + " unlocked."
                self.locked_agent[agent_id] = False
        
        # Fill friends with current agent data
        self.friends[agent_id] = AgentData(observation.loc[0], observation.loc[1],
                observation.angle, observation.ammo, observation.collided,
                observation.respawn_in, observation.hit)
        
        # add foes to the joint observation
        for foe in observation.foes:
            self.foes[self.step].add(foe)
            
        # copy over the control point observations
        self.cp = observation.cps
        
        # if object is in vision range, then add it to the object list
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
        
        # add walls to the joint observation
        walls_as_tuples = [tuple(w) for w in observation.walls]
        for wall in set(self.walls.keys() + walls_as_tuples):
            if wall in observation.walls:
                self.walls[wall].add(agent_id)
            else:
                try:
                    self.walls[wall].remove(agent_id)
                except KeyError:
                    pass
        # the difference in score for both teams between the current and last time step
        self.diff_score = (observation.score[0] - self.score[0],
                           observation.score[1] - self.score[1])

        # checks if all agents have been called
        self.called_agents.add(agent_id)
        if len(self.called_agents) == self.number_of_agents:
            # once all agents have been called, the joint observation can be processed into a state
            self.state = self.process_joint_observation() # Process the joint observation
            self.new_state_key = self.state.toKey() # key used to lookup in the state action dict
            self.chooseJointAction() # choose a new joint action
            self.setReward() # determine the reward for the current state
            if self.old_state_key != -1:
                self.update_policy() # Update policy when joint observation has been processed

            # Remember the values of the current state as values of previous state
            self.old_state_key = self.new_state_key
            self.old_joint_action = self.new_joint_action
            self.old_available_joint_actions = self.available_joint_actions

        # find all paths for the current agent to every interest point
        self.paths[agent_id] = find_all_paths(observation.loc, observation.angle, self.interest_points, 
                                self.mesh, self.grid, self.settings.max_speed, self.settings.max_turn,
                                self.settings.tilesize)
        
        # Perform Monte Carlo localisation
        if agent_id == (self.number_of_agents - 1):
            self.MC_localisation()
    def update_action(self, agent_id, action_tuple):
        """ Register the chosen action of agents.  This information can be
            retrieved through methods such as:
              - JointObservation.targeted_foes() -> list of foe locations
            for agents to decide which action to take.
        """
        if agent_id == (self.number_of_agents - 1):
            self.actions = {}
        else:
            self.actions[agent_id] = action_tuple

    def goal_chosen(self, goal):
        """ Check if an other agent is moving towards the same goal.
        """
        for agent in self.goals:
            if self.goals[agent] == goal:
                return True
        return False
    
    def targeted_foes(self):
        """ A list of foes that are targeted to be shot.
        """
        return filter(lambda x: type(x[2]) is tuple, self.actions.values())


    def chooseJointAction(self):
        """ Used with wolf and Sarsa.
            Choose the next joint action out of the available actions.        
        """

        # Update old_joint_action
        self.old_joint_action = self.new_joint_action

        # brute force acquisition of available actions
        self.available_joint_actions = []
        # if every agent is available, make every action available
        if self.locked_agent == [False] * self.number_of_agents:
            self.available_joint_actions = list(self.joint_actions);
        else:
            # if not, then see which agents are locked, and only add actions that correspond to the old action
            for action in self.joint_actions:
                add_action = True
                for agent_id in sorted(self.friends):
                    if not self.old_joint_action == -1:
                        if self.locked_agent[agent_id] and action[agent_id] != self.old_joint_action[agent_id]:
                            add_action = False
                            break
                if add_action:
                    self.available_joint_actions.append(action)

        # Update Q values.
        # If the state key is not recognized in the state action pairs,
        # then the value of that key is set to the initial value.
        try:
            action_value_dict = self.state_action_pairs[self.new_state_key]
            # print "action_value_dict: " + str(action_value_dict)
        except KeyError:
            self.state_action_pairs[self.new_state_key] = {}
            for action in self.joint_actions:
                self.state_action_pairs[self.new_state_key][action] = self.initial_value
            # print "Added actions: " + str(self.available_joint_actions)
            action_value_dict = self.state_action_pairs[self.new_state_key]

        joint_action = -1

        # If WoLF is not used, simply choose the epsilon greedy action based on the Q values.
        if (not self.useWoLF):
            # Epsilon greedy algorithm
            if randint(1,100) * 0.01 >= self.epsilon:
                # Epsilon random action
                joint_action = random.choice(self.available_joint_actions)
            else:
                # Greedy action
                max_val = -1000
                for action in self.available_joint_actions:
                    try:
                        value = action_value_dict[action]
                    except KeyError:
                        value = self.initial_value
                    if value > max_val:
                        max_val = value
                        self.new_joint_action = action

        # If WoLF is used, update an explicit policy and base action on this policy.
        else:
            try:
                policy_action_probability_dict = self.WoLF_policy[self.new_state_key]
            except KeyError:
                self.WoLF_policy[self.new_state_key] = {}
                for action in self.joint_actions:
                    self.WoLF_policy[self.new_state_key][action] = 1.0/len(self.joint_actions)
                policy_action_probability_dict = self.WoLF_policy[self.new_state_key]

            # Epsilon greedy algorithm
            if randint(1,100) * 0.01 <= self.epsilon:
                # Epsilon random action
                joint_action = random.choice(self.available_joint_actions)
            else:
            # Sample a joint action based on the action probability distribution in the explicit policy.
                threshold_prob = randint(0,999) * 0.001
                cumulative_prob = 0
                total_prob = 0.000001  # Use an almost zero value in order to evade divide by zero errors. 
                                       # Using log-probs would be better, but for now this works fine.
                for action in self.available_joint_actions:
                    total_prob += policy_action_probability_dict[action]
                for action in self.available_joint_actions:
                    cumulative_prob += (policy_action_probability_dict[action] / total_prob)
                    # print "THRESHOLD VALUE: " + str(threshold_prob)
                    # print "TOTAL VALUE: " + str(cumulative_prob)
                    if threshold_prob <= cumulative_prob:
                        self.new_joint_action = action
                        break

        # update old values
        if self.old_joint_action == -1:
            self.old_joint_action = self.new_joint_action
        # once a joint action is picked, all agent's actions should be locked.
        self.locked_agent = [True] * self.number_of_agents

    
    def setReward(self):
        """ Set the reward for the current state
        """
        difference = self.diff_score[0] if self.team == TEAM_RED else self.diff_score[1]
        
        if difference > 0:
            reward = difference
        else:
            reward = 0
            for agent_id, agentRegion in zip(sorted(self.friends), sorted(self.state.locations.values())):
                if self.friends[agent_id].ammo > 0 and (agentRegion == 2 or agentRegion == 12):
                    reward += 1
                if self.friends[agent_id].ammo == 0 and (agentRegion == 2 or agentRegion == 12):
                    reward -= 1

        self.reward = reward


    def update_policy(self):
        """ Update the Q values (and possibly explicit policy when using WoLF).
        """


        # Get the value of the current state-action pair
        cur_val = self.state_action_pairs[self.new_state_key][self.new_joint_action]
     
        # Get the value of the previous state-action pair
        old_val = self.state_action_pairs[self.old_state_key][self.old_joint_action]
        
        # Calculate the new value of the previous state-action pair
        new_val = old_val + self.alpha * (self.reward + self.gamma*cur_val - old_val)
        
        # Update the value of the state-action pair
        self.state_action_pairs[self.old_state_key][self.old_joint_action] = new_val


        # Update the explicit agent policy when using WoLF
        if self.useWoLF == True:

            # Get the actions and probabilities of the WoLF policy for the previous state.
            prev_q_values = self.state_action_pairs[self.old_state_key]
            prev_state_policy = self.WoLF_policy[self.old_state_key]

            # Calculate the total probability of all available actions in previous state.
            # Also create a policy sub-dictionary for all available actions in previous state.
            available_action_prob_sum = 0
            prev_state_policy_subdict = {}
            prev_q_values_subdict = {}
            for action in self.old_available_joint_actions:
                prev_q_values_subdict[action] = prev_q_values[action]
                available_action_prob_sum += prev_state_policy[action]
                prev_state_policy_subdict[action] = prev_state_policy[action]
            # Compute average Q-value for all actions for this state
            # Find the the action yielding the highest Q value for the previous state
            max_action_value = -1
            max_action = -1
            for action, value in prev_q_values_subdict.iteritems():
                if value > max_action_value:
                    max_action = action
                    max_action_value = value


            # Compute the average policy
            average_policy = 0
            for action, probability in prev_state_policy_subdict.iteritems():
                average_policy += ((1.0/self.step) * (probability - average_policy))

            #Compute the sum of rewards when following the current policy and when following the average policy. 
            reward_sum_current_policy = 0
            reward_sum_average_policy = 0
            for action, probability in prev_state_policy_subdict.iteritems():
                reward_sum_current_policy += (probability * prev_q_values_subdict[action])
                reward_sum_average_policy += (average_policy * prev_q_values_subdict[action])

            # Determine the value of delta based on the performance of the two policies
            if reward_sum_current_policy > reward_sum_average_policy:
                delta = self.deltaWin
            else:
                delta = self.deltaLose

            # Compute if the update should be contrainted to keep a correct probability distribution.
            constrained_update = False
            for action, probability in prev_state_policy_subdict.iteritems():
                if action == max_action:
                    if (self.WoLF_policy[self.old_state_key][action] + delta) > available_action_prob_sum:
                        constrained_update = True
                    break

            # Weight delta based on the probability sum of all available joint actions.
            weighted_delta = delta * available_action_prob_sum

            # Do a normal update
            if constrained_update == False:
                # Update policy for each available state-action pair
                for action, probability in prev_state_policy_subdict.iteritems():
                    if action == max_action:
                        self.WoLF_policy[self.old_state_key][action] += weighted_delta
                    else:
                        self.WoLF_policy[self.old_state_key][action] -= (weighted_delta/(len(prev_state_policy_subdict)-1))
            # Or do a contrained update
            else:
                for action, probability in prev_state_policy_subdict.iteritems():
                    if action == max_action:
                        self.WoLF_policy[self.old_state_key][action] = available_action_prob_sum
                    else:
                        self.WoLF_policy[self.old_state_key][action] = 0


    def process_joint_observation(self):
        """ Creates an abstract representation of the observation, on which we will learn.
        """
        # Initialize as empty state
        state = State()

        # Create the location based on the path distances to all interest points.
        for id, agent_paths in self.paths.iteritems():
            sorted_goal_list = []
            for goal, path_info in agent_paths.iteritems():
                path_length = path_info[1]
                sorted_goal_list.append((path_length, goal))
            sorted_goal_list.sort()

            # Get the nearest cp and am elements
            cp_distance = 0
            cp = ""
            am_distance = 0 
            am = ""
            for distance, cp in sorted_goal_list:
                if cp[:2] == "cp":
                    cp_distance = distance
                    break

            for distance, am in sorted_goal_list:
                if am[:2] == "am":
                    am_distance = distance
                    break
            
            # Add tuple to locations to state
            if cp_distance < am_distance:
                state.locations[id] = (cp, am)
            else:
                state.locations[id] = (am, cp)


        # grabs the information from self.friends that is relevant to the state space
        for key, val in self.friends.iteritems():
            state.respawning[key]   = False if val.respawn_in <= 0 else True
            state.has_ammo[key]     = False if val.ammo == 0 else True
        
        # Represent the status of the control points
        cp_amount = len(self.cps)
        for cpx, cpy, val in sorted(self.cps.iteritems()):
            if val == self.team:
                state.control_points[(cpx, cpy)] = True
            else:
                state.control_points[(cpx, cpy)] = False

        return state
    
    ########################################################################
    ##### Code for MC Localisation #########################################
    ########################################################################
    
    def init_MC_points(self):
        """Initialise Monte Carlo points for each foe at their start position
        """
        mc_points = []
        for pos in self.enemy_spawn_pos:
            mc_points.append(self.init_MC_points_single(pos))
        return mc_points
        
    def init_MC_points_single(self, loc):
        return [loc] * self.mc_count

    def init_MC_probs(self):
        mc_probs = []
        for i in range(self.number_of_agents):
            mc_probs.append(self.init_MC_probs_single())
        return mc_probs
        
    def init_MC_probs_single(self):
        mc_probs = []
        tile_dict = {}
        
        for x in range(16,480,16):
            for y in range(16,272,16):
                tile_dict[(x,y)] = 0.0
        
        return tile_dict

    def MC_localisation(self):
        """Perform Monte Carlo localisation
        """
        #print self.foe_ids
        #for id in range(len(self.mc_points)):
        #    print len(self.mc_points[id])
        
        # Calculate which foes are observed
        foes_gone = self.update_foe_ids()
        
        # Update MC points
        if self.step > 1:
            for id in range(self.number_of_agents):
                # If foe is observed, store a single MC point
                if id in self.foe_ids:
                    self.mc_points[id] = [self.foe_ids[id]]
                # If foe is just gone, initialise MC points at previous location
                elif id in foes_gone:
                    if self.check_vision(foes_gone[id]):
                        self.mc_points[id] = self.init_MC_points_single(self.enemy_spawn_pos[id])
                    else:
                        self.mc_points[id] = self.init_MC_points_single(foes_gone[id])
                # If foe is still not observed, spread around the MC points
                else:
                    self.mc_points[id] = self.update_MC_points(id)
        
        # Update tile probabilities
        for id in range(self.number_of_agents):
            self.mc_probs[id] = self.update_MC_probs(id)
    
    def update_foe_ids(self):
        """Updates the ids of observed foes
        """
        new_foe_ids = {}
        
        # Get positions of foes at current timestep
        foes_new = list(self.foes[self.step])
        
        # If no foes observed at new timestep, return empty dictionary
        if len(foes_new) == 0:
            pass
        
        # If same number of foes is observed, assume these are the same foes
        elif len(foes_new) == len(self.foe_ids):
            count = 0
            for id in self.foe_ids:
                new_foe_ids[id] = foes_new[count][0:2]
                count += 1
        
        # If more foes are observed, assume the same foes are present, and more
        elif len(foes_new) > len(self.foe_ids):

            # For each old foe calculate the closest new foe
            assigned = []
            for foe_old in self.foe_ids:
                min_dist = float('inf')
                min_id = -1
                foe_old_loc = self.foe_ids[foe_old]
                
                for foe_new in foes_new:
                    if not foe_new in assigned:
                        dist = point_dist(foe_old_loc, foe_new[0:2])
                        if dist < min_dist:
                            min_dist = dist
                            min_id = foe_new
                assigned.append(min_id)
                new_foe_ids[foe_old] = min_id[0:2]
            
            # Get the list of foes that were not observed before
            foes_really_new = []
            for foe in foes_new:
                if not foe in assigned:
                    foes_really_new.append(foe)
            
            # For each new foe that was not observed before, calculate the appropriate id
            assigned_new = []
            for foe in foes_really_new:
                tile = self.get_tile_coords(foe[0:2])
                max_prob = -1.0
                max_id = -1
                
                for id in range(len(self.mc_probs)):
                    if (not id in self.foe_ids) and (not id in assigned_new):
                        prob = self.mc_probs[id][tile]
                        if prob > max_prob:
                            max_prob = prob
                            max_id = id
                new_foe_ids[max_id] = foe[0:2]
                assigned_new.append(max_id)
            
        # If fewer foes are observed, compute which agent is gone
        else:
            # For each new foe calculate the closest old foe
            assigned = []
            for foe_new in foes_new:
                min_dist = float('inf')
                min_id = -1
                for foe_old in self.foe_ids:
                    if not foe_old in assigned:
                        dist = point_dist(foe_new[0:2], self.foe_ids[foe_old])
                        if dist < min_dist:
                            min_dist = dist
                            min_id = foe_old
                assigned.append(min_id)
                new_foe_ids[min_id] = foe_new[0:2]
        
        # Determine which foes are no longer observed compared to previous time step
        foes_gone = {}
        for id in self.foe_ids:
            if id not in new_foe_ids:
                foes_gone[id] = self.foe_ids[id]
                
        # Update and return
        self.foe_ids = new_foe_ids
        return foes_gone
    
    def update_MC_points(self, id):
        new_points = []
        for point in self.mc_points[id]:
            
            # If the point is at an interest point, there is a 50% chance it will not move
            at_ip = False
            for ip in self.interest_points:
                if point_dist(point, self.interest_points[ip]) < self.settings.tilesize:
                    at_ip = True
            
            if at_ip and random.uniform(0,1) < 0.5:
                new_point = point
            
            # Else look which mesh points are reachable from the current point and move towards one randomly
            else:
                reachable_mesh_pts = []
                for n in self.mesh:
                    if not line_intersects_grid(point,n,self.grid,self.settings.tilesize):
                        reachable_mesh_pts.append(n)
                
                if len(reachable_mesh_pts) == 0:
                    new_point = point
                else:
                    target = reachable_mesh_pts[random.randrange(len(reachable_mesh_pts))]
                    dx = target[0] - point[0]
                    dy = target[1] - point[1]
                    x_offset = min(self.settings.max_speed, dx) if dx > 0 else max(-self.settings.max_speed, dx)
                    y_offset = min(self.settings.max_speed, dy) if dy > 0 else max(-self.settings.max_speed, dy)

                    new_x= point[0] + x_offset
                    new_y = point[1] + y_offset
                    new_point = (new_x, new_y)
                
            # Check if point is in current line of sight
            in_vision = self.check_vision(new_point)
            
            # Check if point is on the map
            #in_bounds = self.check_bounds(new_point)
            
            # Check if point is reachable from old point
            reachable = not line_intersects_grid(point, new_point, self.grid, 16)
            
            # Add new point to the list
            if not in_vision and reachable: # and in_bounds and reachable:
                new_points.append(new_point)
        return new_points

    def check_vision(self, point):
        """Checks whether a point is in vision of at least one agent
        """
        for id in self.friends:
            if (point[0] >= (self.friends[id][0] - self.settings.max_see)
                and point[0] <= (self.friends[id][0] + self.settings.max_see)
                and point[1] >= (self.friends[id][1] - self.settings.max_see)
                and point[1] <= (self.friends[id][1] + self.settings.max_see)):
                return True
        return False

    def check_bounds(self, point):
        """Checks whether a point is within bounds of the field
        """
        if point[0] >= 464 or point[0] < 16:
            return False
        elif point[1] >= 256 or point[1] < 16:
            return False
        else:
            return True

    def update_MC_probs(self, id):
        """Calculate new probabilities using the updated MC points
        """
        new_MC_probs = self.init_MC_probs_single()
        
        for point in self.mc_points[id]:
            tile = self.get_tile_coords(point)
            new_MC_probs[tile] = new_MC_probs[tile] + 1.0/len(self.mc_points[id])
        
        return new_MC_probs
        
    def get_tile_coords(self, pos):
        x = int(math.floor(pos[0]/16.0)*16.0)
        y = int(math.floor(pos[1]/16.0)*16.0)
        
        return (x,y)

    def correct_tile_coords(self, coords):
        
        if coords[0] > 464:
            x = 464
        elif coords[0] < 16:
            x = 16
        else:
            x = coords[0]
        
        if coords[1] > 256:
            y = 256
        elif coords[1] < 16:
            y = 16
        else:
            y = coords[1]
        
        return (x,y)

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

        self.locations = defaultdict(tuple)
        self.respawning = defaultdict(bool)
        self.has_ammo = defaultdict(bool)
        self.control_points = defaultdict(bool)

    def toKey(self):
        key = str(self.locations) + str(self.respawning) + str(self.has_ammo) + str(self.control_points)
        return key


class NamedInt(int):
    """ Show an Autobot or Decepticon themed name, depending on the team.

    This will be printed instead of the agent id.
    """
    NAMES = ["Megatron", "Starscream", "Blitzwing",
             "Optimus Prime", "Bumblebee", "Bulkhead"]
    def __init__(self, *args, **kwargs):
        int.__init__(self, *args, **kwargs)

    def __str__(self):
        return NamedInt.NAMES[self % len(NamedInt.NAMES)]
