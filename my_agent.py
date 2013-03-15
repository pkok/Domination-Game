from collections import defaultdict, namedtuple
from itertools import product
from random import randint, choice
import math
import sys
import time
import cPickle as pickle

from domination.libs import astar
astar = astar.astar


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
        
        self.id = NamedInt(id).set_team(team)
        self.team = team
        self.settings = settings
        # self.state_action_pairs = defaultdict(lambda: [None, 10])
        self.epsilon = 0.05
        self.gamma = 0.9
        self.alpha = 0.1
        self.initial_value = 10
        self.number_of_agents = 3
        # self.joint_actions = createJointActions(self.joint_observation) # Fill the joint_actions object with all possible joint actions.
        self.joint_observation = JointObservation(settings, field_grid, team, nav_mesh, self.epsilon, self.gamma, self.alpha, self.initial_value, self.number_of_agents, Agent.INTEREST_POINTS)
        # self.joint_action = (2,2,6) #random.choice(joint_actions) # Initial random joint action for step 1.
        self.mesh = self.joint_observation.mesh
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

        jo = self.joint_observation

        # If the agent is not agent 0, its goal has already been computed
        if self.id != 0:
            self.goal = jo.goals[self.id]
            return
        
        # Intinialise some handy variables
        goals = [(0,0), (0,0), (0.0)]
        assigned = []       
        ammo_chart = {}
        for agent_id, agent in jo.friends.items():
            if agent[3] not in ammo_chart:
                ammo_chart[agent[3]] = [agent_id]
            else:
                ammo_chart[agent[3]].append(agent_id)
        have_ammo = [] #join ammo_chart[key] for key > 1
        for agent_id, agent in jo.friends.items():
            if agent[3] > 1:
                have_ammo.append(agent_id)
        
        cp_data = namedtuple("cp_data", location, distance)
        ammo_data = namedtuple("ammo_data", location, distance, is_present, timer)
        cps = [cp_data(Agent.INTEREST_POINTS['cp1'], []), 
               cp_data(Agent.INTEREST_POINTS['cp2'], [])]
        ammos = [ammo_data(Agent.INTEREST_POINTS['am1'], [], False, 0), 
                 ammo_data(Agent.INTEREST_POINTS['am2'], [], False, 0)]
        """
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
        """
        
        for obj in jo.objects:
            for ammo in ammos:
                if obj[:2] == ammo.location:
                    ammo_timer = jo.step - jo.objects[obj][0]
                    if ammo.timer != 0:
                        ammo.timer = max(0, self.settings.ammo_rate - ammo.timer + 1)
                    else:
                        ammo.is_present = True
                    break

        for id in jo.friends:
            # Add respawn timer to distance calculation
            if jo.respawn_in[id] > 0:
                respawn_time = jo.respawn_in[id]
            else:
                respawn_time = 0
            # Calculate distances
            cps[0].distance.append(jo.paths[id]['cp1'][1] + respawn_time)
            cps[1].distance.append(jo.paths[id]['cp2'][1] + respawn_time)
            ammos[0].distance.append(jo.paths[id]['am1'][1] + respawn_time)
            ammos[1].distance.append(jo.paths[id]['am2'][1] + respawn_time)
        
        team = int(self.team == TEAM_BLUE)
        controlling_cps = map(lambda cp: cp[2] == team, self.obs.cps)

        """
        if all(controlling_cps):
            danger_zone = min([self.settings.max_range,
                self.settings.max_speed])
            def cp_under_pressure(cp):
                for foe in jo.foes[jo.step]:
                    dx = foe[0] - cps[0]
                    dy = foe[1] - cps[1]
                    distance = (dx**2 + dy**2) ** 0.5
                    if distance < danger_zone:
                        return True
                return False
            safe_cps = filter(lambda x: not cp_under_pressure(x), self.obs.cps)
            if safe_cps:
                cp_dists = []
                if cps[0].location in safe_cps:
                    cp_dists += map(lambda x: (x[1], x[0], cps[0],), enumerate(cps[0].distance))
                if cps[1].location in safe_cps:
                    cp_dists += map(lambda x: (x[1], x[0], cps[1],), enumerate(cps[1].distance))
                # Sort the cp distances on the distance per agent.  Each item
                # is formatted according to (distance, agent_id, cp)
                sort(cp_dists)
                cp_dists = filter(lambda x: x[0] == cp_dists[0][0], cp_dists)
                nearest = random.choice(cp_dists)
                chosen_agent = jo.friends[nearest[1]]
                chosen_cp = nearest[2].location
                cp_distance = nearest[0]

                dx = ammos[0].location[0] - chosen_agent[0]
                dy = ammos[0].location[1] - chosen_agent[1]
                distance_am1 = (dx**2 + dy**2) ** 0.5
                dx = ammos[1].location[0] - chosen_agent[0]
                dy = ammos[1].location[1] - chosen_agent[1]
                distance_am2 = (dx**2 + dy**2) ** 0.5
                if distance_am1 < distance_am2: 
                    chosen_am = ammos[0].location
                else:
                    chosen_am = ammos[1].location

                # Agents not "associated" with a CP
                selectable_agents = filter(lambda friend: 
                        friend not in nearby_agent_id.values(), 
                        jo.friends)
                if len(selectable_agents) >= 1:
                    # Select the nearest to the CP
                    # Move this one to the CP
                    # Move chosen_agent to the chosen_am
                    #return after setting the goals, with commands such as...
                    #     self.joint_observation.goals = goals
                    #     self.goal = goals[self.id]
                    pass
        """

        #If no agent has ammo, follow this policy
        if len(have_ammo) == 0:
            
            # Send agent closest to cp1 to cp1
            min_dist, min_id = float("inf"), 0
            for id in jo.friends:
                if cps[0].distance[id] < min_dist:
                    min_dist = cps[0].distance[id]
                    min_id = id
            goals[min_id] = cps[0].location
            assigned.append(min_id)
            
            # Send agent closest to cp2 to cp2
            min_dist, min_id = float("inf"), 0
            for id in jo.friends:
                if (id not in assigned) and cps[1].distance[id] < min_dist:
                    min_dist = cps[1].distance[id]
                    min_id = id
            goals[min_id] = cps[1].location
            assigned.append(min_id)

            # Send the other agent to collect ammo
            for id in jo.friends:
                if id not in assigned:
                    # Go to closest ammo
                    if am1 and am2:
                        goals[id] = ammos[0].location if (ammos[0].distance[id] < ammos[1].distance[id]) else ammos[1].location
                    elif am1:
                        goals[id] = ammos[0].location
                    elif am2:
                        goals[id] = ammos[1].location
                    else:
                        am1_delay = max(ammos[0].distance[id], am1_timer)
                        am2_delay = max(ammos[1].distance[id], am2_timer)
                        
                        if am1_delay < am2_delay:
                            goals[id] = ammos[0].location
                        else:
                            goals[id] = ammos[1].location
                assigned.append(id)

        # If one agent has ammo, follow this policy
        elif len(have_ammo) == 1:
            # Send the agent with ammo to nearest uncontrolled cp, and one to the other cp, and one for ammo

            # Assign the agent with ammo to the nearest cp
            ammo_id = have_ammo[0]
            goals[ammo_id] = cps[0].location if (cps[0].distance[ammo_id] < cps[1].distance[ammo_id]) else cps[1].location
            assigned.append(ammo_id)
            
            # Send agent closest to other cp to other cp
            if cps[0].location in goals:
                other_cp_dist, other_cp_loc = cps[1].distance, cps[1].location
            else:
                other_cp_dist, other_cp_loc = cps[0].distance, cps[0].location
                
            min_dist, min_id = float("inf"), 0
            for id in jo.friends:
                if (id not in assigned) and other_cp_dist[id] < min_dist:
                    min_dist = other_cp_dist[id]
                    min_id = id
            goals[min_id] = other_cp_loc
            assigned.append(min_id)
            
            # Send other agent to collect ammo
            for id in jo.friends:
                if id not in assigned:
                    # Go to closest ammo
                    if am1 and am2:
                        goals[id] = ammos[0].location if (ammos[0].distance[id] < ammos[1].distance[id]) else ammos[1].location
                    elif am1:
                        goals[id] = ammos[0].location
                    elif am2:
                        goals[id] = ammos[1].location
                    else:
                        am1_delay = max(ammos[0].distance[id], am1_timer)
                        am2_delay = max(ammos[1].distance[id], am2_timer)
                        
                        if am1_delay < am2_delay:
                            goals[id] = ammos[0].location
                        else:
                            goals[id] = ammos[1].location
                assigned.append(id)
        
        # If two agents have ammo, follow this policy
        elif len(have_ammo) == 2:
            # Both agents with ammo go for a closest control point each, the other gets ammo
            
            # Assign the agent with ammo to the nearest cp
            ammo_id = have_ammo[0]
            goals[ammo_id] = cps[0].location if (cps[0].distance[ammo_id] < cps[1].distance[ammo_id]) else cps[1].location
            assigned.append(ammo_id)
            
            # Send agent with ammo closest to cp1 to cp1
            min_dist, min_id = float("inf"), 0
            for id in have_ammo:
                if cps[0].distance[id] < min_dist:
                    min_dist = cps[0].distance[id]
                    min_id = id
            goals[min_id] = cps[0].location
            assigned.append(min_id)
        
            # Send other agent with ammo to other cp
            for id in have_ammo:
                if id not in assigned:
                    goals[id] = cps[1].location if (cps[0].location in goals) else cps[0].location
            
            # Send other agent to get ammo
            for id in jo.friends:
                if id not in assigned:
                    # Go to closest ammo
                    if am1 and am2:
                        goals[id] = ammos[0].location if (ammos[0].distance[id] < ammos[1].distance[id]) else ammos[1].location
                    elif am1:
                        goals[id] = ammos[0].location
                    elif am2:
                        goals[id] = ammos[1].location
                    else:
                        am1_delay = max(ammos[0].distance[id], am1_timer)
                        am2_delay = max(ammos[1].distance[id], am2_timer)
                        
                        if am1_delay < am2_delay:
                            goals[id] = ammos[0].location
                        else:
                            goals[id] = ammos[1].location
                assigned.append(id)
            
        # If three agents have ammo, and at least one cp is uncontrolled, follow this policy
        elif len(have_ammo) == 3:
            # Send two agents to one unheld cp, and one agent to the other cp
            
            # Send agent closest to cp1 to cp1
            min_dist, min_id = float("inf"), 0
            for id in jo.friends:
                if cps[0].distance[id] < min_dist:
                    min_dist = cps[0].distance[id]
                    min_id = id
            goals[min_id] = cps[0].location
            assigned.append(min_id)
            
            # Send agent closest to cp2 to cp2
            min_dist, min_id = float("inf"), 0
            for id in jo.friends:
                if (id not in assigned) and cps[1].distance[id] < min_dist:
                    min_dist = cps[1].distance[id]
                    min_id = id
            goals[min_id] = cps[1].location
            assigned.append(min_id)
            
            # If at least one cp is uncontrolled, send other agent to closest cp
            if not all(controlling_cps):
                for id in jo.friends:
                    if id not in assigned:
                        goals[id] = cps[0].location if (cps[0].distance[id] < cps[1].distance[id]) else cps[1].location
                        assigned.append(id)
        
            # If both cps are controlled, send other agent to ammo spawn closest to enemy base
            else:
                for id in jo.friends:
                    if id not in assigned:
                        goals[id] = ammos[0].location if (self.team == TEAM_BLUE) else ammos[1].location
                        assigned.append(id)
        
        # Set the joint goal
        jo.goals = goals
        
        # Instantiate goal for agent 0
        self.goal = goals[0]
    
    def get_action(self):
        """This function returns the action tuple for the agent.
        """
        path = []
        for ip in Agent.INTEREST_POINTS:
            if self.goal == Agent.INTEREST_POINTS[ip]:
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
            if self.allow_reverse_gear(path, path_angle, path_reverse_angle):
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


    def allow_reverse_gear(self, path, forward_angle, reverse_angle):
        """ Is it safe/smart to drive backwards?
        """
        angle_threshold = 0
        #angle_threshold = 1 * math.pi / 180
        return abs(reverse_angle) < (abs(forward_angle) + angle_threshold)
    
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
        if self.id == 0:
            surface.fill((0,0,0,0))    
        
        # Draw a red, green and blue dot on agent 1, 2, and 3.
        color = ((255,0,0), (0,255,0), (0,0,255))
        agent_color = color[self.id % len(color)]
        pygame.draw.circle(surface, agent_color, self.obs.loc, 1)

        # Draw line to goal along the planned path
        if self.goal is not None:
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

        if self.selected:
            # Handle selected agents specially
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
        end = ends[end_point]
        
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

    def __init__(self, settings, grid, team, nav_mesh, epsilon, gamma, alpha, initial_value, number_of_agents, interest_pts):
        if team == TEAM_BLUE:
            print JointObservation.ascii_blue
        else:
            print JointObservation.ascii_red
        self.interest_points = interest_pts
        self.team = team        
        self.grid = grid
        self.mesh = transform_mesh(nav_mesh, interest_pts, grid, settings.tilesize, settings.max_speed, settings.max_turn)
        self.state_action_pairs = {}
        self.epsilon = epsilon
        self.gamma = gamma
        self.alpha = alpha
        self.initial_value = initial_value
        self.number_of_agents = number_of_agents

        self.state = -1

        self.new_state_key = -1
        self.old_state_key = -1
        self.locked_agent = [False] * self.number_of_agents
        self.new_joint_action = -1
        self.old_joint_action = -1
        self.reward = 0

        # TODO: use interest points instead of regions

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
        self.coordlist = [(232, 56), (184, 168), (312, 104), (264, 216)]
        
        # Switch the regions around when we start on the other side of the screen
        if self.team == TEAM_BLUE:
            self.regions.reverse()
            self.coordlist.reverse()

        self.coords = {2: self.coordlist[0], 6: self.coordlist[1], 8: self.coordlist[2], 12: self.coordlist[3]}
        
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

        # TODO: joint actions should be interest points, not regions.
        # Create a list of all possible joint actions
        interestRegions = self.ROI["cp"] + self.ROI["am"]
        self.joint_actions = list(product(interestRegions, repeat=self.number_of_agents))
        
        # Keep track of paths to interest points for each agent at each timestep
        self.paths = {} # agent_id: {'cp1':(path,length), ..}
        # Keep track of each agent's goal
        self.goals = [] # [agent0_goal, agent1_goal, ...]
        # Keep track of actions from other agents during each timestep
        self.actions = {} # agent_id: (turn, speed, shoot)

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
            self.chosen_goals = {}
            self.paths = {}

        # check if the agent has reached it's goal or died
        # if so then reset the lock on that agent
        if (not self.old_joint_action == -1):
            passed_hor = False
            passed_ver = False
            objective = self.coords[self.old_joint_action[agent_id]]
            #case 1: bot is left of objective, and is to the right afterwards or equal to the position
            if (self.friends[agent_id].x <= objective[0] +3 and objective[0] -3 <= observation.loc[0]):
                passed_hor = True
            #case 2: vice versa
            elif(self.friends[agent_id].x >= objective[0] -3 and objective[0] +3 >= observation.loc[0]):
                passed_hor = True
            #case 3: bot is to the top of objective, and is to the bottom or equal afterwards
            if (self.friends[agent_id].y <= objective[1] +3 and objective[1] -3 <= observation.loc[1]):
                passed_ver = True
            #case 4: vice versa
            elif (self.friends[agent_id].y >= objective[1] -3 and objective[1] +3 >= observation.loc[1]):
                passed_ver = True
            if (passed_ver and passed_hor) or observation.respawn_in > 0:
                print "%s unlocked." % agent_id
                self.locked_agent[agent_id] = False
            
        self.friends[agent_id] = AgentData(observation.loc[0], observation.loc[1],
                observation.angle, observation.ammo, observation.collided,
                observation.respawn_in, observation.hit)
        for foe in observation.foes:
            self.foes[self.step].add(foe)
        self.cp = observation.cps
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
                try:
                    self.walls[wall].remove(agent_id)
                except KeyError:
                    pass
        self.diff_score = (observation.score[0] - self.score[0],
                           observation.score[1] - self.score[1])
        self.ammo[agent_id] = observation.ammo
        self.collided[agent_id] = observation.collided
        self.respawn_in[agent_id] = observation.respawn_in
        self.hit[agent_id] = observation.hit

        
        
        self.called_agents.add(agent_id)
        if len(self.called_agents) == self.number_of_agents:
            self.state = self.process_joint_observation() # Process the joint observation
            self.new_state_key = self.state.toKey()
            self.chooseJointAction()
            self.setReward()

            if self.old_state_key != -1:
                self.update_policy(self.old_state_key, self.new_state_key) # Update policy when joint observation has been processed
        
        self.paths[agent_id] = find_all_paths(observation.loc, observation.angle, self.interest_points, 
                                self.mesh, self.grid, self.settings.max_speed, self.settings.max_turn,
                                self.settings.tilesize)
                                
        
        

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
        # if self.state_action_pairs[self.new_state_key] == {}:
        #      for action in self.joint_action:
        #          self.state_action_pairs[self.new_state_key][action] = self.initial_value

        try:
            action_value_dict = self.state_action_pairs[self.new_state_key]
        except KeyError:
            self.state_action_pairs[self.new_state_key] = {}
            for action in self.joint_actions:
                self.state_action_pairs[self.new_state_key][action] = self.initial_value
                action_value_dict = self.state_action_pairs[self.new_state_key]
        # # Initialize all state-action pairs with the initial value if this state is encoutered for the first time.
        # if action_value_dict == {}:
        #     for action in self.joint_actions:
        #         action_value_dict[action] = self.initial_value
        # print "action_value_dict: "+ str(action_value_dict)
        
        # brute force acquisition of available actions
        available_joint_actions = []
        # if every agent is available, make every action available
        if self.locked_agent == [False] * self.number_of_agents:
            available_joint_actions = list(self.joint_actions);
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
                    available_joint_actions.append(action)
        
        
        
        if randint(1,100) * 0.01 >= self.epsilon:
            joint_action = random.choice(available_joint_actions)
        else:
            max_val = -1000
            joint_action = -1
            for action in available_joint_actions:
                value = action_value_dict[action]
                if value > max_val:
                    max_val = value
                    joint_action = action
        self.new_joint_action = joint_action
        if self.old_joint_action == -1:
            self.old_joint_action = joint_action
        # once a joint action is picked, all agents should be locked.
        self.locked_agent = [True] * self.number_of_agents

    
    def setReward(self):
        difference = self.diff_score[0] if self.team == TEAM_RED else self.diff_score[1]
        
        if difference > 0:
            reward = difference
        else:
            reward = 0
            for agent_id, agentRegion in zip(sorted(self.friends), sorted(self.state.locations.values())):
                if self.ammo[agent_id] > 0 and (agentRegion == 2 or agentRegion == 12):
                    reward += 1
                if self.ammo[agent_id] == 0 and (agentRegion == 2 or agentRegion == 12):
                    reward -= 1

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
            #state.locations[key]    = in_region(self, val.x, val.y)
            #state.orientations[key] = angle_to_wd(val.angle)
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
    NAMES = {TEAM_RED:  ["Megatron", "Starscream", "Blitzwing"],
             TEAM_BLUE: ["Optimus Prime", "Bumblebee", "Bulkhead"]}
    def __init__(self, *args, **kwargs):
        int.__init__(self, *args, **kwargs)

    def set_team(self, team):
        self.team = team
        return self

    def __str__(self):
        return NamedInt.NAMES[self.team][self % len(NamedInt.NAMES[self.team])]
