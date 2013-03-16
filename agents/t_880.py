# Paste your code.
import math
import domination
import cmath
import string
from domination.libs import astar
astar = astar.astar
MAP = """
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
w _ _ _ _ _ _ _ _ _ _ _ _ _ D _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ N _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ w w w w w w w w w w w w w w w _ _ _ _ _ _ _ w
w _ _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ A _ _ _ _ _ _ w _ _ _ w
w R _ _ w _ _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ B w
w R _ _ w _ _ _ w _ _ _ _ w w w w w _ _ _ _ w _ _ _ w _ _ B w
w R _ _ w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ B w
w _ _ _ w _ _ _ _ _ _ A _ _ _ _ _ _ _ _ _ _ w _ _ _ w _ _ _ w
w _ _ _ _ _ _ _ w w w w w w w w w w w w w w w _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ N _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ C _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ w
w _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ D _ _ _ _ _ _ _ _ _ _ _ _ _ w
w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w w
"""
class CpData:
    def __init__(self, label, loc, owned):
        self.label = label
        self.loc = loc
        self.owned = owned
class ApData:
    def __init__(self, label, loc, hasAmmo, turn):
        self.label = label
        self.loc = loc
        self.hasAmmo = hasAmmo
        self.hasAmmoUpdated = turn
class SpawnData:
    def __init__(self, label, loc):
        self.label = label
        self.loc = loc
class FoeData:
    def __init__(self, grid, settings, label, loc, angle, turn):
        self.label = label
        self.settings = settings
        self.grid = grid
        self.oldLoc = None
        self.oldLocUpdated = turn
        self.loc = loc
        self.locUpdated = turn
        self.angle = angle
        self.angleUpdated = turn
        self.threat = 0
        self.threatUpdated = turn
        self.predictedTarget = None
        self.predictedTargetUpdated = turn
    def incThreat(self, turn):
        self.threat = min(self.threat + 1, 4)
        self.threatUpdated = turn
    def setLoc(self, loc, angle, turn):
        oldLoc = self.loc
        self.loc = loc
        self.locUpdated = turn
        self.angle = angle
        self.angleUpdated = turn
        if ( self.oldLocUpdated == turn - 1 ):
            self.oldLoc = oldLoc
        else:
            self.oldLoc = None
        self.oldLocUpdated = turn
    def predictTarget(self, turn):
        if ( self.predictedTargetUpdated == turn ):
            return
        if ( self.locUpdated == turn and self.oldLocUpdated == turn and self.oldLoc != None ):
            self.predictedTarget = None
            maxR = max(self.settings.max_range, self.settings.max_speed)
            theta = self.settings.max_turn
            objects = Agent.cps
            for l, a in Agent.aps.items():
                if ( a.hasAmmo ):
                    objects[l] = a
            for l, f in Agent.friends.items():
                objects[l] = f
            for l, o in objects.items():
                k = o.loc
                if line_intersects_grid(k, self.loc, self.grid, self.settings.tilesize):
                    continue
                objectPos = complex(k[0] - self.loc[0], k[1] - self.loc[1])
                (objR, objPhi) = cmath.polar(objectPos)    
                if ( objR < maxR and objPhi > self.angle - theta and objPhi < self.angle + theta ):
                    if (self.predictedTarget == None or objR < minR):
                        self.predictedTarget = l
                        minR = objR
            self.predictedTargetUpdated = turn
class Agent(object):
    NAME = "T-880"
    mesh = {}
    lineMesh = {}
    cps = {}
    aps = {}
    rspawns={}
    bspawns={}
    dninjas = {}
    ninjas = {}
    foeNameCounter = 0
    foes = {}
    friends = {}
    helps = []
    jobs = []
    width = 0
    widthConservative = 0
    teabagged = False
    @staticmethod
    def linkWithFoeLabel(foe):
        matches = []
        for l, oFoe in Agent.foes.items():
            if point_dist((foe[0], foe[1]), oFoe.loc) > oFoe.settings.max_speed:
                continue
            phi = math.atan2((foe[1]-oFoe.loc[1]),float((foe[0]-oFoe.loc[0])))
            eps = 0.1
            if inCone(foe[2], eps, phi) == 0:
                matches.append(l)
        return matches
    @staticmethod
    def updateFoes(observedFoes, settings, turn, grid):
        possibleFoes = []
        for observedFoe in observedFoes: 
            possibleFoes.append(Agent.linkWithFoeLabel(observedFoe))
        labelsDone = []
        newFoes = []
        size = len(possibleFoes) + 1
        while size != len(possibleFoes):
            size = len(possibleFoes)
            k = 0
            while k < len(possibleFoes):
                observedFoe = observedFoes[k]
                matchingFoes = possibleFoes[k]
                for label in labelsDone:
                    if label in matchingFoes:
                        matchingFoes.remove(label)
                if len(matchingFoes) == 0:
                    foeLabel = "FO" + str(Agent.foeNameCounter)
                    Agent.foeNameCounter = Agent.foeNameCounter + 1
                    newFoes.append(FoeData(label=foeLabel,loc=(observedFoe[0], observedFoe[1]),angle = observedFoe[2],turn = turn, settings = settings, grid = grid))
                    del possibleFoes[k]
                    del observedFoes[k]
                    continue
                if ( len(matchingFoes) == 1 ):
                    Agent.foes[matchingFoes[0]].setLoc((observedFoe[0], observedFoe[1]),observedFoe[2],turn)
                    labelsDone.append(matchingFoes[0])
                    del possibleFoes[k]
                    del observedFoes[k]
                else:
                    k = k+1
        for foe in newFoes:
            Agent.foes[foe.label] = foe
    @staticmethod
    def findCp(loc):
        for l, cp in Agent.cps.items():
            if loc[0] <= cp.loc[0] + 8 and loc[0] >= cp.loc[0] - 8 \
            and loc[1] <= cp.loc[1] + 8 and loc[1] >= cp.loc[1] - 8:
                return l
    @staticmethod
    def findAp(loc):
        for l, ap in Agent.aps.items():
            if loc[0] <= ap.loc[0] + 8 and loc[0] >= ap.loc[0] - 8 \
            and loc[1] <= ap.loc[1] + 8 and loc[1] >= ap.loc[1] - 8:
                return l
    @staticmethod
    def updateCps(cps):
        for cp in cps:
            cpLabel = Agent.findCp( (cp[0],cp[1]) )
            Agent.cps[cpLabel].owned = cp[2]
    @staticmethod
    def updateAps(seeDistance, agentLoc, aps, turn):
        for ap in Agent.aps:
            if Agent.aps[ap].hasAmmoUpdated != turn and point_dist(agentLoc, Agent.aps[ap].loc) < seeDistance:
                Agent.aps[ap].hasAmmo = False
                for apo in aps:
                    if Agent.aps[ap].loc[0] == apo[0] and Agent.aps[ap].loc[1] == apo[1]:
                        Agent.aps[ap].hasAmmo = True
                Agent.aps[ap].hasAmmoUpdated = turn
    def __init__(self, id, team, settings=None, field_rects=None, field_grid=None, nav_mesh=None, blob=None, matchinfo = None, **kwargs):
        self.id = id
        self.team = team
        self.mesh = nav_mesh
        self.grid = field_grid
        self.settings = settings
        self.goal = None
        self.currentGoal = None
        self.callsign = '%s-%d'% (('BLU' if team == TEAM_BLUE else 'RED'), id)
        self.shootedGuy = False
        self.oldTarget = None
        self.path = None
        self.length = None
        self.direct = None
        if self.id == 0:
            extractCoords(MAP, self.settings.tilesize)
            (Agent.mesh, Agent.lineMesh) = setupMesh(nav_mesh, self.grid, self.settings.tilesize, self.settings.max_turn, self.settings.max_speed)
            Agent.jobs = ["Def"+self.getRelativeMap("CP1"), "AmmoColl", "Def"+self.getRelativeMap("CP2")]
            Agent.width = (2**0.5)*self.settings.tilesize
            Agent.widthConservative = self.settings.tilesize - 5.0
            self.label = "SK0"
        elif self.id == 1:
            self.label = "SK1"
        else:
            self.label = "SK2"
        self.currentBehaviour = Agent.jobs[self.id]
        self.currentTBehaviour = None
        self.currentSecondaryBehaviour = None
        self.currentSecondaryTarget = None
    def observe(self, observation):
        self.observation = observation
        self.selected = observation.selected
        self.loc = self.observation.loc
        self.angle = self.observation.angle
        if self.id == 0:
            Agent.friends = {}
            Agent.observedFoes = []
        Agent.friends["SK"+str(self.id)] = self
        for o in self.observation.foes:
            if o not in Agent.observedFoes:
                Agent.observedFoes.append(o)
        if self.id == 0:
            Agent.updateCps(self.observation.cps)
        if self.currentBehaviour == "AmmoColl":
            if self.team == TEAM_RED:
                if self.observation.hit == TEAM_BLUE:
                    self.shootedGuy = True
            else:
                if self.observation.hit == TEAM_RED:
                    self.shootedGuy = True
        Agent.updateAps(self.settings.max_see, self.loc, filter(lambda x: x[2] == "Ammo", self.observation.objects), self.observation.step)
    def action(self):
        if self.id == 0:
            Agent.updateFoes(observedFoes = Agent.observedFoes, turn = self.observation.step, settings = self.settings, grid = self.grid)
            for l, f in Agent.foes.items():
                if ( f.locUpdated != self.observation.step ):
                    del Agent.foes[l]
            if not Agent.teabagged and \
                ((self.team == TEAM_RED and self.observation.score[0] > 97) or \
                (self.team == TEAM_BLUE and self.observation.score[1] > 97 )):
                Agent.teabagged = True
                print Agent.NAME+": Have a nice day\n"
        if self.observation.respawn_in > 0:
            for l, n in Agent.friends.items():
                if n.id < self.id: 
                    if n.observation.respawn_in == self.observation.respawn_in:
                        if Agent.jobs.index(n.currentBehaviour) > Agent.jobs.index(self.currentBehaviour):
                            n.currentBehaviour, self.currentBehaviour = \
                              self.currentBehaviour, n.currentBehaviour
                            for h in Agent.helps:
                                if h[1] == n:
                                    h[1] = self
                                elif h[1] == self:
                                    h[1] = n
            self.currentTBehaviour = None
            self.shootedGuy = False
            if self.currentBehaviour == "DefCP1" or self.currentBehaviour == "DefCP2": 
                label = self.getRelativeMap(self.currentBehaviour[3:])
                if [label, self] not in Agent.helps:
                    Agent.helps.append([label, self])
            return (0,0,False)
        if self.observation.ammo > 0:
            for l, foe in Agent.foes.items():
                if self.inSingleRange(foe.loc):
                    return self.EngageFoe(l)
        (ap, d) = self.findNearestAmmo()
        if ap is not None:
            (result, move) = self.inSingleMove(Agent.aps[ap].loc)
            if result:
                return move
        if (self.currentBehaviour == "AmmoColl" and self.observation.ammo > 2) or \
           (self.observation.ammo > 0 and self.shootedGuy):
            if len(Agent.helps) > 0:
                self.shootedGuy = False
                (label, friend) = Agent.helps.pop(0)
                label = self.getRelativeMap(label) 
                self.currentBehaviour = "Def"+label
                self.currentTBehaviour = None
                friend.currentBehaviour = "AmmoColl"
            elif self.observation.ammo > 0:
                for l,f in Agent.friends.items():
                    if l != self.label and f.observation.ammo < self.observation.ammo:
                        self.shootedGuy = False
                        self.currentBehaviour, f.currentBehaviour = f.currentBehaviour, self.currentBehaviour
                        self.currentTBehaviour = None
                        break
        return getattr(self, self.currentBehaviour)()
    def AmmoColl(self):
        if self.observation.ammo > 0:
            (foe, steps) = self.findNearestFoe()
            if foe is not None and steps < 2:
                return self.EngageFoe(foe)
            for l, foe in Agent.foes.items():
                if point_dist(self.loc, foe.loc) < self.settings.max_see and \
                   not line_intersects_grid(self.loc,foe.loc,self.grid,self.settings.tilesize) and \
                   inCone(self.angle, self.settings.max_turn*2, getAngle(self.loc, foe.loc))==0:
                    return self.EngageFoe(l)
        (a,d) = self.findNearestAmmo()
        if a is not None:
            self.currentTBehaviour = a
        ap2 = self.getRelativeMap("AP2")
        ap1 = self.getRelativeMap("AP1")
        if self.currentTBehaviour == ap2:
            ap1, ap2 = ap2, ap1
        if point_dist(self.loc, Agent.aps[ap1].loc) < 5.0:
            self.currentTBehaviour = ap2
            return self.DefendAmmopoint(ap2)
        else:
            return self.DefendAmmopoint(ap1)
    def DefCP1(self):
        cp1 = self.getRelativeMap("CP1")
        return self.DefendCheckpoint(cp1)
    def DefCP2(self):
        cp2 = self.getRelativeMap("CP2")
        return self.DefendCheckpoint(cp2)
    def TurnTo(self, target):
        turn = angle_fix(getAngle(self.loc, target) - self.angle)
        return (turn, 0, False)
    def MoveTo(self, target):
        self.path, self.length, self.direct = bangBangStar(self.loc, self.angle, target, Agent.mesh, Agent.lineMesh, self.grid, self.settings.tilesize, self.settings.max_turn, self.settings.max_speed)
        if self.path:
            if point_dist(self.loc, self.path[0]) < 5.0 and not self.direct:
                self.path.pop(0)
            dx = self.path[0][0] - self.loc[0]
            dy = self.path[0][1] - self.loc[1]
            self.currentGoal = self.path
            turn = angle_fix(math.atan2(dy, dx) - self.angle)
            distance = (dx**2 + dy**2)**0.5
            if distance == 0: 
                return (0,0,False)
            targetWidth = min(10.0, 2*distance)
            eps = math.asin(targetWidth/(2*distance))
            if coneInCone(0,self.settings.max_turn, turn, eps) != 0:
                absMissing = math.fabs(turn)-self.settings.max_turn
                if absMissing <= self.settings.max_turn:
                    if not self.direct:
                        coneWidth = self.settings.max_turn
                        sStepAngle = angle_fix(getAngle(self.path[0], self.path[1]) - self.angle)
                        result = inCone(math.copysign(math.pi/2, turn), coneWidth, sStepAngle)
                        result = coneInCone(math.copysign(math.pi/2, turn), coneWidth, sStepAngle, self.settings.max_turn/2)
                    else:
                        result = turn 
                    if result * turn < 0 and distance < self.settings.max_speed*2:
                        speed = 0
                    else:
                        xDiffRot= dx*math.cos(-self.angle) - dy*math.sin(-self.angle)
                        speed = math.fabs(xDiffRot)*(2**0.5)
                else:
                    backTurn = angle_fix(math.atan2(-dy, -dx) - self.angle)
                    if self.direct and \
                        coneInCone(0, self.settings.max_turn, backTurn, eps) == 0:
                        turn = backTurn
                        if self.currentSecondaryBehaviour != "EngageFoe":
                            speed = -distance
                        else:
                            speed = -self.settings.max_speed
                            turn = angle_fix(turn-math.copysign(0.18, turn))
                    else:
                        xDiffRot= dx*math.cos(-self.angle) - dy*math.sin(-self.angle)
                        reducer = distance//self.settings.max_speed + 2
                        speed = max((-math.fabs(xDiffRot)*(2**0.5)), -self.settings.max_speed)
                        speed = speed / reducer
            else:
                speed = distance
        else:
            turn = angle_fix(getAngle(self.loc, target) - self.angle)
            speed = point_dist(self.loc, target)
        return (turn,speed, False)
    def EngageFoe(self,l):
        shootAction = False
        distanceAction = 0
        foe = Agent.foes[l]
        self.goal = foe.loc
        distanceToTarget = point_dist(self.loc, foe.loc)
        if ( distanceToTarget > self.settings.max_range ) or ( line_intersects_grid(self.loc,foe.loc,self.grid,self.settings.tilesize)):
            return self.MoveTo(foe.loc)
        angleWithTarget = angle_fix(getAngle(self.loc, foe.loc) - self.angle)
        epsShoot = math.asin(Agent.widthConservative/(2*distanceToTarget))
        angleAction = math.copysign(min(math.fabs(angleWithTarget),self.settings.max_turn), angleWithTarget)
        if inCone(angleWithTarget, epsShoot, angleAction) != 0:
            distanceAction = -self.settings.max_speed/4 
        else :
            shootAction = True
            for l, friend in Agent.friends.items():
                if friend != self:
                    distanceToFriend = point_dist(self.loc,friend.loc)
                    if distanceToFriend < distanceToTarget:
                        angleWithFriend = angle_fix(getAngle(self.loc,friend.loc) - self.angle)
                        eps = math.asin(Agent.width/(2*distanceToFriend))
                        if inCone(angleAction, eps, angleWithFriend) == 0:
                            shootAction = False
                            break
        return (angleAction, distanceAction, shootAction)
    def CampFoe(self, l):
        shootAction = False
        distanceAction = 0
        foe = Agent.foes[l]
        self.goal = foe.loc
        distanceToTarget = point_dist(self.loc, foe.loc)
        if ( distanceToTarget > self.settings.max_range ) or ( line_intersects_grid(self.loc,foe.loc,self.grid,self.settings.tilesize)):
            return self.TurnTo(foe.loc)
        angleWithTarget = angle_fix(getAngle(self.loc, foe.loc) - self.angle)
        epsShoot = math.asin(Agent.widthConservative/(2*distanceToTarget))
        angleAction = math.copysign(min(math.fabs(angleWithTarget),self.settings.max_turn), angleWithTarget)
        if inCone(angleWithTarget, epsShoot, angleAction) != 0:
            distanceAction = 0
        else :
            shootAction = True
            for l, friend in Agent.friends.items():
                if friend != self:
                    distanceToFriend = point_dist(self.loc,friend.loc)
                    if distanceToFriend < distanceToTarget:
                        angleWithFriend = angle_fix(getAngle(self.loc,friend.loc) - self.angle)
                        eps = math.asin(Agent.width/(2*distanceToFriend))
                        if inCone(angleAction, eps, angleWithFriend) == 0:
                            shootAction = False
                            break
        return (angleAction, distanceAction, shootAction)
    def CollectAmmo(self, label):
        self.goal = Agent.aps[label].loc
        return self.MoveTo(Agent.aps[label].loc)
    def ActivateCheckpoint(self, label):
        self.goal = Agent.cps[label].loc
        return self.MoveTo(Agent.cps[label].loc)
    def DefendCheckpointSecondary(self, label):
        cp = Agent.cps[label]
        defendThreshold = 4.6
        (nearestFoe, dist) = self.findNearestFoeToLoc(cp.loc, defendThreshold)
        if ( nearestFoe is not None ) and ( not line_intersects_grid(cp.loc,Agent.foes[nearestFoe].loc,self.grid,self.settings.tilesize)):
            if self.observation.ammo == 0:
                if [label, self] not in Agent.helps:
                    Agent.helps.append([label, self])
            else:
                if point_dist(self.loc, Agent.ninjas["N"+label]) <= 5.0:
                    return ("CampFoe", nearestFoe)
                return ("EngageFoe", nearestFoe)
        else:
            if [label,self] in Agent.helps:
                Agent.helps.remove([label,self])
        if not cp.owned == self.team and self.loc != cp.loc:
            return ("ActivateCheckpoint",label)
        return (None, None)
    def DefendCheckpoint(self, label):
        (self.currentSecondaryBehaviour, self.currentSecondaryTarget) = \
            self.DefendCheckpointSecondary(label)
        if self.currentSecondaryBehaviour is not None:
            return getattr(self, self.currentSecondaryBehaviour)(self.currentSecondaryTarget)
        if self.observation.ammo > 0 and len(Agent.ninjas) > 0:
            ncp = Agent.ninjas["N"+label]
            if point_dist(self.loc, ncp) > 5.0:
                self.goal = ncp
                return self.MoveTo(ncp)
            return self.TurnTo(Agent.dninjas["D"+label])
        else:
            cp = Agent.cps[label]
            if self.loc != cp.loc:
                self.goal = cp.loc
                return self.MoveTo(cp.loc)
            (nearestFoe, dist) = self.findNearestFoeWall()
            if nearestFoe is not None:
                return self.TurnTo(Agent.foes[nearestFoe].loc)
            if self.team == TEAM_RED:
                return self.TurnTo(Agent.bspawns['BS2'].loc)
            else:
                return self.TurnTo(Agent.rspawns['RS2'].loc)
    def DefendAmmopointSecondary(self, label):
        ap = Agent.aps[label]
        if self.observation.ammo != 0:
            defendThreshold = 4
            (nearestFoe, dist) = self.findNearestFoeToLoc(ap.loc, defendThreshold)
            if nearestFoe is not None:
                return ("EngageFoe", nearestFoe)
        return (None, None)
    def DefendAmmopoint(self, label):
        (self.currentSecondaryBehaviour, self.currentSecondaryTarget) = \
            self.DefendAmmopointSecondary(label)
        if self.currentSecondaryBehaviour is not None:
            return getattr(self, self.currentSecondaryBehaviour)(self.currentSecondaryTarget)
        ap = Agent.aps[label]
        if self.loc != ap.loc:
            self.goal = ap.loc
            return self.MoveTo(ap.loc)
        (nearestFoe, dist) = self.findNearestFoe()
        if nearestFoe is not None:
            return self.TurnTo(Agent.foes[nearestFoe].loc)
        if self.team == TEAM_RED:
            return self.TurnTo(Agent.bspawns['BS2'].loc)
        else:
            return self.TurnTo(Agent.rspawns['RS2'].loc)
    def inSingleMove(self, t):
        dx = t[0] - self.loc[0]
        dy = t[1] - self.loc[1]
        distance = (dx**2 + dy**2)**0.5
        if distance > self.settings.max_speed:
            return (False,(0, 0, False))
        turn = angle_fix(math.atan2(dy, dx) - self.angle)
        if turn < self.settings.max_turn and turn > -self.settings.max_turn:
            return (True,(turn, distance, False))
        revTurn = angle_fix(math.atan2(dy,dx) - self.angle)
        if revTurn < self.settings.max_turn and revTurn > -self.settings.max_turn:
            return (True,(revTurn, -distance, False))
        return (False,(0,0, False))
    def inSingleRange(self, loc):
        if point_dist(self.loc, loc) > self.settings.max_range:
            return False
        dx = loc[0] - self.loc[0]
        dy = loc[1] - self.loc[1]
        turn = angle_fix(math.atan2(dy, dx) - self.angle)
        if turn < self.settings.max_turn and turn > -self.settings.max_turn and \
           not line_intersects_grid(self.loc, loc, self.grid, self.settings.tilesize):
                return True
        return False
    def getRelativeMap(self, label):
        if self.team == TEAM_BLUE:
            if label == "CP1":
                label = "CP2"
            elif label == "CP2":
                label = "CP1"
            elif label == "AP1":
                label = "AP2"
            elif label == "AP2":
                label = "AP1"
        return label
    def getOtherAp(self, label):
        if label == "AP1":
            return "AP2"
        return "AP1"
    def findNearestAp(self, minSteps = 999999):
        resAP = None
        steps = 0
        for l, ap in Agent.aps.items():
            steps = self.getApproxSteps(ap.loc)
            if steps < minSteps:
                minSteps = steps
                resAP = l
        return (resAP, steps)
    def findNearestAmmo(self, minSteps = 999999):
        resAP = None
        steps = 0
        for l, ap in Agent.aps.items():
            if ( ap.hasAmmo and ap.hasAmmoUpdated == self.observation.step ):
                steps = self.getApproxSteps(ap.loc)
                if steps < minSteps:
                    minSteps = steps
                    resAP = l
        return (resAP, steps)
    def findNearestFoe(self, minSteps = 999999):
        resFoe = None
        steps = 0
        for l, foe in Agent.foes.items():
            steps = self.getApproxSteps(foe.loc)
            if steps < minSteps:
                minSteps = steps
                resFoe = l
        return (resFoe, steps)
    def findNearestFoeWall(self, minSteps = 999999):
        resFoe = None
        steps = 0
        for l, foe in Agent.foes.items():
            if line_intersects_grid(self.loc, foe.loc, self.grid, self.settings.tilesize):
                continue
            steps = self.getApproxSteps(foe.loc)
            if steps < minSteps:
                minSteps = steps
                resFoe = l
        return (resFoe, steps)
    def findNearestFoeToLoc(self, loc, minSteps = 999999):
        resFoe = None
        steps = 0
        for l, foe in Agent.foes.items():
            steps = self.getApproxSteps(agent = foe, target = loc)
            if steps < minSteps:
                minSteps = steps
                resFoe = l
        return (resFoe, steps)
    def getApproxSteps(self, target, agent = None):
        if agent == None:
            agent = self
        (nodes, length, straightLine) = myFindPath(agent.loc, target, self.mesh, self.grid, self.settings.tilesize)
        dx = nodes[0][0] - self.loc[0]
        dy = nodes[0][1] - self.loc[1]
        angle = math.fabs(math.atan2(dy, dx) - agent.angle)
        return angle / self.settings.max_turn + length / self.settings.max_speed + len(nodes) - 1
    def getApproxStepsNew(self, target, agent = None):
        if agent == None:
            agent = self
        nodes, length, straightLine = bangBangStar(agent.loc, agent.angle, target, Agent.mesh, Agent.lineMesh, self.grid, self.settings.tilesize, self.settings.max_turn, self.settings.max_speed)
        return round(length / self.settings.max_speed) + 1
    def finalize(self, interrupted=False):
        """ This function is called after the game ends, 
            either due to time/score limits, or due to an
            interrupt (CTRL+C) by the user. Use it to
            store any learned variables and write logs/reports.
        """
        pass
    def debug(self, surface):
        pass
def getAngle(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return math.atan2(dy, dx)
def coneInCone(coneDir, conWidth, angle, angleWidth):
    minBorder = inCone(coneDir, conWidth, angle_fix(angle - angleWidth))
    maxBorder = inCone(coneDir, conWidth, angle_fix(angle + angleWidth))
    res = minBorder * maxBorder
    if res > 0:
        return minBorder
    if res == 0:
        return 0
    if inCone(angle, angleWidth, coneDir) == 0:
        return 0
    return inCone(coneDir, conWidth, angle)
def inCone(coneDir, conWidth, angle):
    if coneDir == 0:
        return baseInCone(conWidth, angle)
    highBound = coneDir + conWidth
    lowBound = coneDir - conWidth
    if lowBound < -math.pi:
        angle = math.fmod(angle-2*math.pi, 2*math.pi) 
    elif highBound > math.pi:
        angle = math.fmod(angle+2*math.pi, 2*math.pi) 
    if angle < lowBound:
        return -1
    elif angle > highBound:
        return 1
    else:
        return 0
def baseInCone(conWidth, angle):
    if angle < -conWidth:
        return -1
    if angle > conWidth:
        return 1
    return 0
def myFindPath(start, end, mesh, grid, tilesize=16):
    """ Uses astar to find a path from start to end,
        using the given mesh and tile grid.
        >>> grid = [[0,0,0,0,0],[0,0,0,0,0],[0,0,1,0,0],[0,0,0,0,0],[0,0,0,0,0]]
        >>> mesh = make_nav_mesh([(2,2,1,1)],(0,0,4,4),1)
        >>> find_path((0,0),(4,4),mesh,grid,1)
        [(4, 1), (4, 4)]
    """
    if not line_intersects_grid(start, end, grid, tilesize):
        return ([end], point_dist(start, end), True)
    direct = False
    mesh = copy.deepcopy(mesh)
    mesh[start] = dict([(n, point_dist(start,n)) for n in mesh if not line_intersects_grid(start,n,grid,tilesize)])
    if end not in mesh:
        endconns = [(n, point_dist(end,n)) for n in mesh if not line_intersects_grid(end,n,grid,tilesize)]
        for n, dst in endconns:
            mesh[n][end] = dst
    neighbours = lambda n: mesh[n].keys()
    cost       = lambda n1, n2: mesh[n1][n2]
    goal       = lambda n: n == end
    heuristic  = lambda n: ((n[0]-end[0]) ** 2 + (n[1]-end[1]) ** 2) ** 0.5
    nodes, length = astar(start, neighbours, goal, 0, cost, heuristic)
    if len(nodes) == 1:
        direct = True
    return (nodes, length, direct)
def setupMesh(baseMesh, grid, tilesize, max_turn, max_speed):
    for n in baseMesh:
        for n2 in baseMesh:
            if n != n2 and n2 not in baseMesh[n] and not line_intersects_grid(n, n2, grid, tilesize):
                baseMesh[n][n2] = point_dist(n, n2)
    lineMesh = {}
    for n in baseMesh:
        for n2 in baseMesh:
            if n2 in baseMesh[n]:
                lineMesh[(n,n2)] = {} 
                for n3 in baseMesh[n2]:
                    lineMesh[(n,n2)][(n2,n3)] = max_speed * \
                        (math.fabs(angle_fix(getAngle(n2,n3)-getAngle(n,n2)))//max_turn) + \
                        point_dist(n2, n3)
    return (baseMesh, lineMesh)
def bangBangStar(start, startAngle, end, mesh, lineMesh, grid, tilesize, max_turn, max_speed):
    if not line_intersects_grid(start, end, grid, tilesize):
        return ([end], point_dist(start, end), True)
    direct = False
    lineMesh = copy.deepcopy(lineMesh)
    lineMesh[(start,start)] = {}
    for n in mesh:
        if not line_intersects_grid(start, n, grid, tilesize):
            if start != n:
                lineMesh[(start,start)][(start,n)] = max_speed * \
                    (math.fabs(angle_fix(getAngle(start,n) - startAngle))//max_turn) + \
                    point_dist(start,n)
            if start not in mesh:
                lineMesh[(start,n)] = {}
                for n2 in mesh[n]:
                    lineMesh[(start,n)][(n,n2)] = max_speed * \
                        (math.fabs(angle_fix(getAngle(n,n2)-getAngle(start,n)))//max_turn) + \
                        point_dist(n, n2)
    if end != start:
        if end not in mesh:
            for n2 in mesh.keys() + [start]:
                if not line_intersects_grid(end, n2, grid, tilesize):
                    lineMesh[(n2,end)] = dict([((end,end),0)])
                    for n in mesh.keys() + [start]:
                        if (n, n2) in lineMesh:
                            lineMesh[(n,n2)][(n2,end)] = max_speed * \
                        (math.fabs(angle_fix(getAngle(n2,end)-getAngle(n,n2)))//max_turn) + \
                        point_dist(n2, end)
        else:
            for n in mesh.keys() + [start]:
                if (n, end) in lineMesh:
                    lineMesh[(n,end)][(end,end)] = 0
    neighbours = lambda n: lineMesh[n].keys()
    cost       = lambda n1, n2: lineMesh[n1][n2]
    goal       = lambda n: n == (end,end)
    heuristic  = lambda n: point_dist(n[1], end)
    nodes, length = astar((start,start), neighbours, goal, 0, cost, heuristic)
    if len(nodes) == 1:
        direct = True
    path = []
    for (n,n2) in nodes:
            path += [n2]
    return (path, length, direct)
def extractCoords(m, width):
    t = string.split(m, "\n")
    coordY = -1
    for line in t:
        line = line.replace(" ", "")
        l = list(line)
        coordX = 0
        for char in l:
            coords = (coordX*width+width/2, coordY*width+width/2)
            if char == 'C':
                label = "CP"+str(len(Agent.cps)+1)
                Agent.cps[label] = CpData(label=label, loc = coords, owned = None)
            elif char == "A":
                label = "AP"+str(len(Agent.aps)+1)
                Agent.aps[label] = ApData(label=label, loc = coords, hasAmmo = True, turn = 1)
            elif char == "R":
                label = "RS"+str(len(Agent.rspawns)+1)
                Agent.rspawns[label] = SpawnData(label=label, loc = coords)
            elif char == "B":
                label = "BS"+str(len(Agent.bspawns)+1)
                Agent.bspawns[label] = SpawnData(label=label, loc = coords)
            elif char == "D":
                label = "DCP"+str(len(Agent.dninjas)+1)
                Agent.dninjas[label] = coords
            elif char == "N":
                label = "NCP"+str(len(Agent.ninjas)+1)
                Agent.ninjas[label] = coords
            coordX = coordX + 1
        coordY = coordY + 1
