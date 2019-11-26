# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
import random, time, util, copy
from game import Directions



#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'Agent', second = 'Agent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).

    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    return None

class Agent(DummyAgent):

  def registerInitialState(self, gameState):
    self.start = gameState.getAgentPosition(self.index)
    CaptureAgent.registerInitialState(self, gameState)
    # Updates food and sets self.myFood
    self.myFood = None
    self.updateMyFood(gameState)

    self.onChasing = False
    self.target = None
    self.lastAction = None
    self.invaderNum = 0
    self.eatFood = 0
    self.missingFood = None

    self.midWidth = gameState.data.layout.width / 2
    noWalls = []
    walls = gameState.getWalls()
    for x in range(walls.width):
      for y in range(walls.height):
        if walls[x][y] is False:
          noWalls.append((x, y))
    self.notWalls = noWalls
    self.startTime= time.time()

  """
  def chooseAction(self, gameState):
    
    actions = gameState.getLegalActions(self.index)
    self.setTarget(gameState)
    if self.target:
      self.onChasing = True
    else:
      self.onChasing = False

    values = [self.reward(gameState, a) for a in actions]
    maxValue = max(values)
    bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    bestAction = random.choice(bestActions)
    self.lastAction = bestAction

    return bestAction

  """
  def chooseAction(self, gameState):
      'preTarget = self.target'
      self.setTarget(gameState)

      self.startTime = time.time()

      if gameState.getAgentState(self.index).isPacman == False:
        self.eatFood = 0
      if self.target:
        teammate = [a for a in self.getTeam(gameState) if a != self.index][0]
        if self.invaderNum == 2:
          self.onChasing = True
          action = self.MDP(gameState)
        elif self.getMazeDistance(gameState.getAgentPosition(self.index), self.target) <= self.getMazeDistance(gameState.getAgentPosition(teammate),self.target):
          self.onChasing = True
          action = self.MDP(gameState)
        else:
          self.onChasing = False
          '''
          if preTarget != None:
            self.onChasing = True
            action = self.MDP(gameState)
          
          else:
            self.target = None
            self.onChasing = False
           '''
          self.updateMyFood(gameState)
          action =  self.MCTSearch(gameState)
      else:
        self.updateMyFood(gameState)
        self.onChasing = False
        action = self.MCTSearch(gameState)
      if len(self.getFood(gameState).asList()) - len(self.getFood(gameState.generateSuccessor(self.index, action)).asList()) > 0:
        self.eatFood += 1
      timelength = round(1000 * (time.time() - self.startTime))
      if timelength > 1000:
        print timelength
      return action

  def MDP(self,gameState):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    'actions = gameState.getLegalActions(self.index)'
    '''
    if "Stop" in actions:
      actions.remove  ("Stop")
    '''
    states = self.notWalls
    gama = 0.9
    v = {}
    a = {}
    vn = {}
    for state in states:
        v[state] = self.reward(gameState,"",state)
        a[state] = ""
    endTime = time.time()
    'while (int(round(1000 * (endTime - self.startTime))) < 750):'
    for i in range(0,3):
        for state in states:
            bestAction = ""
            bestV = -100000
            nextStates = []
            if (state[0] + 1, state[1]) in states:
              nextStates.append([(state[0] + 1, state[1]), "East"])
            if (state[0] , state[1] + 1) in states:
              nextStates.append([(state[0] , state[1] + 1), "North"])
            if (state[0] - 1, state[1]) in states:
              nextStates.append([(state[0] - 1, state[1]), "West"])
            if (state[0], state[1] - 1) in states:
              nextStates.append([(state[0], state[1] - 1), "South"])
            for nextState in nextStates:
              v_tem = gama/4 * self.reward(gameState, nextState[1], nextState[0]) + v[state]
              if bestV == -100000:
                bestAction = nextState[1]
                bestV = v_tem
              if v_tem > bestV:
                bestAction = nextState[1]
                bestV = v_tem
            vn[state] = bestV
            a[state] = bestAction
            """
            for action in actions:
                successor = gameState.generateSuccessor(self.index, action)
                nextState = successor.getAgentState(self.index).getPosition()
                v_tem = self.reward(gameState,action) + gama * v[nextState]
                if v_tem > bestV:
                    bestAction = action
                    bestV = v_tem
            v[state] = bestV
            a[state] = bestAction
            """
        v = copy.deepcopy(vn)
        vn = {}
        'endTime = time.time()'
    return a[myPos]

  def reward(self, gameState, action, myPos):
    features = self.getFeatures(gameState, action, myPos)
    weights = self.getWeights(gameState, action)

    return features * weights

  def getFeatures(self, gameState, action, myPos):
    

    if self.onChasing and self.target:
      features = self.getDefenceFeatures(gameState, action, myPos)
      return features
    else:
      features = self.getGeneralFeatures(gameState, action)
      return features


  def getWeights(self, gameState, action):

    return { 'stop': -100, 'reverse': -80, 'distanceToMiddle':-50,
              'distanceToTarget': -10, 'onDefense':1000}



  def getGeneralFeatures(self, gameState, action):
    
    features = util.Counter()
    successor = self.getSuccessor(gameState, action)
    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()

    distanceToMiddle = min([self.distancer.getDistance(myPos, (self.midWidth, i))
                             for i in range(gameState.data.layout.height)
                             if (self.midWidth, i) in self.notWalls])

    features['distanceToMiddle'] = distanceToMiddle

    features['onDefense'] = 1
    if myState.isPacman:
      features['onDefense'] = 0

    # always prevent stopping
    if action == 'Stop':
      features['stop'] = 1

    # prevent going back to the last spot
    if self.lastAction and action == Directions.REVERSE[self.lastAction]:
      features['reverse'] = 1

    return features

  def getDefenceFeatures(self, gameState, action, myPos):
    # if there is enemy in our side
    features = util.Counter()

    """
    successor = self.getSuccessor(gameState, action)
    
    myState = successor.getAgentState(self.index)
    myPos = myState.getPosition()
    

    # prevent it to stop
    if action == Directions.STOP:
      features['stop'] = 1
    """
    # go to the target
    features["distanceToTarget"] = self.getMazeDistance(self.target, myPos)
    return features

  def enemyComing(self, gameState):
    missingFood = None
    if self.missingFood:
      if self.getMazeDistance(self.missingFood, gameState.getAgentPosition(self.index)) < 3:
        self.missingFood = None
    if self.red:
      redFood = gameState.getRedFood().asList()
      if len(redFood) < len(self.myFood):
        for food in self.myFood:
          if food not in redFood:
            missingFood = food
            self.myFood = redFood
            break
    else:
      blueFood = gameState.getBlueFood().asList()
      if len(blueFood) < len(self.myFood):
        for food in self.myFood:
          if food not in blueFood:
            missingFood = food
            self.myFood = blueFood
            break
    if missingFood == None:
      return self.missingFood
    else:
      return missingFood

  def setTarget(self, gameState):
    myState = gameState.getAgentState(self.index)
    myPos = myState.getPosition()
    self.missingFood = self.enemyComing(gameState)
    enemyIndex = [i for i in self.getOpponents(gameState)]
    enemyState = [gameState.getAgentState(i) for i in enemyIndex]
    invader = [i for i in enemyState if i.isPacman and i.getPosition() != None]
    invaderPos = [i.getPosition() for i in invader]
    if len(invader) > 1:
      self.invaderNum = 2
    else:
      self.invaderNum = 0
    closest = None
    closestInvader = None
    for i in invaderPos:
        distance = self.getMazeDistance(myPos,i)
        if  distance < closest or closest == None:
            closest = distance
            closestInvader = i
    enemyPos = closestInvader

    if len(invader) != 0:
      self.target = enemyPos
    elif self.target == myPos:
      self.updateMyFood(gameState)
      self.missingFood = None
      self.target = None
      self.onChasing = False
    elif self.missingFood:
      self.target = self.missingFood
    else:
      self.updateMyFood(gameState)
      self.target = None
      self.onChasing = False
      """
    if missingFood:
      self.target = missingFood
    elif len(invader) != 0:
      self.target = enemyPos
    if self.onChasing and len(invader) > 0:
      self.target = enemyPos
    if self.target == myPos:
      self.missingFood = None
      self.target = None
      self.onChasing = False
      """

  def updateMyFood(self, gameState):
    if self.red:
      self.myFood = gameState.getRedFood().asList()
    else:
      self.myFood = gameState.getBlueFood().asList()

  def getSuccessor(self, gameState, action):
    successor = gameState.generateSuccessor(self.index, action)

    return successor

  def MCTSearch(self, gameState):
    initPos = gameState.getAgentState(self.index).getPosition()
    initEnemyPos = [gameState.getAgentState(enemy).getPosition() for enemy in self.getOpponents(gameState)]
    states = [[[initPos, initEnemyPos, 0, self.getScore(gameState)], None, 0, 0.0, 0]]
    endTime = time.time()
    'while (int(round(1000 * (endTime - self.startTime))) < 750):'
    while len(states) < 15:
      footprint = [states[0][0]]
      succ = self.seekTeamSucc(gameState, states, footprint)
      if succ[0] == None:
        continue
      reward = self.seekReward(gameState, succ, states)
      self.upgrade(states, reward, succ, footprint)
      endTime = time.time()
    q = -100000000
    for state in states:
      if state[2] == 1:
        if state[3] > q:
          q = state[3]
          action = state[1]
    return action

  def seekTeamSucc(self, gameState, states, footprint):
    actions = gameState.getLegalActions(self.index)
    if "Stop" in actions:
      actions.remove("Stop")
    while True:
      otherA = False
      action = random.choice(actions)
      actions.remove(action)
      succState = gameState.generateSuccessor(self.index, action)
      score = self.getScore(succState) - self.getScore(gameState)
      newStep = [succState.getAgentState(self.index).getPosition(),
                 [gameState.getAgentState(enemy).getPosition() for enemy in self.getOpponents(succState)],
                 len(self.getFood(gameState).asList()) - len(self.getFood(succState).asList()),
                 score]
      if len(footprint) != 0:
        newStep[2] = newStep[2] + footprint[len(footprint) - 1][2]
      for step in footprint:
        if step == newStep:
          if len(actions) == 0:
            return [None, None, None, None]
          else:
            otherA = True
            break
      if otherA == True:
        continue
      else:
        break
    footprint.append(newStep)
    for state in states:
      if state[0] == newStep:
        newStep, action, score, succState = self.seekEnemySucc(succState, states, footprint)
        break
    return [newStep, action, score, succState]

  def seekEnemySucc(self, gameState, states, footprint):
    prePos = footprint[len(footprint) - 1]
    enemy = self.getOpponents(gameState)
    enemyPos = [None, None]
    removeEnemy = []
    for e in enemy:
      if gameState.getAgentState(e).isPacman == True:
        removeEnemy.append(e)
      else:
        ePos = gameState.getAgentState(e).getPosition()
        if ePos != None:
          if self.getMazeDistance(prePos[0], ePos) >= 10:
            enemyPos.pop(0)
            enemyPos.append(ePos)
          else:
            removeEnemy.append(e)
        else:
          removeEnemy.append(e)
    for e in removeEnemy:
      enemy.remove(e)
    action = None
    succState = gameState
    for e in enemy:
      actions = gameState.getLegalActions(e)
      if "Stop" in actions:
        actions.remove("Stop")
      action = random.choice(actions)
      succState = gameState.generateSuccessor(e, action)
      enemyPos.remove(gameState.getAgentState(e).getPosition())
      enemyPos.append(succState.getAgentState(e).getPosition())
    if enemyPos == prePos[1]:
      return self.seekTeamSucc(gameState, states, footprint)
    score = self.getScore(succState)
    newStep = [prePos[0], enemyPos, prePos[2], prePos[3]]
    footprint.append(newStep)
    for state in states:
      if state[0] == newStep:
        newStep, action, score, succState = self.seekTeamSucc(succState, states, footprint)
        break
    return [newStep, action, score, succState]

  def seekReward(self, gameState, succ, states):
    food = self.getFood(succ[3]).asList()
    reward = 20.0 * (len(self.getFood(gameState).asList()) - len(food))
    reward += 40 * (len(self.getCapsules(gameState)) - len(self.getCapsules(succ[3])))
    if len(food) > 0:
      'foodDistance = min([self.getMazeDistance(succ[0][0], foodDot) for foodDot in food])'
      minFoodDot = min([foodDot for foodDot in food], key = lambda x: self.getMazeDistance(succ[0][0], x))
      foodDistance = self.getMazeDistance(succ[0][0], minFoodDot)
      foodDistance += sum([self.getMazeDistance(foodDot, minFoodDot) for foodDot in food])
    else:
      foodDistance = 0
      reward -= 0.01 * self.getMazeDistance(succ[0][0], self.start)
    '''
    if gameState.getAgentState(self.index).isPacman == False:
      if len(food) > 0:
        reward -= foodDistance
    else:
      if succ[3].getAgentState(self.index).isPacman == False:
        if succ[2] == 0:
          reward -= foodDistance
          '''
    reward -= 0.001 * foodDistance
    enemies = [succ[3].getAgentState(ghost) for ghost in self.getOpponents(succ[3])]
    if succ[3].getAgentState(self.index).isPacman:
      p = 1
    else:
      p = 0
    ghosts = [a for a in enemies if ((a.isPacman == False) and (a.getPosition() != None))]
    enemyDistance = 10
    if len(ghosts) > 0:
      if gameState.getAgentState(self.getOpponents(succ[3])[0]).scaredTimer == 0:
        enemyDistance = min([self.getMazeDistance(succ[0][0], ghostAgent.getPosition()) for ghostAgent in ghosts])
    if enemyDistance > 10:
      enemyDistance = 10
    reward += -500 * (5 - enemyDistance) * p + 900*succ[2]
    if self.eatFood > 4:
      reward -= 1000 * self.getMazeDistance(succ[0][0], self.start) - 800 * succ[2]
    return reward

  def upgrade(self, states, reward, succ, footprint):
    states.append([succ[0], succ[1], len(footprint) - 1, reward, -1])
    for step in footprint:
      for state in states:
        if state[0] == step:
          if state[4] != -1:
           state[3] = 1.0 * (state[3] * state[4] + reward) / (state[4] + 1)
          state[4] = state[4] + 1
          break

