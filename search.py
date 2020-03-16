# search.py
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    c = problem.getStartState()
    exploredState = []
    exploredState.append(start)
    states = util.Stack()
    stateTuple = (start, [])
    states.push(stateTuple)
    while not states.isEmpty() and not problem.isGoalState(c):
        state, actions = states.pop()
        exploredState.append(state)
        successor = problem.getSuccessors(state)
        for i in successor:
            coordinates = i[0]
            if not coordinates in exploredState:
                c = i[0]
                direction = i[1]
                states.push((coordinates, actions + [direction]))
    return actions + [direction]
    # startNode_copy = problem.getStartState()
    # startNode = problem.getStartState()
    # visited_state = []
    # # exploredState.append(start)
    # states = []
    # stateTuple = (startNode, [])
    # states.append(stateTuple)
    # count = 0
    # while len(states) > 0:
    #     if(not bool(problem.isGoalState(startNode_copy))):
    #         state, actions = states.pop()
    #         visited_state.append(state)
    #         successor = problem.getSuccessors(state)
    #         count = count + 1
    #         for i in successor:
    #             coordinates = i[0]
    #             if not coordinates in visited_state:
    #                 startNode_copy = i[0]
    #                 direction = i[1]
    #                 # print("direction ", direction)
    #                 states.append((coordinates, actions + [direction]))
    #                 if (bool(problem.isGoalState(startNode_copy))):
    #                     return actions + [direction]

    # # print("actions ", actions)
    # # print(actions + [direction])
    # util.raiseNotDefined()


def breadthFirstSearch(problem):

    print("Inside Bfs")
    start = problem.getStartState()

    #start = (4,5)
    VisitedState = []
    que = util.Queue()
    node = (start, [])
    que.push(node)
    print("start state is :", start)

    while que:
        state, path = que.pop()

        if(not state in VisitedState):
            VisitedState.append(state)

            if problem.isGoalState(state):
                return path
            else:
                neighbour = problem.getSuccessors(state)
                for i in neighbour:
                    position = i[0]
                    direction = i[1]
                    que.push((position, path+[direction]))
    return []
    util.raiseNotDefined()


def uniformCostSearch(problem):
    "Search the node of least total cost first. "
    "*** YOUR CODE HERE ***"
    start = problem.getStartState()
    exploredState = []
    states = util.PriorityQueue()
    states.push((start, []), 0)
    while not states.isEmpty():
        state, actions = states.pop()
        if problem.isGoalState(state):
            return actions
        if state not in exploredState:
            successors = problem.getSuccessors(state)
            for succ in successors:
                coordinates = succ[0]
                if coordinates not in exploredState:
                    directions = succ[1]
                    newCost = actions + [directions]
                    states.push(
                        (coordinates, actions + [directions]), problem.getCostOfActions(newCost))
        exploredState.append(state)
    return actions
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    "Search the node that has the lowest combined cost and heuristic first."
    start = problem.getStartState()
    VisitedState = []
    que = util.PriorityQueue()
    node = (start, [])
    cost = 0
    que.push(node, nullHeuristic(start, problem))

    while que:
        state, path = que.pop()

        if(not state in VisitedState):
            VisitedState.append(state)

            if problem.isGoalState(state):
                return path
            else:
                neighbour = problem.getSuccessors(state)
                for i in neighbour:
                    position = i[0]
                    direction = i[1]
                    cost = problem.getCostOfActions(
                        path+[direction]) + heuristic(position, problem)
                    que.push((position, path+[direction]), cost)
    return []
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
