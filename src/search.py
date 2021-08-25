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
import sys, traceback
import searchAgents

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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    
    pila = util.Stack()
    actual = Node(problem.getStartState(),0,0,None)
    expanded = []

    while not problem.isGoalState(actual.state):
        successors = problem.getSuccessors(actual.state)
        expanded.append(actual.state)
    
        for x in successors:
            if not x[0] in expanded:
                # state, action_since_parent, way_cost,parent
                new = Node(x[0], x[1], actual.way_cost + x[2], actual)   
                pila.push(new)

        actual = pila.pop()

    return actual.build_path_actions()  


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    return generic_search_function(problem, nullHeuristic)


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    return generic_search_function(problem, nullHeuristic)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    try:
        actions = generic_search_function(problem, heuristic)
    except:
        exc_info = sys.exc_info()
        print(exc_info[0])
        print(exc_info[1])
        traceback.print_tb(exc_info[2])
    return actions


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

def generic_search_function(problem,heuristic):
    priorityqueue = util.PriorityQueue()
    parents = {}
    distances = {}
    action_since_parent = {}

    initial = problem.getStartState()
    parents[initial] = None
    distances[initial] = 0
    action_since_parent[initial] = None
    actual = initial

    while not problem.isGoalState(actual):        
        successors = problem.getSuccessors(actual)

        for x in successors:
            new_state = x[0]
             
            if not new_state in parents or distances[new_state] > distances[actual] + x[2]:
                parents[new_state] = actual
                distances[new_state] = distances[actual] + x[2]
                action_since_parent[new_state] = x[1]
                priorityqueue.update(new_state, distances[new_state] + heuristic(new_state,problem))

        actual = priorityqueue.pop()              

    actions = []
    while actual != initial:
        actions = [action_since_parent[actual]] + actions
        actual = parents[actual]

    return actions

class Node:
    def __init__(self, state, action_since_parent, way_cost,parent):
        self.state = state
        self.parent = parent
        self.action_since_parent = action_since_parent
        self.way_cost = way_cost

    def __str__(self):
        ps = "null" if self.parent is None else self.parent.state
        #return "(state: "+str(self.state)+", parent_state: "+str(ps)+")"
        return "(state: "+str(self.state)+")"

    def __eq__(self,other):
        return self.state == other.state

    def build_path_actions(self):
        return [] if self.parent is None else self.parent.build_path_actions() + [self.action_since_parent]