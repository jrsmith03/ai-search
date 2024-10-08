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
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    def dfsHelp(start_state, problem, goal_reached) :

        if goal_reached[0] == 1 :
            print("return from if statement")
            return stack
        if (problem.isGoalState(start_state)) :
            print("Goal reached")
            goal_reached[0] = 1
            return stack
        visited.add(start_state)
        # actions.add(start_state.action)

        children = problem.getSuccessors(start_state)
        print(start_state)
        for kid in children:
            if kid[0] not in visited :
                print(kid)
                stack.push(kid[1])
                dfsHelp(kid[0], problem, goal_reached)

                if goal_reached[0] == 1:
                    return stack
                else:     
                    print("POP: ", stack.pop())
    
    visited = set()
    actions = list()
    stack = util.Stack()
    goal_reached = [0]
    
    dfsHelp(problem.getStartState(), problem, goal_reached)
    while (not stack.isEmpty()) :
        actions.append(stack.pop())
    print(actions[::-1])
    return actions[::-1]

    
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    visited = set()  
    queue = util.Queue()

    start = problem.getStartState()
    queue.push((start, []))

    while not queue.isEmpty():
        current_state, path = queue.pop()

        if problem.isGoalState(current_state):
            return path  

        if current_state not in visited:
            visited.add(current_state)

            for next_state, action, _ in problem.getSuccessors(current_state):
                if next_state not in visited:
                    queue.push((next_state, path + [action]))

    return []  
 
# Dijkstra's algorithm
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    frontier.push((problem.getStartState(), [], 0),0)
    visited = set()

    while not frontier.isEmpty():
        state, actions, cost = frontier.pop()

        if problem.isGoalState(state):
            return actions
        
        if state not in visited:
            visited.add(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                frontier.push((successor, actions + [action], cost + stepCost), cost + stepCost)
    return []
            

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    frontier = util.PriorityQueue()
    frontier.push((problem.getStartState(), [], 0),0)
    visited = set()

    while not frontier.isEmpty():
        state, actions, cost = frontier.pop()

        if problem.isGoalState(state):
            return actions
        
        if state not in visited:
            visited.add(state)
            for successor, action, stepCost in problem.getSuccessors(state):
                frontier.push((successor, actions + [action], cost + stepCost), cost + stepCost + heuristic(successor, problem))
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
