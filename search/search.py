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
    "*** YOUR CODE HERE ***"

    # def ucs_init(cur) :
    #     if cur in logged :
    #         return None
    #     print("cur: ", cur, "; is goal? ", problem.isGoalState(cur))
    #     child = problem.getSuccessors(cur)
    #     if not child or cur in logged :
    #         return None
    #     logged.append(cur)
        
    #     for c in child :
    #         ucs_init(c[0])
    
    # logged = list()

    # cur = problem.getStartState()
    # ucs_init(cur)
    # print(logged)

    visited = list()
    queue = util.Queue()
    bfs_res = list()

    start = problem.getStartState()
    queue.push(start)
    goal_pending = True
    while not queue.isEmpty() and goal_pending:
        cur = queue.pop()
        print("examine: ",cur)
        if problem.isGoalState(cur) :
            goal_pending = False
            print("Goal reached")
            break
        if cur not in visited : 
            visited.append(cur)
          
            children = problem.getSuccessors(cur) 
            for neighbor in children :
                if neighbor[0] not in visited :
                    print("     expand: ", neighbor[0])
                    if problem.isGoalState(neighbor[0]) :
                        goal_pending = False
                        print("Goal reached")
                        break
                    queue.push(neighbor[0])
                    bfs_res.append(neighbor)

    rev_bfs_res = bfs_res[::-1]

    prev = rev_bfs_res[0]
    actions = list()
    # actions.append(prev)
    # print(rev_bfs_res)
    for action in rev_bfs_res :
        if (action != prev) :
            childrens = problem.getSuccessors(action[0])
            # print("prev: ", prev)
            # print("     action: ", action)
            if prev in childrens :
                prev = action
                actions.append(action)

    print(actions[::-1])
    retlist = []
    for a in actions :
        retlist.append(a[1])
    print(retlist[::-1])
    return retlist[::-1]

 
# Dijkstra's algorithm
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # def ucs_init(cur) :

    #     if cur in logged :
    #         return None
        
    #     child = problem.getSuccessors(cur)
    #     if not child or cur in logged :
    #         return None
    #     logged.append(cur)
        
    #     for c in child :
    #         working_queue.push(c[0], float('inf')) 
    #         ucs_table[c[0]] = float('inf')
    #         ucs_init(c[0])

    # working_queue = util.PriorityQueue()

    # visited = set()

    # cur = problem.getStartState()
    # ucs_init(cur)
    # ucs_table[cur] = 0
    # working_queue.update(cur, 0)
    # print(ucs_table)
        
    # while not working_queue.isEmpty() :
    #     cur = working_queue.pop()
    #     print("CUR: ", cur)
    #     if problem.isGoalState(cur) :
    #         print("Goal reached")

    #         break
    #     children = problem.getSuccessors(cur) 
    #     if cur not in visited :
    #         visited.add(cur)
    #         for neighbor in children :
    #             print(neighbor)
    #             new_cost = ucs_table[cur] + ucs_table[neighbor[0]]
    #             print(new_cost)
    #             if (ucs_table[neighbor[0]][1] < new_cost) :
    #                 working_queue.update(neighbor, new_cost)
    #                 ucs_table.update(neighbor, new_cost)
    # for key in ucs_table :
    #     print (ucs_table[key])
    # return logged
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
