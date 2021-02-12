#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os
from search import * #for search engines
from snowman import SnowmanState, Direction, snowman_goal_state #for snowball specific classes
from test_problems import PROBLEMS #20 test problems

def heur_manhattan_distance(state):
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a snowman state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each snowball that has yet to be stored and the storage point is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    ans = 0
    factor = 1
    x_d, y_d = state.destination
    for x,y in state.snowballs.keys():
      size = state.snowballs[(x,y)]
      if size == 6:
        factor = 3
      elif size >= 3:
        factor = 2
      else:
        factor = 1
      ans += (abs(x_d - x) + abs(y_d - y))*factor
    return ans


#HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible snowball heuristic'''
  '''INPUT: a snowball state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  return len(state.snowballs)

def heur_alternate(state):
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.

    dest_factor = 3
    robo_factor = 1
    robo_dest_factor = 0

    h = 0

    # 2. corners are bad unless the destination is there
    # 3. walls are bad unless the destination is there
    for new_location in state.snowballs:
      if (new_location[0] == 0 and state.destination[0] != 0) or (state.destination[0] != state.width-1 and new_location[0] == state.width-1):
          return float('inf')
      if (state.destination[1] != 0 and new_location[1] == 0) or (state.destination[1] != state.height-1 and new_location[1] == state.height-1):
          return float('inf')


    # 4. obstacles are bad unless the destination is there
    #    - tunnels are okay IMPLEMENT
    for x,y in state.snowballs:
      if (x-1, y) in state.obstacles and (x, y+1) in state.obstacles and (x,y) not in state.destination:
          return float('inf')
      if (x-1, y) in state.obstacles and (x, y-1) in state.obstacles and (x,y) not in state.destination:
          return float('inf')
      if (x+1, y) in state.obstacles and (x, y+1) in state.obstacles and (x,y) not in state.destination:
          return float('inf')
      if (x+1, y) in state.obstacles and (x, y-1) in state.obstacles and (x,y) not in state.destination:
          return float('inf')

    # closer the snowballs to the destination the better
    factor = 1
    x_d, y_d = state.destination
    for x,y in state.snowballs:
      size = state.snowballs[(x,y)]
      if size == 6:
        factor = 3
      elif size >= 3:
        factor = 2
      else:
        factor = 1
      h += (abs(x_d - x) + abs(y_d - y))*factor*dest_factor

    # closer the robot to the snowballs the better
    factor = 1
    x_d, y_d = state.robot
    for x,y in state.snowballs:
      size = state.snowballs[(x,y)]
      if size == 6:
        factor = 3
      elif size >= 3:
        factor = 2
      else:
        factor = 1
      h += (abs(x_d - x) + abs(y_d - y))*factor*robo_factor

    # robot being closer to the destination is good
    x, y = state.destination
    h += (abs(x_d - x) + abs(y_d - y))*robo_dest_factor

    # return net value
    return h

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 5):
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  start_time = os.times()[0]
  end_time = start_time + timebound

  se = SearchEngine('best_first', 'full')

  # if no sol, we can't prune since there's nothing to compare with
  cost = None
  ans = None

  while(os.times()[0] < end_time):
    se.init_search(initial_state,snowman_goal_state,heur_fn, lambda sN : fval_function(sN,weight))
    final = se.search(end_time-os.times()[0], cost)
    if final:
      ans = final
      # prune paths that are worse
      cost = (final.gval - 1, float('inf'), final.gval - 1)
    else:
      break
    weight = weight-1
  return ans

def anytime_gbfs(initial_state, heur_fn, timebound = 5):
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of weighted astar algorithm'''
  start_time = os.times()[0]
  end_time = start_time + timebound

  se = SearchEngine('best_first', 'full')
  se.init_search(initial_state, goal_fn=snowman_goal_state, heur_fn=heur_fn)

  # if no sol, we can't prune since there's nothing to compare with
  cost = None
  ans = None

  while(os.times()[0] < end_time):
    final = se.search(end_time-os.times()[0], cost)
    if final:
      ans = final
      # prune paths that are worse
      cost = (final.gval - 1, float('inf'), float('inf'))
    else:
      break
  return ans
