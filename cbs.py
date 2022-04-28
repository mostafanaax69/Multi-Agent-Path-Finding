import time as timer
import heapq
import copy
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    collision_lst = []
    for timestep in range(max(len(path1), len(path2))):
        agent1_currloc = get_location(path1, timestep)
        agent2_currloc = get_location(path2, timestep)
        # vertex : 2 agents - same cell - same time
        if agent1_currloc == agent2_currloc:
            collision_lst.append({'loc': [agent1_currloc], 'timestep': timestep})

        # edges :  if the robots swap their location at the same timestep.
        if timestep != 0:
            prev_agent1 = get_location(path1, timestep - 1)  # prev loc of first agent
            prev_agent2 = get_location(path2, timestep - 1)  # prev loc of sec agent
            if [prev_agent1, agent1_currloc] == [agent2_currloc, prev_agent2]:
                collision_lst.append({'loc': [prev_agent1, agent1_currloc], 'timestep': timestep})

    return collision_lst


def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.

    collisions_el = []
    for agent1 in range(len(paths)-1):
        start = agent1 + 1
        end = len(paths)
        for agent2 in range(start, end):
            collosion = detect_collision(paths[agent1], paths[agent2])
            #list of first collosions
            # collosion paramter from detect_collosion is the first collosing between these paths
            #could have used if c is not none // maybe better but it works like this haha
            for c in collosion:
                collisions_el.append(
                    {'a1': agent1, 'a2': agent2, 'loc': c['loc'], 'timestep': c['timestep']})
                # print(c)

    return collisions_el


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    # edge , location has 2 fields
    if len(collision['loc']) != 1:
        first_cons = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        #the other direction of the edge in the constraint
        sec_cons = {'agent': collision['a2'], 'loc': [collision['loc'][1], collision['loc'][0]],
                    'timestep': collision['timestep']}
        return [first_cons, sec_cons]

    # vertex , there is only on location which is the collision location :
    else:
        first_cons = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        sec_cons = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']}
        return [first_cons, sec_cons]



def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent

            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit




        # R.patyhs , collision , cost already declared and calcuated above so we start from step 6 in algorithm 1

        while (self.open_list) is not None:  # open is not empty

            P = self.pop_node()  # node with the smallest cost

            if len(P['collisions']) == 0:  # step 8 in algo
                return P['paths']  # P is a goal node , step 9 in algo

            collision = P['collisions'][0]  # step 10 in algo
            constraints = standard_splitting(collision)  # step 11

            for constraint in constraints:  # step 12 in algo
                # step 13 create a new node , same as root node
                Q = {'cost': 0,
                     'constraints': [],
                     'paths': [],
                     'collisions': []}

                # step 14 , Union constraints and P constraints into Q constraints

                for i in P['constraints']:
                    Q['constraints'].append(i)

                Q['constraints'].append(constraint)

                # step 15 q[paths] equal to p[paths]
                Q['paths'] = copy.deepcopy(P['paths'])

                # step 16
                agent = constraint['agent']

                # step 17
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent,
                              Q['constraints'])

                # step 18 in algo
                if path is not None:
                    Q['paths'][agent] = path  # step 19 in algo
                    Q['collisions'] = detect_collisions(Q['paths'])  # step 20 in algo
                    Q['cost'] = get_sum_of_cost(Q['paths'])  # step 21 in algo

                    self.push_node(Q)  # step 22

        raise BaseException('No solutions')

    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
