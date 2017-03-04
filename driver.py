import sys


class Node:
    '''Data structure for nodes in search tree '''
    def __init__(self, parent=None, path="", path_cost=0, **kwargs):
        self.parent = parent
        self.action = ""
        self.path_cost = 1 + parent.path_cost if parent is not None else 0
        self.board_state = ''
        self.neighbours = []
        self.depth = 0

    def __cmp__(self, other):
        return cmp(self.path_cost, other.path_cost)

    def expand(self, method=''):
        ''' Node expansion function
            - gets possible moves from current board state
            - create child nodes from the possible moves and states
        '''
        lst_successors = successors(self.board_state, method)
        neighbours = []

        if lst_successors:

            for successor in lst_successors:

                node = Node()
                node.parent = self
                node.action = self.action + ',' + successor.get('action')
                node.path_cost = self.path_cost + 1
                node.board_state = successor.get('state')

                if method == 'ast' or method == 'ida':
                    # path cost implementation , f(x) = g(x) + h(x)
                    node.path_cost += h_one(node.board_state)

                node.depth = self.depth + 1

                neighbours.append(node)

        self.neighbours = neighbours


class Solver():
    ''' Solver class

    '''
    def __init__(self, method, board):

        if method.lower() not in ('bfs', 'dfs', 'ast', 'ida'):
            print 'Invalid method:{}'.format(method)
            return

        lst_board = board.split(',')
        if len(lst_board) != 9:
            print 'Invalid board: {}'.format(board)
            return

        try:
            board = map(int, lst_board)
            for tile in board:
                if tile not in range(0, 9):
                    print 'Invalid tile :%d for a 3 x 3 board' % tile
                    return
            board_state = [board[0:3], board[3:6], board[6:9]]
        except ValueError as error:
            print 'Invalid board:', error
            return

        self.method = method
        self.board_state = board_state

    def solve(self):
        '''Runs search algorithm with given board state'''
        import time
        import resource
        parent_node = Node()
        parent_node.board_state = self.board_state
        output_file = 'output.txt'

        result = None
        #print 'Searching...'
        if self.method == 'bfs':
            start_time = time.time()
            result = bfs(parent_node, goal_test)
            max_ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024*1024)

        if self.method == 'dfs':
            start_time = time.time()
            result = dfs(parent_node, goal_test)
            max_ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024*1024)

        if self.method == 'ast':
            start_time = time.time()
            result = ast(parent_node, goal_test)
            max_ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024*1024)

        if self.method == 'ida':
            start_time = time.time()
            result = ida(parent_node, goal_test)
            max_ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / (1024*1024)

        if result:
            path_to_goal = 'path_to_goal:{}'.format(full_path(result.get('goal_node').action))
            print path_to_goal
            cost_of_path = 'cost_of_path:{}'.format(result.get('goal_node').path_cost)
            print cost_of_path
            nodes_expanded = 'nodes_expanded:%d' % result.get('nodes_expanded')
            print nodes_expanded
            fringe_size = 'fringe_size:%d' % result.get('fringe_size')
            print fringe_size
            max_fringe_size = 'max_fringe_size:%d' % result.get('max_fringe_size')
            print max_fringe_size
            search_depth = 'search_depth:%d' % result.get('goal_node').depth
            print search_depth
            max_search_depth = 'max_search_depth:%d' % result.get('max_search_depth')
            print max_search_depth
            running_time = time.time() - start_time
            print 'running_time:', running_time
            print 'max_ram_usage:', max_ram_usage

            try:
                with open(output_file, 'w') as f:
                    from os import linesep
                    f.write(path_to_goal + linesep)
                    f.write(cost_of_path + linesep)
                    f.write(nodes_expanded + linesep)
                    f.write(fringe_size + linesep)
                    f.write(max_fringe_size + linesep)
                    f.write(search_depth + linesep)
                    f.write(max_search_depth + linesep)
                    f.write('running_time:{}'.format(running_time) + linesep)
                    f.write('max_ram_usage:{}'.format(max_ram_usage) + linesep)
                    f.close()
            except Exception as e:
                print e

        else:
            print 'Solution was not found'


def full_path(path):
    path = path.split(',')
    full = []

    for action in path:
        if action == 'U':
            full.append('Up')
        if action == 'L':
            full.append('Left')
        if action == 'R':
            full.append('Right')
        if action == 'D':
            full.append('Down')

    return full


def successors(state, method=''):
    # find blank position
    # generators ??? could also work
    blank_pos = get_blank_pos(state)
    # find possible moves from blank position
    moves = possible_moves(blank_pos)

    if method == 'dfs' or method == 'ida':
        moves = moves[::-1]

    # print 'Possible moves :', moves, 'From:', blank_pos
    children = []
    for move in moves:
        board_state = make_move(state, blank_pos, move)
        if board_state is not None:
            children.append({'action': move, 'state': board_state})
    return children


def get_blank_pos(state):
    '''Returns position of the blank space'''
    blank = 0
    for i, r in enumerate(state):
        try:
            c = r.index(blank)
            return (i, c)
        except ValueError as e:
            pass
    return (-1, -1)


def possible_moves(blank_pos):
    '''Maintains a map of all blank_pos and possible moves '''
    # position - move mapping
    position_move_mapping = {
        (0, 0): ('D', 'R'),
        (0, 1): ('D', 'L', 'R'),
        (0, 2): ('D', 'L'),
        (1, 0): ('U', 'D', 'R'),
        (1, 1): ('U', 'D', 'L', 'R'),
        (1, 2): ('U', 'D', 'L'),
        (2, 0): ('U', 'R'),
        (2, 1): ('U', 'L', 'R'),
        (2, 2): ('U', 'L')
    }

    if blank_pos in position_move_mapping:
        return position_move_mapping[blank_pos]

    return None


def make_move(state, blank_pos, move):
    '''blank_pos to new position on given move'''
    import copy
    # R = i + 1
    # L = j - 1
    # D = i + 1
    # U = i - 1
    blank_tile = state[blank_pos[0]][blank_pos[1]]

    if move is 'U':
        adj_tile_pos = (blank_pos[0] - 1, blank_pos[1])

    if move is 'D':
        adj_tile_pos = (blank_pos[0] + 1, blank_pos[1])

    if move is 'L':
        adj_tile_pos = (blank_pos[0], blank_pos[1] - 1)

    if move is 'R':
        adj_tile_pos = (blank_pos[0], blank_pos[1] + 1)

    # print 'Adjacent tile position:', adj_tile_pos
    try:
        adj_tile = state[adj_tile_pos[0]][adj_tile_pos[1]]
        nstate = copy.deepcopy(state)
        # swap adj tile and blank space
        nstate[blank_pos[0]][blank_pos[1]] = adj_tile
        nstate[adj_tile_pos[0]][adj_tile_pos[1]] = blank_tile
        # print blank_pos, adj_tile_pos
        return nstate
    except IndexError as e:
        print e
        return None


def goal_test(state):
    goal_states = [
        [[1, 2, 3], [4, 5, 6], [7, 8, 0]],
        [[0, 1, 2], [3, 4, 5], [6, 7, 8]],
        [[1, 2, 3], [8, 0, 4], [7, 6, 5]]
    ]
    return state in goal_states


def bfs(parent_node, goal_test):
    ''' Returns a solution or failure'''
    import Queue
    frontier = Queue.Queue()
    explored = []
    frontier.put(parent_node)

    nodes_expanded = 0
    fringe_size = 0
    max_fringe_size = 0
    max_search_depth = 0

    while not frontier.empty():
        max_fringe_size = frontier.qsize()
        # Remove node from the frontier
        node = frontier.get()
        # Is it a goal state ?
        if goal_test(node.board_state):
            fringe_size = frontier.qsize()
            return {
                'goal_node': node,
                'max_fringe_size': max_fringe_size,
                'nodes_expanded': nodes_expanded,
                'fringe_size': fringe_size,
                'max_search_depth': max_search_depth
            }
        explored.append(node.board_state)
        # if not expand the node
        node.expand()
        nodes_expanded += 1
        # add its neighbours in the frontier
        for neighbour in node.neighbours:
            # if neighbour not in frontier.queue and neighbour not in explored:
            if neighbour.board_state not in explored:
                frontier.put(neighbour)
                max_search_depth = neighbour.depth

    return None


def dfs(problem, goal_test, depth_limit=0):
        ''' Returns a solution or failure'''
        import Queue
        frontier = Queue.LifoQueue()
        explored = []
        frontier.put(problem)

        nodes_expanded = 0
        fringe_size = 0
        max_fringe_size = 0
        max_search_depth = 0

        while not frontier.empty():
            max_fringe_size = frontier.qsize()
            # Remove node from the frontier
            node = frontier.get_nowait()
            # Is it a goal state ?
            if goal_test(node.board_state):
                fringe_size = frontier.qsize()
                return {
                    'goal_node': node,
                    'max_fringe_size': max_fringe_size,
                    'nodes_expanded': nodes_expanded,
                    'fringe_size': fringe_size,
                    'max_search_depth': max_search_depth
                }
            explored.append(node.board_state)
            # if not expand the node
            node.expand('dfs')
            nodes_expanded += 1

            # add its neighbours in the frontier
            for neighbour in node.neighbours:
                ''' IDA depth limit implementation '''
                # print depth_limit
                # print neighbour.depth
                if depth_limit and neighbour.depth >= depth_limit:
                    return None
                # if neighbour not in frontier.queue and neighbour not in explored:
                if neighbour.board_state not in explored:
                    if goal_test(neighbour.board_state):
                        fringe_size = frontier.qsize()
                        return {
                            'goal_node': neighbour,
                            'max_fringe_size': max_fringe_size,
                            'nodes_expanded': nodes_expanded,
                            'fringe_size': fringe_size,
                            'max_search_depth': max_search_depth
                        }
                    else:

                        frontier.put_nowait(neighbour)
                        max_search_depth = neighbour.depth

        return None


def ast(parent_node, goal_test):
    ''' Returns a solution or failure'''
    import Queue
    frontier = Queue.PriorityQueue()
    explored = []
    frontier.put(parent_node)

    nodes_expanded = 0
    fringe_size = 0
    max_fringe_size = 0
    max_search_depth = 0

    while not frontier.empty():
        max_fringe_size = frontier.qsize()
        # Remove node from the frontier
        node = frontier.get()
        # Is it a goal state ?
        if goal_test(node.board_state):
            fringe_size = frontier.qsize()
            return {
                'goal_node': node,
                'max_fringe_size': max_fringe_size,
                'nodes_expanded': nodes_expanded,
                'fringe_size': fringe_size,
                'max_search_depth': max_search_depth
            }
        explored.append(node.board_state)
        # if not expand the node
        node.expand()
        nodes_expanded += 1
        # add its neighbours in the frontier
        for neighbour in node.neighbours:
            # if neighbour not in frontier.queue and neighbour not in explored:
            if neighbour.board_state not in explored:
                frontier.put(neighbour)
                max_search_depth = neighbour.depth

    return None


def ida(parent_node, goal_test):
    ''' Iterative deeping search algorithm implementation
        - returns a solution or failure
        - implements DFS algorithm with a depth limit
     '''
    depth_limit = 0 + h_one(parent_node.board_state)

    for i in range(1, depth_limit + 1):
        result = dfs(parent_node, goal_test, i)

        if result:
            return result

    return None


def h_one(state):
    ''' h_one function
        - Calculates total number of tiles in wrong position from goal state
    '''
    goal_state = [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    goal = []
    # honizontally convert goal state
    for i in goal_state:
        goal.extend(i)

    tiles = []
    for i in state:
        tiles.extend(i)

    #print goal
    #print tiles
    h = 0
    for i, tile in enumerate(tiles):
        try:
            diff = goal.index(tile) - tiles.index(tile)
            if diff != 0:
                # h = h + abs(diff) + 0
                h += 1
        except Exception as e:
            pass

    return h

if __name__ == '__main__':
    solver = Solver(sys.argv[1], sys.argv[2])
    solver.solve()
