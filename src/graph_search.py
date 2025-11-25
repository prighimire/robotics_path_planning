import numpy as np
from .graph import Cell
from .utils import trace_path
from collections import deque

""" 
General graph search instructions:

First, define the correct data type to keep track of your visited cells
and add the start cell to it. If you need to initialize any properties
of the start cell, do that too.

Next, implement the graph search function. When you find a path, use the
trace_path() function to return a path given the goal cell and the graph. You
must have kept track of the parent of each node correctly and have implemented
the graph.get_parent() function for this to work. If you do not find a path,
return an empty list.

To visualize which cells are visited in the navigation webapp, save each
visited cell in the list in the graph class as follows:
     graph.visited_cells.append(Cell(cell_i, cell_j))
where cell_i and cell_j are the cell indices of the visited cell you want to
visualize.
"""


def depth_first_search(graph, start, goal):
    """Depth First Search (DFS) algorithm. This algorithm is optional for P3.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement DFS (optional)."""

    # If no path was found, return an empty list.
    return []


def breadth_first_search(graph, start, goal):
    """Breadth First Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    # Queue for BFS
    q = deque()

    # Start indices
    si, sj = start.i, start.j
    gi, gj = goal.i, goal.j

    # Mark start as visited and set its parent to None
    if not graph.is_cell_in_bounds(si, sj) or not graph.is_cell_in_bounds(gi, gj):
        # Invalid start or goal
        return []

    if graph.check_collision(si, sj) or graph.check_collision(gi, gj):
        # Start or goal in collision
        return []

    graph.visited[sj, si] = True
    graph.parent[sj][si] = None
    q.append(Cell(si, sj))

    # Record start in visited_cells for visualization
    graph.visited_cells.append(Cell(si, sj))

    while q:
        current = q.popleft()
        ci, cj = current.i, current.j

        # Check if we've reached the goal
        if ci == gi and cj == gj:
            # ✅ Correct order: first the goal cell, then the graph
            return trace_path(goal, graph)

        # Explore neighbors
        neighbors = graph.find_neighbors(ci, cj)
        for ni, nj in neighbors:
            # Skip cells in collision
            if graph.check_collision(ni, nj):
                continue

            # If not visited, add to queue
            if not graph.visited[nj, ni]:
                graph.visited[nj, ni] = True
                graph.parent[nj][ni] = Cell(ci, cj)  # parent of neighbor is current

                # Record neighbor in visited_cells (for visualization order)
                graph.visited_cells.append(Cell(ni, nj))

                q.append(Cell(ni, nj))

    # If we exit the loop, no path was found
    return []


def a_star_search(graph, start, goal):
    """A* Search (BFS) algorithm.
    Args:
        graph: The graph class.
        start: Start cell as a Cell object.
        goal: Goal cell as a Cell object.
    """
    graph.init_graph()  # Make sure all the node values are reset.

    """TODO (P3): Implement A*."""

    # If no path was found, return an empty list.
    return []
