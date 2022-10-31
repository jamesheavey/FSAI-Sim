import numpy as np
from scipy import interpolate

# All of this, is just an implementation of this:
# ref: http://phungdinhthang.com/2016/12/16/calculate-racing-lines-automatically/?i=1


def get_normal(point_a, point_b):
    v = (point_b[0] - point_a[0], point_b[1] - point_a[1])
    ln = float(np.linalg.norm(v))
    u = (v[0] / ln, v[1] / ln)

    return (-u[1], u[0])


def generate_offset_points(spline, resolution=100, distance=0.05):
    x, y = interpolate.splev(np.linspace(0, 1, int(resolution + 1)), spline)

    points = list(zip(list(x), list(y)))
    normals = [get_normal(point, points[i - 1])
               for i, point in enumerate(points[1:])]

    offset = [
        (
            point[0] + normal[0] * distance,
            point[1] + normal[1] * distance
        )
        for point, normal in zip(points, normals)]

    return list(zip(*offset))


class Line:
    def __init__(self, prev, points):
        self.prev = prev
        self.nodes = [Node(self, point) for point in points]
        self.next = None

        if self.prev:
            self.prev.set_next(self)

    def set_next(self, node):
        self.next = node


class Node:
    def __init__(self, line, point):
        self.line = line
        self.point = point
        self.rvm_value = {}
        self.rvm_next_node = {}


def generate_nodes(spline, width, res=50, segments=5):
    nodes = [
        list(zip(
            *generate_offset_points(spline, res,
                                    width / 2 - width / (segments - 1) * i)
        ))
        for i in range(segments)]

    nodes = list(zip(*nodes))

    return nodes


def fitness_function(prev, curr, nxt, alpha, beta):
    A = (curr[0] - prev[0], curr[1] - prev[1])
    B = (nxt[0] - curr[0], nxt[1] - curr[1])

    a = np.linalg.norm(A)
    b = np.linalg.norm(B)

    cos_p = (A[0] * B[0] + A[1] * B[1]) / (a * b)

    return alpha * cos_p - beta * (a + b)


def get_best_rvm(curr_line, alpha, beta):
    if not curr_line.next:
        return
    if not curr_line.prev:
        for curr_node, next_node in zip(curr_line.nodes, curr_line.next.nodes):
            curr_node.rvm_value = next_node.rvm_value
            curr_node.rvm_next_node = next_node.rvm_next_node
        return

    for i, node in enumerate(curr_line.nodes):
        for j, prev_node in enumerate(curr_line.prev.nodes):
            max_route_val = -np.inf
            max_node = None
            for k, next_node in enumerate(curr_line.next.nodes):
                this_route_val = fitness_function(
                    prev_node.point, node.point, next_node.point, alpha, beta) \
                    + next_node.rvm_value.get(i, 0)
                if this_route_val > max_route_val:
                    max_route_val = this_route_val
                    max_node = k
            node.rvm_value[j] = max_route_val
            node.rvm_next_node[j] = max_node


def walk_nodes(line, start_i):
    curr_i = start_i
    while line:
        node = line.nodes[curr_i]
        yield node.point
        curr_i = node.rvm_next_node.get(curr_i, 0)
        line = line.next


def calculate_racing_line(spline, width, res=50, segments=5, alpha=0.5, beta=0.5):    # noqa E501
    nodes = generate_nodes(spline, width, res, segments)

    lines = []
    for i, line in enumerate(nodes):
        lines.append(Line(lines[i - 1] if i != 0 else None, line))

    for line in reversed(lines):
        get_best_rvm(line, alpha, beta)

    start_index = max(
        range(segments), key=lambda i: lines[0].nodes[i].rvm_value[i]
    )

    paths = [list(walk_nodes(lines[0], i)) for i in range(segments)]

    best_path = list(walk_nodes(lines[1], start_index))  # TODO: auto select best loop path

    return nodes, paths, best_path


def to_spline(points):
    """
    Given a set of numpy points, calculate spline
    """
    x, y = points
    try:
        x_wrap = np.r_[x, x[0]]
        y_wrap = np.r_[y, y[0]]
        tck, _ = interpolate.splprep([x_wrap, y_wrap], s=0, per=True)
    except Exception:
        tck, _ = interpolate.splprep([x, y], s=0, per=True)
    return tck
