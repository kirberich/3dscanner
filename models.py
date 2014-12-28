from bisect import bisect_left


class Box(object):
    def __init__(self, x1=None, y1=None, x2=None, y2=None):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2

    def to_tuple(self):
        return (self.x1, self.y1, self.x2, self.y2)

    def is_complete(self):
        return None not in self.to_tuple()


class Vertex(object):
    def __init__(self, x=0, y=0, z=0, index=None):
        self.x = x
        self.y = y
        self.z = z
        self.index = None

    def to_tuple(self):
        return (self.x, self.y, self.z)

    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        self.z += other.z
        return self

    def __div__(self, number):
        self.x /= number
        self.y /= number
        self.z /= number
        return self


class Frame(object):
    """ A frame is a collection of vertices. It behaves mostly like a list

        Use Frame.add_vertex to add vertices while making sure the list stays sorted by y-coordinates
    """
    def __init__(self):
        self._vertices = []
        self._vertex_keys = []
        self.num_vertices = 0
        self.current_vertex_index = 0

    def __getitem__(self, index):
        return self._vertices[index]

    def __iter__(self):
        self.current_vertex_index = 0
        for vertex in self._vertices:
            yield vertex
            self.current_vertex_index += 1

    def average(self, max_y_diff):
        """ Averages any points that have a y-distances of less than max_y_diff.
            The y distance is used because the frame's vertices are sorted by y """
        new_vertices = []
        sum_vertex = Vertex()
        average_count = 0
        vertex_index = 0
        last_vertex = Vertex()


        for vertex in self._vertices:
            if abs(vertex.y - last_vertex.y) < max_y_diff:
                sum_vertex += vertex
                average_count += 1
            else:
                if average_count:
                    sum_vertex /= average_count
                    sum_vertex.index = vertex_index
                    new_vertices.append(sum_vertex)
                else:
                    vertex.index = vertex_index
                    new_vertices.append(vertex)

                sum_vertex = Vertex()
                average_count = 0
                vertex_index += 1
            last_vertex = vertex

        if average_count:
            sum_vertex /= average_count
            sum_vertex.index = vertex_index
            new_vertices.append(sum_vertex)
            vertex_index += 1

        self._vertices = new_vertices
        self.num_vertices = vertex_index + 1
        self.current_vertex_index = vertex_index

    @property
    def next_vertex_index(self):
        return min(self.current_vertex_index + 1, self.num_vertices - 1)

    def next_vertex(self):
        return self._vertices[self.next_vertex_index]

    def get_vertex(self, index, or_last=False):
        if or_last:
            index = min(index, self.num_vertices - 1)
        return self._vertices[index]

    def append(self, vertex):
        if not isinstance(vertex, Vertex):
            raise TypeError("Only Vertices can be added to a frame.")

        index = bisect_left(self._vertex_keys, vertex.y)
        self._vertex_keys.insert(index, vertex.y)
        self._vertices.insert(index, vertex)
        self.num_vertices += 1

    def number_vertices(self, current_index):
        """ Number vertices in this frame, starting at current_index """
        for vertex in self:
            vertex.index = current_index
            current_index += 1
        return current_index


class Scan(object):
    def __init__(self):
        self._frames = []
        self.num_frames = 0
        self.current_frame_index = 0
        self.vertex_index = 1

    def __getitem__(self, index):
        return self._frames[index]

    def __iter__(self):
        self.current_frame_index = 0
        for frame in self._frames:
            yield frame
            self.current_frame_index += 1

    @property
    def next_frame_index(self):
        return (self.current_frame_index + 1) % self.num_frames

    @property
    def prev_frame_index(self):
        return self.current_frame_index - 1

    def next_frame(self):
        return self._frames[self.next_frame_index]

    def append(self, frame):
        if not isinstance(frame, Frame):
            raise TypeError("Only Frames can be added to a scan.")

        # Set vertex indices in frame
        self.vertex_index = frame.number_vertices(self.vertex_index)
        self._frames.append(frame)
        self.num_frames += 1
