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
	def __init__(self, x=0, y=0, z=0):
		self.x = x
		self.y = y
		self.z = z

	def to_tuple(self):
		return (self.x, self.y, self.z)