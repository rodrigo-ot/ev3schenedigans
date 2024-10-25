class Coordenada:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if isinstance(other, Coordenada):
            return self.x == other.x and self.y == other.y
        return False
    
    def __add__(self, other):
        if isinstance(other, Coordenada):
            return Coordenada(self.x + other.x, self.y + other.y)
        return NotImplemented
    
    def __sub__(self, other):
        if isinstance(other, Coordenada):
            return Coordenada(self.x - other.x, self.y - other.y)
        return NotImplemented