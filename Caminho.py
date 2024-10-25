class Caminho:
    def __init__(self, coord1, coord2):
        self.coord1 = coord1
        self.coord2 = coord2

    def __eq__(self, other):
        if isinstance(other, Caminho):
            return ((self.coord1 == other.coord1 and self.coord2 == other.coord2) or
                    (self.coord1 == other.coord2 and self.coord2 == other.coord1))
        return False