from enum import Enum


class DataContactLocation(Enum):
    """
    Enum class for all DataContactLocation values
    """
    INVALID = 0
    UNKNOWN = 1
    LEFT = 2
    RIGHT = 3
    SUCCESSOR = 4
    PREDECESSOR = 5
    OVERLAP = 6

    def __eq__(self, other):
        return self.name == other.name
