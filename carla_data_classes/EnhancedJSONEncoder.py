import dataclasses, json
from enum import Enum


class EnhancedJSONEncoder(json.JSONEncoder):
    """
    Custom JSON encoder for dataclasses based on
    https://stackoverflow.com/questions/51286748/make-the-python-json-encoder-support-pythons-new-dataclasses
    """
    def default(self, o):
        if dataclasses.is_dataclass(o):
            return dataclasses.asdict(o)
        if isinstance(o, Enum):
            return o.name
        return super().default(o)
