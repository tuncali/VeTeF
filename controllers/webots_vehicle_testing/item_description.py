"""Define ItemDescription class"""


class ItemDescription(object):
    """ItemDescription class holds a reference to a simulation item and possibly a state of the item"""
    ITEM_TYPE_TIME = 1
    ITEM_TYPE_VEHICLE = 2

    def __init__(self):
        self.item_type = None
        self.item_index = None
        self.item_state_index = None
