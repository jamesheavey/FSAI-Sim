import os
import json


def touch_folder(path):
    """
    Create target folder if not already exist
    """
    if not os.path.isdir(path):
        os.makedirs(path)


def touch_json(path):
    """
    Create target json if not already exist
    """
    if os.path.isfile(path):
        return
    with open(path, "w") as f:
        json.dump({}, f)
