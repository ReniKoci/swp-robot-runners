class Grid:
    # transform the map input file from JSON to correct format
    # symbols used in JSON format:
    # @ -> hard obstacle
    # T -> hard obstacle (for 'trees' in game env)
    # . -> free space
    # E -> emitter point (for 'delivery' goal)
    # S -> service point (for 'pick up' goal)

    # MAP EXAMPLE (delete later, only for reference)

    # type octile
    # height 33
    # width 57
    # map
    # @@@@..@@@....@@@....@@@....@@@....@@@....@@@....@@@..@@@@
    # @@@@.E@@@E..E@@@E..E@@@E..E@@@E..E@@@E..E@@@E..E@@@E.@@@@
    # @@@@..@@@....@@@....@@@....@@@....@@@....@@@....@@@..@@@@
    # @@@@.................................................@@@@
    # .........................................................
    # .E.......................................................

    def __init__(self):
        pass

    def get_grid(self):
        pass
