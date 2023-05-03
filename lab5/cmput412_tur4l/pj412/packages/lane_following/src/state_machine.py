"""
defines all states needed to execute the entire project
"""

class BotState:
    
    def __init__(self, goal_stall):
        """
        goal_stall (int) - between 1 and 4, specifies the goal stall for parking
        """
        self.stateid_expected_tags = {
            # part1 (ends when the robot exits the circle)
            0: [48, 56],  # before first apriltag
            # go straight
            10: [48],
            11: [48],
            12: [48],
            13: [50],
            # turn right
            20: [50],
            21: [50],
            22: [50],
            23: [56],

            # part2 (ends when the robot sees the parking lot entry)
            30: [163],  # before first crosswalk
            31: [163],  # before second crosswalk
            32: [38],  # before seeing the entry apriltag to the parking lot

            # part3 (have a separate variable to indicate which stall is the goal)
            40: [227],  # before seeing the tag inside the parking lot
            41: [207, 226, 228, 75],  # moving forward towards the goal stall
            42: [207, 226, 228, 75],  # turn around 180 degrees
            43: [207, 226, 228, 75],  # backing into the goal stall
        }
        self.goal_stall = goal_stall
        self.stateid = 0

    def is_legal_stateid(self, stateid):
        return stateid in self.stateid_expected_tags

    def get_expected_tags(self):
        return self.stateid_expected_tags[self.stateid]

