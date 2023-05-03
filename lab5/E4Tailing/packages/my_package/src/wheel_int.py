import math

TICK_TO_METER = 0.00165

def to_sec(time):
    return time.nsecs * 1e-9 + time.secs

class WheelPositionIntegration:
    def __init__(self, radius_millimeter, init_x=0.0, init_y=0.0, init_theta=0.0):
        self.x = init_x
        self.y = init_y
        self.theta = init_theta
        self.left = None
        self.leftList = []
        self.right = None
        self.rightList = []
        self.time = None
        self.radius = radius_millimeter


    def update_left(self, leftw, t):
        if self.left is None:
            rightTime = self.time
            self.time = t
            self.left = leftw
            if rightTime is not None:
                self.init_both_wheels(t, rightTime)
            return
        if t < self.time:
            print('received data out of order!')
            return
        
        self.leftList.append((t, leftw))
        self.int_position()
    
    def update_right(self, rightw, t):
        if self.right is None:
            leftTime = self.time
            self.time = t
            self.right = rightw
            if leftTime is not None:
                self.init_both_wheels(leftTime, t)
            return
        if t < self.time:
            print('received data out of order!')
            return
        
        self.rightList.append((t, rightw))
        self.int_position()
    
    def init_both_wheels(self, leftInitTime, rightInitTime):
        if leftInitTime < rightInitTime:
            self.rightList.append((rightInitTime, self.right))
            self.rightTime = leftInitTime
        else:
            self.leftList.append((leftInitTime, self.left))
            self.leftTime = rightInitTime

    def int_position(self):
        
        while len(self.leftList) >= 1 and len(self.rightList) >= 1:
            oldt = self.time
            newt = max(self.leftList[-1][0], self.rightList[-1][0])
            passt = (newt - oldt) * (2 / (len(self.leftList) + len(self.rightList)))
            leftTime, left = self.leftList[0]
            rightTime, right = self.rightList[0]
            # if oldt >= leftTime:
            #     del self.leftList[0]
            #     continue
            # if oldt >= rightTime:
            #     del self.rightList[0]
            #     continue
            del self.leftList[0]
            del self.rightList[0]

            # leftdt = leftTime - oldt
            # rightdt = rightTime - oldt
            # if leftTime < rightTime:
            #     q = leftdt / rightdt
            #     right = self.right * (1 - q) + right * q
            #     self.advance_time(leftTime, left, right)
            # else:
            #     q = rightdt / leftdt
            #     left = self.left * (1 - q) + left * q
            #     self.advance_time(rightTime, left, right)
            self.advance_time(oldt + passt, left, right)
    
    def advance_time(self, newt, newleft, newright):
        # dt = newt - self.time
        dleft = newleft - self.left
        dright = newright - self.right
        # dleft and dright are in millimeters, and dt is seconds
        
        distance = (dleft + dright) / 2
        dtheta = (dright - dleft) / self.radius / 2

        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        self.theta += dtheta

        self.time = newt
        self.left = newleft
        self.right = newright

    def get_state(self):
        return self.x, self.y, self.theta

    def get_state_meters(self):
        return self.x * TICK_TO_METER, self.y * TICK_TO_METER, self.theta
