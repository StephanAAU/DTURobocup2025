from orighelpers.spose import SPose
class systemVars():
    def __init__(self):
        self.var1 = 0
        self.var2 = 0
        self.itemInForc = False
        self.timeSTS = 0 #STS = since state start
        self.distancedrivenSTS = 0
        self.timesInLinefollow = 0
        self.following_line = False

        self.crossing_counter = 0

    
    def __str__ (self) -> str:
        res = ""
        for name, value in vars(self).items():  # Same as self.__dict__.items()
            res = res + f"{name} = {value}\n"
        return res