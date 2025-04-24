from abc import ABC, abstractmethod

from Statehelpers.systemVars import systemVars

class instruction(ABC):
    
    @abstractmethod
    def execute(self,systemvars:systemVars):
        pass