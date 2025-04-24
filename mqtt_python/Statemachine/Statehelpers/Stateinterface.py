
from abc import ABC, abstractmethod

from Statehelpers.instruction import instruction
from Statehelpers.Transition import Transition
from Statehelpers.systemVars import systemVars
from typing import List
class State(ABC):
    def __init__(self,transitions:List[Transition],systemvars:systemVars):
        self.systemvars = systemvars
        self.transitions = transitions

    @abstractmethod
    def execute(self) -> Transition: # the main function of the state, return the transition to take when done with job use check if done
        pass

    def checkIfDone(self) -> Transition:
        for trans in self.transitions:
    
            if trans.condition():

                return trans
        else:
            return None