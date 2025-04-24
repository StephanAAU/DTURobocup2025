from __future__ import annotations

from Statehelpers.systemVars import systemVars
from Statehelpers.instruction import instruction

class Transition():
    
    def __init__(self, nextstate:"State", condition:function, instructions:list[function]):
        self.nextstate = nextstate
        self.condition = condition
        self.instructions = instructions

    def execute_instructions(self,systemvars:systemVars):
        for ins in self.instructions:
            ins()