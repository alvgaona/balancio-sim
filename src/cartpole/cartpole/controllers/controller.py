from abc import ABC, abstractmethod


class Controller(ABC):
    @abstractmethod
    def compute(self, x: float):
        pass
