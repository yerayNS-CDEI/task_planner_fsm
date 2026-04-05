from dataclasses import dataclass
from typing import Any, List

from task_planner_fsm.states.proc_utils import start_proc, stop_proc


@dataclass
class ProcessManager:
    ctx: dict

    def start(self, key: str, cmd: List[str]) -> None:
        start_proc(self.ctx, key, cmd)

    def stop(self, key: str) -> None:
        stop_proc(self.ctx, key)


@dataclass
class RosFacade:
    node: Any

    def logger(self):
        return self.node.get_logger()
