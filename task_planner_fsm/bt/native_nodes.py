import py_trees


class WaitForStartFlag(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, ctx):
        super().__init__(name=name)
        self._ctx = ctx

    def update(self):
        if self._ctx.get("start"):
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class StartManagedProcess(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, process_manager, key: str, cmd):
        super().__init__(name=name)
        self._process_manager = process_manager
        self._key = key
        self._cmd = cmd

    def update(self):
        self._process_manager.start(self._key, self._cmd)
        return py_trees.common.Status.SUCCESS


class StopManagedProcess(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, process_manager, key: str):
        super().__init__(name=name)
        self._process_manager = process_manager
        self._key = key

    def update(self):
        self._process_manager.stop(self._key)
        return py_trees.common.Status.SUCCESS
