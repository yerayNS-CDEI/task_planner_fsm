class State:
    def __init__(self, name):
        self.name = name

    def on_enter(self, ctx):
        pass

    def run(self, ctx):
        pass

    def check_transition(self, ctx):
        return None

    def on_exit(self, ctx):
        pass
