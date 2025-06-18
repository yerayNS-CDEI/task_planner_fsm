class StateMachine:
    def __init__(self, states, initial_state, ctx):
        self.states = {s.name: s for s in states}
        self.ctx = ctx
        self.current_state = self.states[initial_state]
        self.ctx["last_state"] = None   # initialization
        self.current_state.on_enter(self.ctx)

    def step(self):
        self.current_state.run(self.ctx)
        next_state = self.current_state.check_transition(self.ctx)
        if next_state and next_state in self.states:
            self.current_state.on_exit(self.ctx)
            self.ctx["last_state"] = self.current_state.name
            self.current_state = self.states[next_state]
            self.current_state.on_enter(self.ctx)
