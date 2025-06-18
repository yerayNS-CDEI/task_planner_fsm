from state import State

class PhasedOperation(State):
    def __init__(self, name):
        super().__init__(name)
        self.started = False
        self.step_count = 0
        self.MAX_STEPS = 5  # máximo permitido antes de declarar error

    def on_enter(self, ctx):
        print(f"[{self.name}] Preparando operación...")
        self.started = False
        self.step_count = 0
        ctx["done"] = False
        ctx["error_triggered"] = False

    def run(self, ctx):
        if not self.started:
            print(f"[{self.name}] Ejecutando operación larga...")
            self.started = True
        else:
            print(f"[{self.name}] Supervisando paso {self.step_count + 1}...")
            self.step_count += 1

            if self.step_count >= self.MAX_STEPS:
                print(f"[{self.name}] ERROR: Se excedió el tiempo máximo.")
                ctx["error_triggered"] = True
            elif self.step_count == 3:  # simula éxito antes del límite
                ctx["done"] = True
                print(f"[{self.name}] Operación completada.")

    def check_transition(self, ctx):
        if ctx.get("done"):
            return "NextState"
        if ctx.get("error_triggered"):
            return "Error"
        return None
