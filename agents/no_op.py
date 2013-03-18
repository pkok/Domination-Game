class Agent(object):
    NAME = "no_op"

    def __init__(self, *args, **kwargs):
        pass

    def observe(self, observation):
        pass

    def action(self):
        return (0, 0, False)

    def debug(self, surface):
        pass

    def finalize(self, interrupted=False):
        pass
