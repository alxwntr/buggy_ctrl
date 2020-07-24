# Mock class for Pi gpio functions
# Only enough is implemented for what we need

class Motor:
    __slots__ = ["pins"]

    def __init__ (self, *pins):
        self.pins = pins

    def _drive (self, dr, duty):
        print("Motor %s driven %s duty %f" % 
            (repr(self.pins), dr, duty))

    def forward (self, duty):
        self._drive("forward", duty)

    def backward (self, duty):
        self._drive("backward", duty)
