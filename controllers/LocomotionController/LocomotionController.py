import sys
import numpy
from controller import Robot


class LocomotionController(Robot):
    """
    This defines a controller for the Yamor robot.
    """

    def __init__(self, amplitude, offset, initial_phase, frequency):
        """
        Initializes the controller's motor which behaves like an oscillator and creates two connectors.

        :param amplitude: amplitude of the oscillator between 0 and 60 degrees
        :param offset: offset of the oscillator between -15 and +15 degrees
        :param initial_phase: phase of the oscillator between 0 and 360 degrees
        :param frequency: frequency of the oscillator between 0 and 1.5 Hz
        """

        Robot.__init__(self)

        self.motor = self.getMotor('motor')
        self.amplitude = 1.0
        self.offset = 0.0
        self.initial_phase = 0.0
        self.frequency = 1.0
        self.configure_oscillator(amplitude, offset, initial_phase, frequency)

        self.rear_connector = self.getConnector('rear_connector')
        self.front_connector = self.getConnector('front_connector')

    def configure_oscillator(self, amplitude, offset, initial_phase, frequency):
        """
        Configures the oscillator.

        :param amplitude: amplitude of the oscillator between 0 and 60 degrees
        :param offset: offset of the oscillator between -15 and +15 degrees
        :param initial_phase: phase of the oscillator between 0 and 360 degrees
        :param frequency: frequency of the oscillator between 0 and 1.5 Hz
        """

        # Convert degrees to radians
        self.amplitude = numpy.pi / 180.0 * amplitude
        self.offset = numpy.pi / 180.0 * offset
        self.initial_phase = numpy.pi / 180.0 * initial_phase

        self.frequency = frequency

    def move_oscillator(self, current_time):
        """
        Moves the oscillator.
        """
        self.motor.setPosition(self.offset + self.amplitude * numpy.sin(self.frequency * (self.initial_phase + current_time)))

    def run(self, step_size=64):
        """
        Runs the controller until ?.
        """

        # Initialize the simulation time
        t = 0.0

        # Perform a simulation step and leave the loop when the simulation is over.
        while self.step(step_size) != -1:

            self.move_oscillator(t)
            t += step_size/1000.0

if __name__ == "__main__":
    # Extract the parameters set by the supervisor from the program parameters
    if len(sys.argv) == 5:
        controller = LocomotionController(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
        controller.run()
    else:
        sys.exit(-1)
