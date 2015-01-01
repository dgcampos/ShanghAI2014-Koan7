import pickle
import time
import numpy
from controller import Robot


class LocomotionController(Robot):
    """
    This defines a controller for the Yamor robot.
    """

    def __init__(self):
        """
        Initializes the controller's motor which behaves like an oscillator and creates two connectors.
        """

        Robot.__init__(self)

        # Derive the oscillator id from its name, which is usually s.th. like "module_1"
        self.id = int(self.getName()[-1]) - 1

        # Get access to the motor
        self.motor = self.getMotor('motor')

        # Get access to the connectors
        self.rear_connector = self.getConnector('rear_connector')
        self.front_connector = self.getConnector('front_connector')

        # The configuration of the oscillator is stored in a file which can be changed by a supervisor
        self.config = {}

        try:
            time.sleep(0.5)
            self.load_configuration()

        except:
            self.config = {
                'active': False
            }

    def load_configuration(self):
        """
        Loads the configuration from a file.
        """

        with open("oscillator_config.pkl", 'rb') as config_file:
            self.config = pickle.load(config_file)

    def move_oscillator(self, t):
        """
        Moves the oscillator.

        :param t: current time step
        """

        amplitude = numpy.pi / 180.0 * self.config['genotype'][self.id][0][0]
        offset = numpy.pi / 180.0 * self.config['genotype'][self.id][0][1]
        initial_phase = numpy.pi / 180.0 * self.config['genotype'][self.id][0][2]
        frequency = self.config['genotype'][self.id][0][3]

        self.motor.setPosition(offset + amplitude * numpy.sin(frequency * (initial_phase + t)))

    def run(self):
        """
        Runs the controller until once with the current parameters. The supervisor node will rate this run and adapt the
        parameters accordingly.
        """

        if self.config['active'] is True:

            print("oscillator #%d started with amplitude=%.2f, offset=%.2f, initial_phase=%.2f and frequency=%.2f." % (self.id, self.config['genotype'][self.id][0][0], self.config['genotype'][self.id][0][1], self.config['genotype'][self.id][0][2], self.config['genotype'][self.id][0][3]))

            # Initialize the simulation time
            t = 0.0

            while t <= self.config['runtime']:

                # Proceed one further simulation step
                self.step(self.config['step_size'])
                t += self.config['step_size'] / 1000.0

                # Perform the simulation
                self.move_oscillator(t)

if __name__ == "__main__":
    controller = LocomotionController()
    controller.run()
