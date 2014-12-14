import numpy
from controller import Supervisor


class LocomotionSupervisor(Supervisor):
    """
    This defines a controller that supervises the learning of locomotion based on developmental evolution.
    """

    def __init__(self, population_size=10):
        """
        Initializes supervisor.
        """

        Supervisor.__init__(self)

        self.root = self.getRoot()

        # Collect all available controller nodes
        self.modules = []
        self.modules += [self.getFromDef('MODULE_1').getField('controller')]
        self.modules += [self.getFromDef('MODULE_2').getField('controller')]
        self.modules += [self.getFromDef('MODULE_3').getField('controller')]
        self.modules += [self.getFromDef('MODULE_4').getField('controller')]
        self.modules += [self.getFromDef('MODULE_5').getField('controller')]
        self.modules += [self.getFromDef('MODULE_6').getField('controller')]
        self.modules += [self.getFromDef('MODULE_7').getField('controller')]
        self.modules += [self.getFromDef('MODULE_8').getField('controller')]

        # Initialize population
        self.population = numpy.empty((population_size, 8, 4))
        self.population_size = population_size

        # Create population, i.e. a list with sets of different parameters for all 8 oscillators which represent the
        # genotype of an individual
        for i in xrange(self.population_size):

            # Scale the parameters to fit the valid interval (see p. 86 of 'Learning locomotion gait through hormone-
            # based controller in modular robots'):
            # - amplitude in [0,60]
            # - offset in [-15, 15]
            # - initial phase in [0, 360]
            # - frequency in [0, 1.5]
            self.population[i][0] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #1
            self.population[i][1] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #2
            self.population[i][2] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #3
            self.population[i][3] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #4
            self.population[i][4] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #5
            self.population[i][5] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #6
            self.population[i][6] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #7
            self.population[i][7] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #8

    def compete_individuals(self, a, b, step_size=64, simulation_time=10, fitness='velocity'):
        """
        Simulates a competition between two individuals based on a specific fitness function.

        :param a: first individual
        :param b: second individual
        :param fitness: fitness function descriptor
        :return: winner, loser, fitness of the winner
        """

        # Initialize the oscillators with the first individual's parameters
        for i, module in enumerate(self.modules):
            module.getField('controllerArgs').setSFString("%f %f %f %f" % (a[i][0], a[i][1], a[i][2], a[i][3]))

        # Start the simulation of the first individual
        self.simulationRevert()

        duration = 0.0
        while self.step(step_size):
            duration += step_size
            if duration * 1000 >= simulation_time:
                break

            # Evaluate the first individual
            # TODO

        # Initialize the oscillators with the second individual's parameters
        for i, module in enumerate(self.modules):
            module.getField('controllerArgs').setSFString("%f %f %f %f" % (a[i][0], a[i][1], a[i][2], a[i][3]))

        # Start the simulation of the second individual
        self.simulationRevert()

        duration = 0.0
        while self.step(step_size):
            duration += step_size
            if duration * 1000 >= simulation_time:
                break

            # Evaluate the second individual
            # TODO

        # Determine the winner
        # TODO
        return a, b, 0.0

    def crossover(self, winner, loser):
        """
        Infects the loser with some genes of the winner.

        :param winner: the winner individual
        :param loser: the loser individual
        """

        # TODO
        pass

    def mutate(self, individual):
        """
        Randomly mutates some of the individual's genes.

        :param individual: the individual
        """

        # TODO
        pass

    def run(self, step_size=64, simulation_time=10):
        """
        Runs a differential evolution algorithm to optimize controller behavior towards optimal locomotion gates.
        :param step_size: the step size for the simulation
        :param simulation_time: the total duration of the simulation
        """

        # Pick two individuals from the population
        a, b = numpy.random.sample(self.population, 2)

        # Run a tournament
        winner, loser, winner_fitness = self.compete_individuals(a, b, step_size, simulation_time, fitness='velocity')

        # Crossover and mutate
        self.crossover(winner, loser)
        self.mutate(loser)

        # Report and store the current progress TODO


if __name__ == "__main__":
    controller = LocomotionSupervisor(population_size=10)
    controller.run()