import numpy
import numpy.random
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
        self.modules += [self.getFromDef('MODULE_1')]
        self.modules += [self.getFromDef('MODULE_2')]
        self.modules += [self.getFromDef('MODULE_3')]
        self.modules += [self.getFromDef('MODULE_4')]
        self.modules += [self.getFromDef('MODULE_5')]
        self.modules += [self.getFromDef('MODULE_6')]
        self.modules += [self.getFromDef('MODULE_7')]
        self.modules += [self.getFromDef('MODULE_8')]

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

        # Register fitness functions
        self.fitness_functions = {'velocity': self.evaluate_velocity}

        # Zero the robot position and timestamp
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.time_record = 0.0

    def evaluate_velocity(self):
        """
        Evaluates the fitness of an individual based on its velocity.

        :return: a value representing the fitness of an individual
        """

        x, y, z = self.modules[0].getField('translation').getSFVec3f()
        t = self.getTime()

        if self.robot_x is None or self.robot_y is None or self.robot_z is None:
            velocity = 0.0
        else:
            velocity = numpy.sqrt((self.robot_x-x)**2 + (self.robot_y-y)**2 + (self.robot_z-z)**2) / t

        self.robot_x = x
        self.robot_y = y
        self.robot_z = z
        self.time_record = t

        return velocity

    def compete_individuals(self, a, b, step_size=64, simulation_time=10, fitness_criterium='velocity'):
        """
        Simulates a competition between two individuals based on a specific fitness function.

        :param a: first individual
        :param b: second individual
        :param fitness_criterium: fitness function descriptor
        :return: winner, loser, fitness of the winner
        """

        # Initialize the oscillators with the parameters of the first individual
        for i, module in enumerate(self.modules):
            individual = self.population[a]
            module.getField('controllerArgs').setSFString("%f %f %f %f" % (individual[i][0], individual[i][1], individual[i][2], individual[i][3]))

        # Start the simulation of the first individual
        print("started simulation of first individual")

        duration = 0.0
        fitness_a = 0.0
        while True:
            # Simulate one further step
            self.step(step_size)
            duration += step_size

            # Quit the simulation if the simulation time is reached
            if duration / 1000 >= simulation_time:
                break

            # Evaluate the first individual
            fitness = self.fitness_functions[fitness_criterium]()
            fitness_a = fitness if fitness > fitness_a else fitness_a

        # Initialize the oscillators with the parameters of the second individual
        for i, module in enumerate(self.modules):
            individual = self.population[b]
            module.getField('controllerArgs').setSFString("%f %f %f %f" % (individual[i][0], individual[i][1], individual[i][2], individual[i][3]))

        # Start the simulation of the second individual
        print("started simulation of second individual")

        duration = 0.0
        fitness_b = 0.0
        while True:
            # Simulate one further step
            self.step(step_size)
            duration += step_size

            # Quit the simulation if the simulation time is reached
            if duration / 1000 >= simulation_time:
                break

            # Evaluate the first individual
            fitness = self.fitness_functions[fitness_criterium]()
            fitness_b = fitness if fitness > fitness_b else fitness_b

        # Determine the winner
        if fitness_a > fitness_b:
            print("individual a is the winner")
            return a, b, fitness_a
        else:
            print("individual b is the winner")
            return b, a, fitness_b

    def crossover(self, winner_index, loser_index, crossover_probability=0.5):
        """
        Infects the loser with some genes of the winner.

        :param winner_index: index of the winner individual
        :param loser_index: index of the loser individual
        """

        # Crossover means the winner infects specific genes of the loser by chance
        crossover_function = lambda x, y, p: x if numpy.random.uniform() < p else y
        vector_crossover_function = numpy.vectorize(crossover_function)
        self.population[loser_index] = vector_crossover_function(self.population[winner_index], self.population[loser_index], crossover_probability)

    def mutate(self, genome_index, average_mutations=1):
        """
        Randomly mutates some of the genes of the individual.

        :param genome_index: index of the genome of the individual
        """

        # In average mutate once per round
        mutation_probability = float(average_mutations) / float(self.population[genome_index].shape[0])

        # Mutate specific genes by chance...
        mutation_function = lambda x, p: x + numpy.random.normal(loc=0.0, scale=0.21) if numpy.random.uniform() < p else x
        vector_mutation_function = numpy.vectorize(mutation_function)

        # ... on each gene in the genome of the individual
        self.population[genome_index] = vector_mutation_function(self.population[genome_index], mutation_probability)

    def run(self, step_size=64, simulation_time=10):
        """
        Runs a differential evolution algorithm to optimize controller behavior towards optimal locomotion gates.

        :param step_size: the step size for the simulation
        :param simulation_time: the total duration of the simulation
        """

        # Try to optimize the locomotion controller
        for _ in xrange(1000):

            # Pick two individuals from the population
            a, b = numpy.random.choice(self.population.shape[0], 2)

            # Run a tournament
            winner, loser, winner_fitness = self.compete_individuals(a, b, step_size, simulation_time, fitness_criterium='velocity')

            # Crossover and mutate
            self.crossover(winner, loser)
            self.mutate(loser)

            # Report and store the current progress TODO
            print("Genome of current winner:")
            print("individual #%d with fitness %f" % (winner, winner_fitness))
            print(self.population[winner])
            print("")

        # Finally present the last winner
        while True:
            self.step()

if __name__ == "__main__":
    controller = LocomotionSupervisor(population_size=10)
    controller.run()