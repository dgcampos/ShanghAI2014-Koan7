import os
import pickle
import time
import numpy
import numpy.random
from controller import Supervisor


class LocomotionSupervisor(Supervisor):
    """
    This defines a controller that supervises the learning of locomotion based on developmental evolution.
    """

    STATE_SETUP_SIMULATION = 1
    STATE_RUN_SIMULATION = 2
    STATE_EVALUATE_SIMULATION = 3
    STATE_SETUP_SHOWCASE = 4
    STATE_RUN_SHOWCASE = 5
    STATE_SHUTDOWN = 6

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

        # Register fitness functions
        self.fitness_functions = {
            'velocity': self.evaluate_velocity,
            'distance': self.evaluate_distance
        }

        # Zero the robot position and timestamp
        self.robot_x = None
        self.robot_y = None
        self.robot_z = None
        self.time_record = None

        # Load configuration
        self.config = {}

        try:
            self.load_configuration()

        except IOError:
            # If the config file doesn't exist yet, setup an initial configuration
            self.reset_configuration(population_size)

    def load_configuration(self):
        """
        Loads the supervisor configuration which contains the current state and other useful information.
        """

        with open("supervisor_config.pkl", 'rb') as config_file:
            self.config = pickle.load(config_file)

    def save_configuration(self, suffix=""):
        """
        Saves the supervisor configuration.
        """

        with open("supervisor_config%s.pkl" % suffix, 'wb') as config_file:
            pickle.dump(self.config, config_file)

    def reset_configuration(self, population_size=10, trials=100, runtime=20.0, step_size=64):
        """
        Resets the supervisor configuration.
        """

        self.config = {
            'state': LocomotionSupervisor.STATE_SETUP_SIMULATION,
            'population_size': population_size,
            'population': numpy.empty((population_size, 8, 1, 4)),
            'trials': trials,
            'current_trial': 0,
            'runtime': runtime,
            'step_size': step_size,
            'showcase_time': 50.0,
            'fitness_criterium': 'distance',
            'competed_individuals': [],
            'current_individual': numpy.random.randint(population_size),
            'individuals_evaluated': 0,
            'fitness_values': numpy.zeros((population_size,))
        }

        self.save_configuration()

    def save_oscillator_configuration(self, active=True, runtime=0.0, step_size=64, genotype=numpy.empty((8, 1, 4))):
        """
        Saves an oscillator configuration to a file.
        """

        # Write the oscillator configuration, i.e. a genotype, to a file which can be loaded by the oscillator node(s)
        oscillator_configuration = {
            'active': active,
            'runtime': runtime,
            'step_size': step_size,
            'genotype': genotype
        }

        with open("./../LocomotionController/oscillator_config.pkl", 'wb') as config_file:
            pickle.dump(oscillator_configuration, config_file)

    def reset_oscillator_configuration(self):
        """
        Deletes an oscillator configuration file.
        """

        try:
            os.remove("./../LocomotionController/oscillator_config.pkl")

        except Exception as e:
            print(e)

    def reset_robot(self):
        """
        Resets the robot to its initial position and restarts all controllers including the supervisor.
        """

        self.simulationRevert()

    def evaluate_distance(self, t, distance):
        """
        Evaluates the fitness of an individual based on its distance in z-direction.

        :return: a value representing the fitness of an individual
        """

        x, y, z = self.modules[0].getField('translation').getSFVec3f()

        if self.robot_x is None or self.robot_y is None or self.robot_z is None or self.time_record is None:
            pass
        else:
            delta_distance = z - self.robot_z
            distance += delta_distance
            #print("delta_distance=%.2f, delta_z=%.2f, delta_t=%.2f" % (delta_distance, z-self.robot_z, t-self.time_record))
            #distance += numpy.sqrt((x-self.robot_x)**2 + (y-self.robot_y)**2 + (z-self.robot_z)**2) / (t-self.time_record)

        self.robot_x = x
        self.robot_y = y
        self.robot_z = z
        self.time_record = t

        return distance

    def evaluate_velocity(self, t, velocity):
        """
        Evaluates the fitness of an individual based on its mean velocity in z-direction.

        :return: a value representing the fitness of an individual
        """

        x, y, z = self.modules[0].getField('translation').getSFVec3f()

        if self.robot_x is None or self.robot_y is None or self.robot_z is None or self.time_record is None:
            pass
        else:
            delta_velocity = (z - self.robot_z) / (t - self.time_record)
            velocity += delta_velocity
            #print("delta_velocity=%.2f, delta_z=%.2f, delta_t=%.2f" % (delta_velocity, z-self.robot_z, t-self.time_record))
            #velocity += numpy.sqrt((x-self.robot_x)**2 + (y-self.robot_y)**2 + (z-self.robot_z)**2) / (t-self.time_record)

        self.robot_x = x
        self.robot_y = y
        self.robot_z = z
        self.time_record = t

        return velocity

    def crossover(self, winner, loser, crossover_probability=0.5):
        """
        Infects the loser with some genes of the winner.

        :param winner: index of the winner individual
        :param loser: index of the loser individual
        """

        # Crossover means the winner infects specific genes of the loser by chance
        crossover_function = lambda x, y, p: x if numpy.random.uniform() < p else y
        vector_crossover_function = numpy.vectorize(crossover_function)
        self.config['population'][loser] = vector_crossover_function(self.config['population'][winner], self.config['population'][loser], crossover_probability)

    def mutate(self, individual, average_mutations=1):
        """
        Randomly mutates some of the genes of the individual.

        :param individual: index of the genome of the individual
        """

        # In average mutate once per round
        mutation_probability = float(average_mutations) / float(self.config['population'][individual].shape[0])

        # Mutate specific genes by chance...
        mutation_function = lambda x, p: x + numpy.random.normal(loc=0.0, scale=0.21) if numpy.random.uniform() < p else x
        vector_mutation_function = numpy.vectorize(mutation_function)

        # ... on each gene in the genome of the individual
        self.config['population'][individual] = vector_mutation_function(self.config['population'][individual], mutation_probability)

    def setup_simulation(self):
        """
        Initializes the simulation.
        """

        print("Initialize simulation...")

        # Create population, i.e. a list with sets of different parameters for all 8 oscillators which represent the
        # genotype of an individual
        for i in xrange(self.config['population_size']):

            # Scale the parameters to fit the valid interval (see p. 86 of 'Learning locomotion gait through hormone-
            # based controller in modular robots'):
            # - amplitude in [0,60]
            # - offset in [-15, 15]
            # - initial phase in [0, 360]
            # - frequency in [0, 1.5]
            self.config['population'][i][0] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #1
            self.config['population'][i][1] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #2
            self.config['population'][i][2] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #3
            self.config['population'][i][3] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #4
            self.config['population'][i][4] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #5
            self.config['population'][i][5] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #6
            self.config['population'][i][6] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #7
            self.config['population'][i][7] = numpy.random.rand(1, 4) * numpy.array([60.0, 30.0, 360.0, 1.5]) + numpy.array([0.0, -15.0, 0.0, 0.0])  # parameters of oscillator #8

        # Set next state
        self.config['state'] = LocomotionSupervisor.STATE_RUN_SIMULATION

    def run_simulation(self):
        """
        Simulates one lifetime of an individual.
        """

        print("Run simulation...")

        # Load the genotype of the first individual
        genotype = self.config['population'][self.config['current_individual']]

        # Activate the oscillator configuration
        self.save_oscillator_configuration(active=True, runtime=self.config['runtime'], step_size=self.config['step_size'], genotype=genotype)

        # Initialize the simulation time and fitness
        t = 0.0
        fitness = 0.0

        while t <= self.config['runtime']:

            # Proceed one further simulation step
            self.step(self.config['step_size'])
            t += float(self.config['step_size']) / 1000.0

            # Measure the fitness over time
            fitness = self.fitness_functions[self.config['fitness_criterium']](t, fitness)

        # Evaluate the current individual
        self.config['fitness_values'][self.config['current_individual']] = numpy.abs(fitness)

        # Delete the oscillator configuration to avoid unnecessary simulation during next state
        self.reset_oscillator_configuration()

        # Set next state
        self.config['state'] = LocomotionSupervisor.STATE_EVALUATE_SIMULATION

    def evaluate_simulation(self):
        """
        Evaluates the performance of the current individual during the last simulation.
        """

        if self.config['individuals_evaluated'] < 2:

            print("Evaluated individual #%d with fitness %.2f..." % (self.config['current_individual'], self.config['fitness_values'][self.config['current_individual']]))

            # Log simulation progress
            self.config['individuals_evaluated'] += 1
            self.config['competed_individuals'] += [self.config['current_individual']]

            # Determine the next individual
            next_individual = self.config['current_individual']

            while next_individual == self.config['current_individual']:
                next_individual = numpy.random.randint(self.config['population_size'])

            self.config['current_individual'] = next_individual

        else:

            # Reset evaluation
            self.config['individuals_evaluated'] = 0

            # Compete the individuals
            a, b = self.config['competed_individuals']
            a_fitness = self.config['fitness_values'][a]
            b_fitness = self.config['fitness_values'][b]

            if a_fitness >= b_fitness:

                # a is the winner
                self.crossover(a, b)
                self.mutate(b)

                print("Completed trial %d/%d with individual #%d as winner and %d as loser..." % (self.config['current_trial']+1, self.config['trials'], a, b))
            else:

                # b is the winner
                self.crossover(b, a)
                self.mutate(a)

                print("Completed trial %d/%d with individual #%d as winner and %d as loser..." % (self.config['current_trial']+1, self.config['trials'], b, a))

            # Complete current trial
            self.config['current_trial'] += 1
            self.config['competed_individuals'] = []

        # Set next state
        if self.config['current_trial'] < self.config['trials']:
            self.config['state'] = LocomotionSupervisor.STATE_RUN_SIMULATION
        else:
            self.config['state'] = LocomotionSupervisor.STATE_SETUP_SHOWCASE

    def setup_showcase(self):
        """
        Prepares the showcase where the best individual is presented.
        """

        self.config['current_individual'] = numpy.argmax(self.config['fitness_values'])

        # Load the genotype of the best individual
        genotype = self.config['population'][self.config['current_individual']]

        # Write the oscillator configuration, i.e. a genotype, to a file which can be loaded by the oscillator node(s)
        oscillator_configuration = {
            'active': True,
            'runtime': self.config['runtime'],
            'step_size': self.config['step_size'],
            'genotype': genotype
        }

        with open("./../LocomotionController/oscillator_config.pkl", 'wb') as config_file:
            pickle.dump(oscillator_configuration, config_file)

        # Set next state
        self.config['state'] = LocomotionSupervisor.STATE_RUN_SHOWCASE

    def run_showcase(self):
        """
        Runs the showcase where the best individual is presented.
        """

        print("Run showcase...")

        # Initialize the simulation time
        t = 0.0

        while t <= self.config['showcase_time']:

            # Proceed one further simulation step
            self.step(self.config['step_size'])
            t += float(self.config['step_size']) / 1000.0

        # Set next state
        self.config['state'] = LocomotionSupervisor.STATE_SHUTDOWN

    def shutdown(self):
        """
        Stores the current config for replay and exits the whole simulation program.
        """

        self.save_configuration(suffix="_%s" % time.strftime("%Y-%m-%d_%H:%M:%S"))

        self.reset_configuration()

    def run(self):
        """
        Runs a differential evolution algorithm to optimize controller behavior towards optimal locomotion gates.
        """

        while True:
            if self.config['state'] is LocomotionSupervisor.STATE_SETUP_SIMULATION:
                self.setup_simulation()
                break

            if self.config['state'] is LocomotionSupervisor.STATE_RUN_SIMULATION:
                self.run_simulation()
                break

            if self.config['state'] is LocomotionSupervisor.STATE_EVALUATE_SIMULATION:
                self.evaluate_simulation()
                break

            if self.config['state'] is LocomotionSupervisor.STATE_SETUP_SHOWCASE:
                self.setup_showcase()
                break

            if self.config['state'] is LocomotionSupervisor.STATE_RUN_SHOWCASE:
                self.run_showcase()
                break

            if self.config['state'] is LocomotionSupervisor.STATE_SHUTDOWN:
                self.shutdown()
                return

        self.save_configuration()

        self.reset_robot()


if __name__ == "__main__":
    controller = LocomotionSupervisor(population_size=10)
    controller.run()