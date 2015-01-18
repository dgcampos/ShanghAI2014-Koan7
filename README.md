ShanghAI2014-Koan7
==================


About
-----

This project contains the group work for Koan 7 of the ShanghAI Lectures 2014. The Koan's topic is: Evolution of locomotion gait in modular robots.

#Dependencies#
In order to run this work, the following dependencies are needed:

 * [Webots](http://www.cyberbotics.com/overview)
 * [CMake](http://www.cmake.org/)
 * [Evolutionary Computing Framework (ECF)](http://gp.zemris.fer.hr/ecf/)

Since it uses fork() and execv() to launch webots in each evaluation, this code only works on Unix systems.


#Structure#
This code is divided into two main parts: 

###1.Evolution###
The evolution part is stored in the *evolution* folder. This part is in charge of the evolution algorithm, which calls Webots to evaluate each individual. The communication between Webots and this program is made via text files, with the parameters for each module and fitness value.

This part requires to have CMake and ECF installed on our computer. To compile this program, once the dependencies are installed is as simple as:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

The program will be placed on the *bin* folder. To run the program, we need the configuration file stored in *config*:

    $ ./evolve-gaits ../config/evolution_parameters.xml

Different aspects of the evolution, such as the algorithm or the number of individuals can be selected by modifying the configuration file.

###2.Evaluation of gaits###
The evaluation of gaits is performed using Webots for simulation. Each time an individual needs to be evaluated, Webots is called with a *LocomotionSupervisor* supervisor controlling the execution of the simulation. Each controller loads the required oscillator parameters from its corresponding text file and runs for the specified evaluation time.

When the evaluation is finished, the travelled distance is written to a text file, that will be read by the evolution algorithm.

