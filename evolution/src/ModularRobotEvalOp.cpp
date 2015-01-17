//------------------------------------------------------------------------------
//-- ModularRobotEvalOp
//------------------------------------------------------------------------------
//--
//-- Function evaluator (objective function) for the Modular Robot
//--
//------------------------------------------------------------------------------
//--
//-- This file belongs to the Hormodular project
//-- (https://github.com/David-Estevez/hormodular.git)
//--
//------------------------------------------------------------------------------
//-- Author: David Estevez-Fernandez
//--
//-- Released under the GPL license (more info on LICENSE.txt file)
//------------------------------------------------------------------------------

#include "ModularRobotEvalOp.h"




void ModularRobotEvalOp::registerParameters(StateP state)
{
    state->getRegistry()->registerEntry("robot.modules", (voidP) (new uint(1)), ECF::UINT, "Number of modules" );
    state->getRegistry()->registerEntry("robot.runtime", (voidP) (new uint(10000)), ECF::UINT, "Max robot runtime (ms)" );
    state->getRegistry()->registerEntry("robot.timestep", (voidP) (new float(1.0)), ECF::FLOAT, "Time step (ms)" );
    state->getRegistry()->registerEntry("robot.simulationCommand", (voidP) (new std::string()), ECF::STRING, "Command to run the simulator");
    state->getRegistry()->registerEntry("robot.parameterFilesFolder", (voidP) (new std::string()), ECF::STRING, "Folder to output the parameter files");
    state->getRegistry()->registerEntry("robot.parameterFilesPrefix", (voidP) (new std::string()), ECF::STRING, "Prefix of the parameter files");

    state->getRegistry()->registerEntry("osc.maxamplitude", (voidP) (new uint(90)), ECF::UINT, "Max amplitude of oscillators");
    state->getRegistry()->registerEntry("osc.maxoffset", (voidP) (new uint(90)), ECF::UINT, "Max offset of oscillators");
    state->getRegistry()->registerEntry("osc.maxphase", (voidP) (new uint(360)), ECF::UINT, "Max phase of oscillators");
    state->getRegistry()->registerEntry("osc.maxfrequency", (voidP) (new float(1.0f)), ECF::FLOAT, "Max frequency of oscillators");
}

ModularRobotEvalOp::~ModularRobotEvalOp()
{

}

bool ModularRobotEvalOp::initialize(StateP state)
{
    //-- Get the robot values from the registry:
    //-----------------------------------------------------------------------------------------
    //-- Number of modules:
    voidP sptr = state->getRegistry()->getEntry("robot.modules");
    n_modules = *((uint*) sptr.get() );
    std::cout << "[Evolve] Info: Loaded \"robot.modules\"="<< n_modules << std::endl;

    //-- Max runtime:
    sptr = state->getRegistry()->getEntry("robot.runtime");
    max_runtime =(unsigned long) *((uint*) sptr.get() );
    std::cout << "[Evolve] Info: Loaded \"robot.runtime\"="<< max_runtime << std::endl;

    //-- Time step:
    sptr = state->getRegistry()->getEntry("robot.timestep");
    timestep = *((float*) sptr.get() );
    std::cout << "[Evolve] Info: Loaded \"robot.timestep\"="<< timestep << std::endl;

    //-- Simulator command:
    sptr = state->getRegistry()->getEntry("robot.simulationCommand");
    simulator_command = *((std::string*) sptr.get() );
    std::cout << "[Evolve] Info: Loaded \"robot.simulationCommand\"=\""<< simulator_command << "\"" << std::endl;

    //-- Parameter files folder:
    sptr = state->getRegistry()->getEntry("robot.parameterFilesFolder");
    parameter_files_folder = *((std::string*) sptr.get() );
    std::cout << "[Evolve] Info: Loaded \"robot.parameterFilesFolder\"=\""<< parameter_files_folder << "\"" << std::endl;

    //-- Simulator command:
    sptr = state->getRegistry()->getEntry("robot.parameterFilesPrefix");
    parameter_files_prefix = *((std::string*) sptr.get() );
    std::cout << "[Evolve] Info: Loaded \"robot.parameterFilesPrefix\"=\""<< parameter_files_prefix << "\"" << std::endl;


    //-- Get the oscillator parameters from the registry:
    //---------------------------------------------------------------------------------------
    //-- Max amplitude:
    sptr = state->getRegistry()->getEntry("osc.maxamplitude");
    int max_amplitude = *((uint*) sptr.get());
    max_amp_0_5 = max_amplitude / 2.0;
    std::cout << "[Evolve] Info: Loaded \"osc.maxamplitude\"="<< max_amplitude << std::endl;

    //-- Max offset:
    sptr = state->getRegistry()->getEntry("osc.maxoffset");
    max_offset = *((uint*) sptr.get());
    std::cout << "[Evolve] Info: Loaded \"osc.maxoffset\"="<< max_offset << std::endl;

    //-- Max phase:
    sptr = state->getRegistry()->getEntry("osc.maxphase");
    int max_phase = *((uint*) sptr.get());
    max_pha_0_5 = max_phase / 2.0;
    std::cout << "[Evolve] Info: Loaded \"osc.maxphase\"="<< max_phase << std::endl;

    //-- Max frequency:
    sptr = state->getRegistry()->getEntry("osc.maxfrequency");
    float max_frequency = *((float*) sptr.get());
    max_freq_0_5 = max_frequency / 2.0;
    std::cout << "[Evolve] Info: Loaded \"osc.maxfrequency\"="<< max_frequency << std::endl;


    /* TODO: Configure other parts of the robot */

    return true;
}

FitnessP ModularRobotEvalOp::evaluate(IndividualP individual)
{
    //-- Create a fitness object to maximize the objective (distance travelled in m)
    FitnessP fitness (new FitnessMax);

    //-- We get the genotype:
    FloatingPoint::FloatingPoint* genotype = (FloatingPoint::FloatingPoint*) individual->getGenotype(0).get();

    //-- Value to store the fitness (distance travelled)
    double fitness_value = 0;

    /* TODO Convert the values of the genotype from [-1, 1] to be within each parameter limits
     * and record them in different files for different modules. */
    recordParameters(genotype);

    /* TODO Start webots to evaluate the individual */
    int pid = fork();

    if (pid == -1)
    {
        std::cerr << "Error forking the process..." << std::endl;
        exit(1);
    }
    else if (pid == 0)
    {
        //-- Call simulator
        char *arguments[3];

        arguments[0] = new char[simulator_command.size()+1];
        strcpy(arguments[0], simulator_command.c_str());

        std::stringstream ss;
        ss << max_runtime;
        arguments[1] = new char[ss.str().size()+1];
        strcpy(arguments[1], ss.str().c_str());

        std::stringstream ss2;
        ss2 << timestep;
        arguments[2] = new char[ss2.str().size()+1];
        strcpy(arguments[2], ss2.str().c_str());

        execve(arguments[0], arguments, NULL);
    }



    /* TODO Wait for webots response */
    int status;
    wait(&status);



    //-- Set the fitness value
    fitness->setValue( fitness_value);

    std::cout << "[Evolve] Return!" << std::endl;
    return fitness;
}

void ModularRobotEvalOp::recordParameters(FloatingPoint::FloatingPoint *genotype)
{
    //-- Convert the values from [-1, 1] to correct intervals and save them
    std::vector<float> amplitudes;
    std::vector<float> offsets;
    std::vector<float> phases;
    float frequency;

    for(int i = 0; i < (int) n_modules; i++)
    {
        float amplitude = genotype->realValue[i*3] * max_amp_0_5 + max_amp_0_5;
        float offset = genotype->realValue[i*3+1] * max_offset;
        float phase = genotype->realValue[i*3+2] * max_pha_0_5 + max_pha_0_5;

        amplitudes.push_back(amplitude);
        offsets.push_back(offset);
        phases.push_back(phase);
    }

    frequency = genotype->realValue[n_modules*3] * max_freq_0_5 + max_freq_0_5;

    //-- Record all values to files
    for (int i=0; i < n_modules; i++)
    {
        //-- Open file
        std::stringstream ss;
        ss << parameter_files_folder+parameter_files_prefix << i+1 << ".txt";

        std::ofstream file( ss.str().c_str());

        if (!file.is_open())
        {
            std::cerr << "Error opening file... " << std::endl;
            return;
        }

        //-- Save values
        file << amplitudes[i] << std::endl;
        file << offsets[i] << std::endl;
        file << phases[i] << std::endl;
        file << frequency;

        //-- Close file
        file.close();
    }

    return;
}
