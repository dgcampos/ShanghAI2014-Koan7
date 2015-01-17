//--
//-- ModularRobotEvalOp
//--
//-- Function evaluator (objective function) for the Modular Robot
//--

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>

#include <ecf/ECF.h>


//-- This will allow us to request the same mssg queue for both programs
#define MSSG_QUEUE_KEY 0

/*!
 *  \class ModularRobotEvalOp
 *  \brief Function evaluator (objective function) for the Modular Robot
 */
class ModularRobotEvalOp : public EvaluateOp
{
    public:

        ~ModularRobotEvalOp();

        //! \brief Loads custom-made registry entries to the ECF
        bool initialize(StateP state);

        //! \brief This adds the custom-made registry entries to the ECF
        void registerParameters( StateP state);

        //!\brief Objective function
        FitnessP evaluate(IndividualP individual);

    protected:

        /***** Constants to bound the oscillator values *****/
        int max_offset;
        float max_amp_0_5;
        float max_pha_0_5;
        float max_freq_0_5;

        //-- Parameters read from config file
        int n_modules;
        unsigned long max_runtime;
        float timestep;
        std::string simulator_command;
        std::string parameter_files_folder;
        std::string parameter_files_prefix;

 private:
        //! \brief Extract the oscillator parameters encoded in the genotype and write them in files
        void recordParameters(FloatingPoint::FloatingPoint *genotype);

};
