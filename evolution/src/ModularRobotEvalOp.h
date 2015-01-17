//--
//-- ModularRobotEvalOp
//--
//-- Function evaluator (objective function) for the Modular Robot
//--


#include <ecf/ECF.h>



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
        std::string config_file;

    private:
        //! \brief Extract the oscillator parameters encoded in the genotype and write them in files
        void genotypeToRobot(FloatingPoint::FloatingPoint* genotype);
};
