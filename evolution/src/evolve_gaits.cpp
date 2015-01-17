//--
//-- Evolution program
//--
//-- Communicates with Webots to evaluate the fitness of each individual
//-- and takes care of all the evolution process
//--

#include <ecf/ECF.h>
#include "ModularRobotEvalOp.h"


int main(int argc, char **argv)
{
    StateP state (new State);

    // set the evaluation operator
    state->setEvalOp(new ModularRobotEvalOp);

    state->initialize(argc, argv);
    state->run();

    return 0;
}
