
/*
 * This is just a test program that simulates Webots for testing the evolution part
 * while the webots controller is being developed
 */

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

static const std::string input_file_folder = "../../controllers/LocomotionController/";
static const std::string input_file_prefix = "module_";
static const int n_modules = 8;

int main(int argc, char * argv[])
{
    std::cout << "[dummy-answer] Called with arguments: " << std::endl;

    for (int i = 1; i < argc; i++)
    {
        std::cout << "\t ->" << argv[i] << std::endl;
    }

    std::cout << "[dummy-answer] Reading parameters and returning fitness..." << std::endl;

    //-- Read parameters from imput files:
    double parameter_sum = 0;

    for (int i = 0; i < n_modules; i++)
    {
        std::stringstream ss;
        ss << input_file_folder + input_file_prefix << i+1 << ".txt";

        std::ifstream input_file(ss.str().c_str() );

        if (input_file.is_open())
        {
            for(int j = 0; j < 4; j++)
            {
                double aux;
                input_file >> aux;
                parameter_sum += aux;
            }

            input_file.close();
        }
        else
        {
            std::cerr << "[dummy-answer] File could not be opened!" << std::endl;
        }
    }

    //-- Write then to fitness file:
    std::cout << "[dummy-answer] Writing fitness: " << parameter_sum << std::endl;

    std::ofstream output_file((input_file_folder+"fitness.txt").c_str());
    if (output_file.is_open())
    {
        output_file << 1 / (double) parameter_sum;
        output_file.close();
    }
    else
    {
        std::cerr << "[dummy-answer] File could not be opened!" << std::endl;
    }

    return 0;
}
