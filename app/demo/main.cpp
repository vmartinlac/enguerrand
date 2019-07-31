#include <iostream>
#include "Engine.h"

int main(int num_args, char** args)
{
    if(num_args != 2)
    {
        std::cerr << "Bad command line!" << std::endl;
        exit(1);
    }

    EngineConfigPtr config(new EngineConfig());

    if( config->loadFromFile(args[1]) == false )
    {
        std::cerr << "Could not load configuration!" << std::endl;
        exit(1);
    }

    Engine engine;
    engine.exec(config);

    return 0;
}

