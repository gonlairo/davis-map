#include <iostream>
#include "MapRouter.h"

int main(int argc, char *argv[])
{
    if (argc >= 1)
    {
        for (int i = 0; i < argc; i++)
        {
            std::cout << argv[i] << std::endl;
        }
    }
    bool Done = false;
    
    while (!Done)
    {
        std::string InputLine;
        std::cout << "> ";
        std::getline(std::cin, InputLine);

        if (InputLine == "exit")
        {
            Done = true;
        }
        else if (InputLine == "help")
        {
            std::cout << "findroute [--data=path | --resutls=path]\n"
            "------------------------------------------------------------------------\n"
            "help     Display this help menu\n"
            "exit     Exit the program\n"
            "count    Output the number of nodes in the map\n"
            "node     Syntax \"node [0, count)\"\n"
            "         Will output node ID and Lat/Lon for node\n"
            "fastest  Syntax \"fastest start end\"\n"
            "         Calculates the time for fastest path from start to end\n"
            "shortest Syntax \"shortest start end\"\n"
            "         Calculates the distance for the shortest path from start to end\n"
            "save     Saves the last calculated path to file\n"
            "print    Prints the steps for the last calculated path\n"
             << std::endl;
        }
        else if (InputLine == "count")
        {
            CMapRouter MapRouter;
            MapRouter.LoadMapAndRoutes()
        }
        

    }
}
