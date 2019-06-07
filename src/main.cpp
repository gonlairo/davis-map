#include <iostream>
#include "MapRouter.h"
#include <fstream>
#include "StringUtils.h"
#include <vector>

int main(int argc, char *argv[])
{

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
            std::cout << 
            "findroute [--data=path | --resutls=path]\n"
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
        else
        {
            const char RoutesCSVFileData[] = "route,stop_id\n"
                                             "A,20\n"
                                             "A,21\n"
                                             "A,23\n";

            std::ifstream osm("data/davis.osm");
            std::ifstream stops("data/stops.csv");
            std::ifstream routes("data/routes.csv");
            
            CMapRouter MapRouter;
            MapRouter.LoadMapAndRoutes(osm, stops, routes);
            auto vstr = StringUtils::Split(InputLine, " ");
            //for(auto x : vstr) std::cout << x << std::endl;
            

            if(InputLine == "count")
            {
                std::cout << MapRouter.NodeCount() << " nodes" << std::endl;
            }
            else if (vstr[0] == "node")
            {
                int node_index = stoi(InputLine.substr(5));
                auto nodeid = MapRouter.GetSortedNodeIDByIndex(node_index);
                auto latlon = MapRouter.GetSortedNodeLocationByIndex(node_index);

                std::cout << "Node " << node_index << ": id = " << nodeid << " is at " << std::get<0>(latlon) << ", " << std::get<1>(latlon) << std::endl;
            }

            else if (vstr[0] == "fastest")
            {
                auto startid = std::stoul(vstr[1]);
                auto endid = std::stoul(vstr[2]);
                std::vector <CMapRouter::TPathStep> path;
                auto fastest_time = MapRouter.FindFastestPath(startid, endid, path);
                std::cout << "Fastest path takes " << fastest_time << " h/m/s" << std::endl;
            }           
            else if (vstr[0] == "shortest")
            {
                auto startid = std::stoul(vstr[1]);
                auto endid = std::stoul(vstr[2]);
                std::vector<CMapRouter::TNodeID> path;
                auto shortest_distance = MapRouter.FindShortestPath(startid, endid, path);
                std::cout << "Shortest path is " << shortest_distance << "mi" << std::endl;
            }
        }    
    }
}
