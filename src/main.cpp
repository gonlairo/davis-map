#include <iostream>
#include "MapRouter.h"
#include <fstream>
#include "StringUtils.h"
#include <vector>
#include "CSVWriter.h"
#include <iomanip> 

int main(int argc, char *argv[])
{
    std::vector<std::string> lastcalculated;
    std::vector<CMapRouter::TNodeID> spath;
    std::vector<CMapRouter::TPathStep> fpath;

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
            

            if (InputLine == "count")
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
                lastcalculated = vstr;
                auto startid = std::stoul(vstr[1]);
                auto endid = std::stoul(vstr[2]);
                auto fastest_time = MapRouter.FindFastestPath(startid, endid, fpath);
                auto fminutes = fastest_time*60;
                lastcalculated.push_back(std::to_string(fastest_time));

                std::cout << "Fastest path takes " << std::setprecision(4) << fminutes << " min" << std::endl;
            }           
            else if (vstr[0] == "shortest")
            {
                lastcalculated = vstr;
                auto startid = std::stoul(vstr[1]);
                auto endid = std::stoul(vstr[2]);
                auto shortest_distance = MapRouter.FindShortestPath(startid, endid, spath);
                lastcalculated.push_back(std::to_string(shortest_distance));
                std::cout << "Shortest path is " << std::setprecision(3) << shortest_distance << "mi" << std::endl;
            }

            else if (vstr[0] == "save")
            {
                
                std::cout << lastcalculated.size() << std::endl;
                std::string outputfile = "results/" + lastcalculated[1] + "_" + lastcalculated[2] + "_" + lastcalculated[3] + ".csv";
                std::ofstream OutFile(outputfile);
                CCSVWriter CWrite(OutFile);

                if(lastcalculated[0] == "fastest")
                {
                    std::cout << __LINE__ << std::endl;
                    std::vector<std::string> header = {"mode", "node_id"};
                    CWrite.WriteRow(header);

                    std::vector<std::string> temp;
                    for(auto pair : fpath)
                    {   
                        temp.push_back(std::get<0>(pair));
                        auto temp_id = std::get<1>(pair);
                        temp.push_back(std::to_string(temp_id)); 
                        CWrite.WriteRow(temp);
                        temp.clear();
                    }
                }
                else
                { //shortest
                    std::cout << __LINE__ << std::endl;
                    std::vector<std::string> header = {"node_id"};
                    CWrite.WriteRow(header);

                    std::vector<std::string> temp;
                    for(auto nodeid : spath)
                    {   
                        temp.push_back(std::to_string(nodeid));
                        CWrite.WriteRow(temp);
                        temp.clear();
                    }

                }
                
            }
            else if (vstr[0] == "print")
            {
                std::vector<std::string> desc;
                if (lastcalculated[0] == "fastest")
                {
                    MapRouter.GetPathDescription(fpath, desc);
                    
                    for (int i = 0; i < desc.size(); i++)
                    {
                        std::cout << desc[i] << std::endl;
                    }
                }
                else
                {
                    auto shrtpath = MapRouter.ShortestPathToTPathStep(spath);
                    MapRouter.GetPathDescription(shrtpath, desc);

                    for (int i = 0; i < desc.size(); i++)
                    {
                        std::cout << desc[i] << std::endl;
                    }
                }
                
            }
            else
            {
                std::cout << "Invalid Input" << std::endl;
            }

        }    
    }
}
