#include "MapRouter.h"
#include <cmath>
#include <iostream>
#include "CSVReader.h"
#include "CSVWriter.h"
#include "XMLReader.h"
#include "XMLEntity.h"
#include <vector>
#include <map>
#include <tuple>
#include <string>
#include <limits>
#include "StringUtils.h"
const CMapRouter::TNodeID CMapRouter::InvalidNodeID = -1;

CMapRouter::CMapRouter(){
}

CMapRouter::~CMapRouter(){
}


double CMapRouter::HaversineDistance(double lat1, double lon1, double lat2, double lon2)
{
    auto DegreesToRadians = [](double deg){return M_PI * (deg) / 180.0;};
	double LatRad1 = DegreesToRadians(lat1);
	double LatRad2 = DegreesToRadians(lat2);
	double LonRad1 = DegreesToRadians(lon1);
	double LonRad2 = DegreesToRadians(lon2);
	double DeltaLat = LatRad2 - LatRad1;
	double DeltaLon = LonRad2 - LonRad1;
	double DeltaLatSin = sin(DeltaLat/2);
	double DeltaLonSin = sin(DeltaLon/2);	
	double Computation = asin(sqrt(DeltaLatSin * DeltaLatSin + cos(LatRad1) * cos(LatRad2) * DeltaLonSin * DeltaLonSin));
	const double EarthRadiusMiles = 3959.88;
	
	return 2 * EarthRadiusMiles * Computation;
}        

double CMapRouter::CalculateBearing(double lat1, double lon1,double lat2, double lon2){
    auto DegreesToRadians = [](double deg){return M_PI * (deg) / 180.0;};
    auto RadiansToDegrees = [](double rad){return 180.0 * (rad) / M_PI;};
    double LatRad1 = DegreesToRadians(lat1);
	double LatRad2 = DegreesToRadians(lat2);
	double LonRad1 = DegreesToRadians(lon1);
	double LonRad2 = DegreesToRadians(lon2);
    double X = cos(LatRad2)*sin(LonRad2-LonRad1);
    double Y = cos(LatRad1)*sin(LatRad2)-sin(LatRad1)*cos(LatRad2)*cos(LonRad2-LonRad1);
    return RadiansToDegrees(atan2(X,Y));
}

bool CMapRouter::LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes)
{
    CXMLReader OsmReader(osm);
    SXMLEntity OsmEnt;
    OsmReader.ReadEntity(OsmEnt, true);
    if ((OsmEnt.DType != SXMLEntity::EType::StartElement) or (OsmEnt.DNameData != "osm"))
    {
        return false;
    }
    while (!OsmReader.End())
    {
        OsmReader.ReadEntity(OsmEnt, true);
        if (OsmEnt.DType == SXMLEntity::EType::StartElement)
        {
            if (OsmEnt.DNameData == "node")
            {
                Node TempNode;
                TempNode.NodeID = std::stoul(OsmEnt.AttributeValue("id"));
                TempNode.location.first = std::stod(OsmEnt.AttributeValue("lat"));
                TempNode.location.second = std::stod(OsmEnt.AttributeValue("lon"));

                VNodes.push_back(TempNode);
                VSortedIds.push_back(TempNode.NodeID);

                int size = MNodeIds.size();
                MNodeIds[TempNode.NodeID] = size;
            }
            else if (OsmEnt.DNameData == "way")
            {
                std::vector<Node> TempStorage;
                double speedlimit = 25;
                bool oneway = true;
                OsmReader.ReadEntity(OsmEnt, true); // skip way id

                while ((OsmEnt.DType != SXMLEntity::EType::EndElement) || (OsmEnt.DNameData != "way"))
                {
                    if ((OsmEnt.DType == SXMLEntity::EType::StartElement) && (OsmEnt.DNameData == "nd"))
                    {
                        TNodeID id = std::stoul(OsmEnt.AttributeValue("ref"));
                    
                        //IGNORE NODE IF WE DIDNT READ IT BEFORE
                        if (!MNodeIds.count(id))
                        {
                            OsmReader.ReadEntity(OsmEnt, true);
                            std::cout << "id not found: " << id << std::endl;
                            std::cout << "name: " << OsmEnt.DNameData << std::endl;
                            //id = std::stoul(OsmEnt.AttributeValue("ref"));
                            //std::cout << "id new: " << id << std::endl;
                        }
                        else{
                            int index = MNodeIds[id];
                            Node CurrNode = VNodes[index];
                            TempStorage.push_back(CurrNode);
                            OsmReader.ReadEntity(OsmEnt, true);
                        }

                        


                    }
                    else if ((OsmEnt.DType == SXMLEntity::EType::StartElement) && OsmEnt.DNameData == "tag")
                    {
                        if (OsmEnt.AttributeValue("k") == "maxspeed")
                        {
                            speedlimit = std::stod(OsmEnt.AttributeValue("v"));
                        }
                        if (OsmEnt.AttributeValue("k") == "oneway")
                        {
                            oneway = OsmEnt.AttributeValue("v") == "yes" ? true : false;
                        }

                        OsmReader.ReadEntity(OsmEnt, true);
                    }
                    else
                    {
                        OsmReader.ReadEntity(OsmEnt, true);
                        
                    }
                }

                
                // CHECK OUT OF BOUNDS (- 1 WORKS?)
                for (int i = 0; i < TempStorage.size() - 1; i++)
                {
                    int walking_speed = 3;
                    Edge connection;

                    connection.distance = HaversineDistance(TempStorage[i].location.first, TempStorage[i].location.second,
                                                            TempStorage[i + 1].location.first, TempStorage[i + 1].location.second);

                    //std::cout << "distance between " << TempStorage[i].NodeID << " and " << TempStorage[i + 1].NodeID << " is: " << connection.distance << std::endl;
                    connection.time = connection.distance/walking_speed;
                    connection.speed = speedlimit;
                    connection.oneway = oneway;
                    connection.busedge = false;

                    TNodeID current = TempStorage[i].NodeID;
                    TNodeIndex current_index = MNodeIds[current];
                    TNodeID next = TempStorage[i + 1].NodeID;
                    TNodeIndex next_index = MNodeIds[next];

                    connection.nodeid = next;
                    connection.nodeindex = next_index;
                    VNodes[MNodeIds[current]].vedges.push_back(connection); // check

                    if (oneway == false)
                    {
                        connection.nodeid = current;
                        connection.nodeindex = current_index;
                        VNodes[MNodeIds[next]].vedges.push_back(connection); // check
                    }
                }
            }
        }
    }

    CCSVReader StopsReader(stops);
    std::vector <std::string> TempStops;
    StopsReader.ReadRow(TempStops); // header

    while (!StopsReader.End())
    {
        StopsReader.ReadRow(TempStops);
        TStopID stopid = std::stoul(TempStops[0]);
        TNodeID nodeid = std::stoul(TempStops[1]);
        int index = MNodeIds[nodeid];

        MTStopNodeIds[stopid] = nodeid;
        MTNodeStopIds[nodeid] = stopid;
        VNodes[index].stop = stopid;
    }

    CCSVReader RoutesReader(routes);
    std::vector<std::string> TempRoutes;
    RoutesReader.ReadRow(TempRoutes);
    
    while (!RoutesReader.End())
    {
        RoutesReader.ReadRow(TempRoutes);
        std::string BusName = TempRoutes[0];
        TStopID stopid = std::stoul(TempRoutes[1]);
        MBusRoutes[BusName].push_back(stopid);
    }
    
    for(auto kv : MBusRoutes)
    {
        auto Vstops = kv.second;
        std::vector< TNodeID > temppath;

        for(int i = 0; i < Vstops.size() - 1; i++)
        {
            TNodeID start = MTStopNodeIds[Vstops[i]];
            TNodeID dest = MTStopNodeIds[Vstops[i+1]];
            std::cout << "start!: " << start << std::endl;
            std::cout << "end!: " << dest << std::endl;
            auto fastest = dijkstras(start, dest, temppath, 2);
            std::cout << "end: " << dest << std::endl;
            BusEdgeInfo temp;
            temp.time = fastest + 1.0/120.0;
            temp.path = temppath;
            temp.RouteNames.push_back(kv.first);
            
            double distance;
            for (int z = 0; z < temppath.size(); z++)
            {
                int index = MNodeIds[temppath[z]];
                int index_next = MNodeIds[temppath[z + 1]];
                auto edges = VNodes[index].vedges;
                for (int j = 0; j < edges.size(); j++)
                {
                    if (edges[j].nodeid == VNodes[index_next].NodeID)
                    {
                        distance += edges[j].distance;
                        
                    }
                }
            }
            
            temp.distance = distance;
            MStopSteps[std::make_tuple(start, dest)] = temp;
        }
    }

    for(auto kv : MStopSteps){
        auto first = kv.first;
        BusEdgeInfo second = kv.second;
        TNodeID startid = std::get<0>(first);
        Edge busedge;

        busedge.nodeid = std::get<1>(first);
        busedge.nodeindex = MNodeIds[std::get<1>(first)];
        busedge.distance = second.distance;
        busedge.time = second.time;
        busedge.busedge = true;
        VNodes[MNodeIds[startid]].vedges.push_back(busedge);
    }

    return true;
}

size_t CMapRouter::NodeCount() const{
    return VSortedIds.size();
}

CMapRouter::TNodeID CMapRouter::GetSortedNodeIDByIndex(size_t index) const{
    return VSortedIds[index];
}

CMapRouter::TLocation CMapRouter::GetSortedNodeLocationByIndex(size_t index) const{
    if (index >= VNodes.size())
    {
        return std::make_pair(180.0, 360.0);
    }
    
    auto search = MNodeIds.find(VSortedIds[index]);
    return VNodes[search->second].location;
}

CMapRouter::TLocation CMapRouter::GetNodeLocationByID(TNodeID nodeid) const{
    int index = MNodeIds.at(nodeid);
    auto Node = VNodes[index];
    return Node.location;
}

CMapRouter::TNodeID CMapRouter::GetNodeIDByStopID(TStopID stopid) const{
    return MTStopNodeIds.at(stopid);
}

size_t CMapRouter::RouteCount() const{
    return MBusRoutes.size();
}

std::string CMapRouter::GetSortedRouteNameByIndex(size_t index) const{
    auto it = MBusRoutes.begin();
    std::advance(it, index); // this is O(n)
    return it->first;
}

bool CMapRouter::GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops){
    stops = MBusRoutes[route];
    return true;
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    return dijkstras(src, dest, path, 0);
}


double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path)
{
    std::vector <TNodeID> route;
    auto fastest = dijkstras(src, dest, route, 1);
    print_vector(route);

    path.push_back(std::make_pair("Walk",route[0]));
    for (int i = 0; i < route.size() - 1; i++)
    {
        auto start = route[i];
        auto start_index = MNodeIds[start];
        auto next = route[i + 1];
        auto next_index = MNodeIds[next];

        if(MStopSteps.end() != MStopSteps.find(std::make_tuple(start, next)))
        {
            auto name = "Bus " + MStopSteps[(std::make_tuple(start, next))].RouteNames[0];
            auto inside_path = MStopSteps[(std::make_tuple(start, next))].path;
            for (int j = 1; j < inside_path.size(); j++)
            {
                path.push_back(std::make_pair(name, inside_path[j]));
            } 
        }
        else
        {
            path.push_back(std::make_pair("Walk", next));
        }  
    }

    for (auto el : path)
    {
        std::cout << el.first << " " << el.second << std::endl;
    }

    return fastest;
}

std::string direction(double degrees)
{
    if (degrees > 0 && degrees < 180)
    {
        return "E";
    }
    else if (degrees > 180 && degrees < 360)
    {
        return "W";
    }
    else if (degrees == 90 && degrees == 360)
    {
        return "N";
    }
    else if (degrees == 180)
    {
        return "S";
    }
}

std::string deg_to_DMS(double degrees){
    
    std::string ddeg = std::to_string(degrees);
    std::vector<std::string> temp = StringUtils::Split(".", ddeg);
    std::string deg = temp[0] + 'd';
    std::string min = std::to_string(std::stoul(temp[1])*60.0) + "\'";
    temp = StringUtils::Split(".", min);
    std::string sec = std::to_string(std::stoul(temp[1])*60.0) + "\"";

    return deg + min + sec;

}

bool CMapRouter::GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const
{
    auto start = path[0].second;
    auto start_index = MNodeIds.at(start);
    auto start_node = VNodes[start_index];

    auto dirA = direction(start_node.location.first);
    auto dirB = direction(start_node.location.second);
    auto locA = deg_to_DMS(start_node.location.first);
    auto locB = deg_to_DMS(start_node.location.second);

    std::string sstart = "Start at " + locA + " " + dirA + ", " + locB + " " + dirB;
    desc.push_back(sstart);

    for (int i = 1; i < path.size(); i++)
    {
        auto start = path[i].second;
        auto start_index = MNodeIds.at(start);
        auto start_node = VNodes[start_index];
        auto next = path[i + 1].second;
        auto next_index = MNodeIds.at(next);
        auto next_node = VNodes[next_index];

        auto bearing = CalculateBearing(start_node.location.first, start_node.location.second,
                                        next_node.location.first, next_node.location.second);

        auto dir = deg_to_DMS(bearing);

        std::cout << "hey" << std::endl;
        if(path[i].first == "Walk")
        {
            auto dirA = direction(next_node.location.first);
            auto dirB = direction(next_node.location.second);
            std::string start = "Walk " + dir + " to "  + locA + " " + dirA + ", " + locB + " " + dirB;
            desc.push_back(start);
        }
        else
        {
            continue;
        }
        std::string ret;
        // std::cout << "sssize: " << v.size() << std::endl;
        
        for (int i = 0; i < desc.size(); i++)
        {
            std::cout << desc[i] << " ";
        }

        std::cout << std::endl;
    }
}



double CMapRouter::dijkstras(TNodeID src, TNodeID dest, std::vector<TNodeID> &path, int method)
{
    std::cout << __LINE__ << std::endl;
    const int infinity = std::numeric_limits<int>::max();
    TNodeIndex src_index = MNodeIds[src];
    TNodeIndex dest_index = MNodeIds[dest];

    typedef std::pair<double, TNodeIndex> npair; // pair of weight / TNodeIndex
    std::priority_queue<npair, std::vector<npair>, std::greater<npair>> pq;


    std::vector<double> dist(VNodes.size(), infinity);
    std::vector<TNodeIndex> prev(VNodes.size());
    std::vector<bool> visited(VNodes.size(), false);

    pq.push(std::make_pair(0, src_index));
    dist[src_index] = 0;
    prev[src_index] = src_index;
    
    while (!pq.empty())
    {
        std::cout << __LINE__ << std::endl;
        TNodeIndex current_index = pq.top().second;
        
        // if(MNodeIds.find(current.NodeID) == MNodeIds.end()) std::cout << "HEY!!!!!!" << std::endl;
        pq.pop();
        if(visited[current_index])
        {
            continue;
        }
        visited[current_index] = true;
        
        if(current_index == dest_index) break;

        for (auto edge : VNodes[current_index].vedges)
        {
            double altdist = dist[current_index] + (method == 1 ? edge.time : (method == 0 ? edge.distance : edge.distance/edge.speed));   
            if (altdist < dist[edge.nodeindex])
            {
                dist[edge.nodeindex] = altdist;
                prev[edge.nodeindex] = current_index;
                pq.push(std::make_pair(altdist, edge.nodeindex));
            }   
        }  
    }

    if(dist[dest_index] == infinity)
    {
        std::cout << "bad" << std::endl;
        exit(0);
    } 

    // for (int i = 0; i < visited.size(); i++)
    // {
    //     if(visited[i]) std::cout << "Node index: " << i << " visited" << std::endl;
    // }
    

    path.clear();
    TNodeIndex Dindex = dest_index;
    std::cout << __LINE__ << std::endl;
    path.insert(path.begin(), VNodes[Dindex].NodeID);
    std::cout << VNodes[Dindex].NodeID << std::endl;
    
    while (Dindex != src_index)
    {
        std::cout << __LINE__ << " " << Dindex << std::endl;
        path.insert(path.begin(), VNodes[prev[Dindex]].NodeID);
        Dindex = prev[Dindex];
        //print_vector_double(prev);
    }

    return dist[dest_index];
}







void CMapRouter::print_vector(std::vector<TStopID> v)
{
    std::string ret;
   // std::cout << "sssize: " << v.size() << std::endl;
    for (int i = 0; i < v.size(); i++)
    {
        std::cout << v[i] << " ";
    }

    std::cout << std::endl;
}
void CMapRouter::print_vector_double(std::vector<double> v)
{
    std::string ret;
   // std::cout << "sssize: " << v.size() << std::endl;
    for (int i = 0; i < v.size(); i++)
    {
        std::cout << v[i] << " ";
    }

    std::cout << std::endl;
}

void CMapRouter::print_vector_string(std::vector<std::string> v)
{
    std::string ret;
    // std::cout << "sssize: " << v.size() << std::endl;
    for (int i = 0; i < v.size(); i++)
    {
        std::cout << v[i] << " ";
    }

    std::cout << std::endl;
}

void CMapRouter::print_vector_bool(std::vector <bool> v)
{
    std::string ret;
    // std::cout << "sssize: " << v.size() << std::endl;
    for (int i = 0; i < v.size(); i++)
    {
        std::cout << v[i] << " ";
    }

    std::cout << std::endl;
}