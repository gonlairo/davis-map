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
#include <algorithm>

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
                bool oneway = false;
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
                            // std::cout << "id not found: " << id << std::endl;
                            // std::cout << "name: " << OsmEnt.DNameData << std::endl;
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

    unsigned long arr[] = {530445028, 5674111324, 5674111323, 480680078 ,265024842 ,3942353121, 5598433943, 3942353124 ,3942353116, 3942353115, 3942353114, 3942353113, 3942353112 ,3942353111, 3942353131 ,3942353132 ,5598433939, 3942353096 ,277046658, 95712770 ,3078996367 ,273245111, 566900815};
    // for(auto el : arr)
    // {
    //     auto nodeindex = MNodeIds[el];
    //     auto node = VNodes[nodeindex];
    //     std::cout << "NodeID: " << el << std::endl;

    //     for(auto x : node.vedges)
    //     {
    //         std::cout << x.nodeid << " ";
    //     }

    //     std::cout << std::endl;
    // }

        for(auto kv : MBusRoutes)
        {
            auto Vstops = kv.second;
            std::vector< TNodeID > temppath;

            for(int i = 0; i < Vstops.size() - 1; i++)
            {
                TNodeID start = MTStopNodeIds[Vstops[i]];
                TNodeID dest = MTStopNodeIds[Vstops[i+1]];
                auto fastest = dijkstras(start, dest, temppath, 2);
                BusEdgeInfo temp;
                temp.time = fastest + 1.0/120.0;
                temp.path = temppath;
                temp.RouteNames.push_back(kv.first);
                std::sort(temp.RouteNames.begin(), temp.RouteNames.end());

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

std::vector<std::string> intersection(std::vector<std::string> &v1,std::vector<std::string> &v2)
{
    std::vector<std::string> v3;
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());

    std::set_intersection(v1.begin(), v1.end(),
                          v2.begin(), v2.end(),
                          back_inserter(v3));
    return v3;
}

std::vector<CMapRouter::TNodeID> CMapRouter::unique_vector(std::vector<CMapRouter::TNodeID> &v)
{
    std::sort(v.begin(), v.end());
    auto last = std::unique(v.begin(), v.end());
    v.erase(last, v.end());

    return v;
}
 // 1 1 2 2 3 3 3 4 4 5 5 6 7

// v now holds {1 2 3 4 5 6 7 x x x x x x}, where 'x' is indeterminate


double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path)
{
    std::vector <TNodeID> route;
    auto fastest = dijkstras(src, dest, route, 1); 
    
    
    
    std::vector<TNodeID> exppath;
    std::vector <TNodeID> tempvect;
    std::vector<std::vector<std::string>> tvectintersect;
    std::string BusName;
    TNodeID start;
    TNodeID next;

    path.push_back(std::make_pair("Walk", route[0]));
    for (int i = 0; i < route.size(); i++)
    {
        start = route[i];
        std::cout << "start: " << start << std::endl;
        if (i < route.size() - 1)
        {
            next = route[i + 1];
            std::cout << "next: " << next << std::endl;
        }                        

        if (MStopSteps.end() != MStopSteps.find(std::make_tuple(start, next)))
        {
            auto v1routes = MStopSteps[(std::make_tuple(start, next))].RouteNames;
            auto v1path = MStopSteps[(std::make_tuple(start, next))].path;

            ++i;
            start = route[i];
            next = route[i + 1];
            
            if (MStopSteps.end() != MStopSteps.find(std::make_tuple(start, next))) // it is a bus stop
            {                
                auto v2routes = MStopSteps[(std::make_tuple(start, next))].RouteNames;
                auto v2path = MStopSteps[(std::make_tuple(start, next))].path;
                auto intersect = intersection(v1routes, v2routes);

                if (!intersect.empty())
                {
                    while (!intersect.empty()) // two buses coincide
                    {
                        tvectintersect.push_back(intersect);
                        auto prev = route[i - 1];
                        tempvect.push_back(prev);
                        tempvect.push_back(start);
                        tempvect.push_back(next);

                        ++i;
                        start = route[i];
                        next = route[i + 1];
                        v2routes = MStopSteps[(std::make_tuple(start, next))].RouteNames;
                        intersect = intersection(intersect, v2routes);
                    }

                    BusName = "Bus " + tvectintersect.back()[0];
                    auto vunique = unique_vector(tempvect);
                    for (int k = 0; k < vunique.size() - 1; k++)
                    {
                        auto inside_path = MStopSteps[(std::make_tuple(vunique[k], vunique[k + 1]))].path;
                        print_vector(inside_path);
                        
                        for (int t = 1; t < inside_path.size(); t++)
                        {
                            path.push_back(std::make_pair(BusName, inside_path[t]));
                        }                       
                    }
                }
                else // intersect is empty: two different buses!
                {
                    BusName = "Bus " + v1routes[0]; // asumming they are sorted
                    for (int j = 0; j < v1path.size(); j++)
                    {
                        path.push_back(std::make_pair(BusName, v1path[j]));
                    }

                    BusName += "Bus " + v2routes[0];
                    for (int j = 0; j < v2path.size(); j++)
                    {
                        path.push_back(std::make_pair(BusName, v2path[j]));
                    }
                }
            }
            else // it is not a bus stop
            {
                std::sort(v1routes.begin(), v1routes.end());
                auto name = "Bus " + v1routes[0];
                auto inside_path = MStopSteps[(std::make_tuple(start, next))].path;
                for (int j = 1; j < inside_path.size(); j++)
                {
                    path.push_back(std::make_pair(name, inside_path[j]));
                }
            }
        }   
        else
        {
            if (i == route.size() - 1) // if we are at the end push the START
            {
                path.push_back(std::make_pair("Walk", start));
            }
            else
            {
                path.push_back(std::make_pair("Walk", next));
            }
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
    if (degrees >= -22.5 && degrees < 27.5)
    {
        return "N";
    }
    else if (degrees >= 27.5 && degrees < 67.5)
    {
        return "NE";
    }
    else if (degrees >= 67.5 && degrees < 112.5)
    {
        return "E";
    }
    else if (degrees >= 112.5 && degrees < 157.5)
    {
        return "SE";
    }
    else if (degrees >= 157.5 && degrees < -157.5)
    {
        return "S";
    }
    else if (degrees >= -157.5 && degrees < -112.5)
    {
        return "SW";
    }
    else if (degrees >= -112.5 && degrees < -67.5)
    {
        return "W";
    }
    else if (degrees >= -67.5 && degrees < -22.5)
    {
        return "NW";
    }
    else
    {
        return "S";
    }
}

std::string deg_to_DMS(double degrees)
{
    std::string ddeg = std::to_string(degrees);
    std::vector<std::string> temp = StringUtils::Split(".", ddeg);
    std::string deg = temp[0] + 'd';
    std::string min = std::to_string(std::stoul(temp[1])*60.0) + "\'";
    temp = StringUtils::Split(".", min);
    std::string sec = std::to_string(std::stoul(temp[1])*60.0) + "\"";
    return deg + min + sec;

}

bool CMapRouter::GetPathDescription(const std::vector<TPathStep> &path, std::vector<std::string> &desc) const
{
    std::cout << __LINE__ << std::endl;
    auto start = path[0].second;
    auto start_index = MNodeIds.at(start);
    auto start_node = VNodes[start_index];
    auto dirA = direction(start_node.location.first);
    auto dirB = direction(start_node.location.second);
    auto locA = deg_to_DMS(start_node.location.first);
    auto locB = deg_to_DMS(start_node.location.second);
    std::string sstart = "Start at " + locA + " " + dirA + ", " + locB + " " + dirB;
    desc.push_back(sstart);
    
    for (int i = 1; i < path.size(); i++) //does this need to be -1
    {
        auto start = path[i].second;
        auto start_index = MNodeIds.at(start);
        auto start_node = VNodes[start_index];
        auto next = path[i + 1].second;
        auto next_index = MNodeIds.at(next);
        auto next_node = VNodes[next_index];

        auto bearing = CalculateBearing(start_node.location.first, start_node.location.second, next_node.location.first, next_node.location.second);

        auto curdir = direction(bearing); //changed
        if (path[i].first == "Walk")
        {
            auto dirA = direction(next_node.location.first);
            auto dirB = direction(next_node.location.second);
            auto locA = deg_to_DMS(next_node.location.first);
            auto locB = deg_to_DMS(next_node.location.second);
            std::string tempdesc = "Walk " + curdir + " to " + locA + " " + dirA + ", " + locB + " " + dirB;
            desc.push_back(tempdesc);
        }
        else
        {
            auto curinfo = StringUtils::Split(" ", path[i].first);
            auto curbus = curinfo[1];

            if (path[i + 1].first == "Walk")
            {
                std::string tempdesc = "Take" + path[i].first + "and get off at" + std::to_string(MTNodeStopIds.at(next_node.NodeID));
                desc.push_back(tempdesc);
            }
            else
            {
                auto nextbus = path[i + 1].first.substr(4, 5);
                if (curbus == nextbus)
                {
                    i++;
                }
                else
                {
                    std::string tempdesc = "Take" + path[i].first + "and get off at" + std::to_string(MTNodeStopIds.at(next_node.NodeID));
                    desc.push_back(tempdesc);                
                }
            }
        }
    }

    return true;
}

double CMapRouter::dijkstras(TNodeID src, TNodeID dest, std::vector<TNodeID> & path, int method)
    {
        //std::cout << __LINE__ << std::endl;
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
            TNodeIndex current_index = pq.top().second;
            pq.pop();
            if (visited[current_index])
            {
                continue;
            }
            visited[current_index] = true;

            if (current_index == dest_index)
                break;

            for (auto edge : VNodes[current_index].vedges)
            {
                double altdist = dist[current_index] + (method == 1 ? edge.time : (method == 0 ? edge.distance : edge.distance / edge.speed));
                if (altdist < dist[edge.nodeindex])
                {
                    dist[edge.nodeindex] = altdist;
                    prev[edge.nodeindex] = current_index;
                    pq.push(std::make_pair(altdist, edge.nodeindex));
                }
            }
        }

        if (dist[dest_index] == infinity)
        {
            std::cout << "bad" << std::endl;
            exit(0);
        }

        path.clear();
        TNodeIndex Dindex = dest_index;
        path.insert(path.begin(), VNodes[Dindex].NodeID);

        while (Dindex != src_index)
        {
            path.insert(path.begin(), VNodes[prev[Dindex]].NodeID);
            Dindex = prev[Dindex];
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

    void CMapRouter::print_vector_bool(std::vector<bool> v)
    {
        std::string ret;
        // std::cout << "sssize: " << v.size() << std::endl;
        for (int i = 0; i < v.size(); i++)
        {
            std::cout << v[i] << " ";
        }

        std::cout << std::endl;
    }
    void CMapRouter::print_vector_vector(std::vector<std::vector<std::string>> v)
    {
        std::string ret;
        for (auto el : v)
        {
            for (int i = 0; i < el.size(); i++)
            {
                std::cout << el[i] << " ";
            }

            std::cout << "//" << std::endl;
        }
        std::cout << std::endl;
    }