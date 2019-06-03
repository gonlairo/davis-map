#include "MapRouter.h"
#include <cmath>
#include <iostream>
#include "CSVReader.h"
#include "CSVWriter.h"
#include "XMLReader.h"
#include "XMLEntity.h"

double dijkstras(TNodeID src, TNodeID dest, std::vector< TNodeID > &path, bool findfastest);

const CMapRouter::TNodeID CMapRouter::InvalidNodeID = -1;

CMapRouter::CMapRouter(){
}

CMapRouter::~CMapRouter(){
}

// CMapRouter::Node &CMapRouter::Node::operator=(const Node &node)
// {
//   this->NodeID  = node.NodeID;
//   this->location = node.location;
//   this->vedges = node.vedges; //this->vedges.operator=(node.vedges);
//   return *this;
// }

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
                        
                        // IGNORE NODE IF WE DIDNT READ IT BEFORE
                        // while(!MNodeIds.count(id)) 
                        // {
                        //     OsmReader.ReadEntity(OsmEnt, true);
                        // }

                        // TNodeID id = std::stoul(OsmEnt.AttributeValue("ref")); // do later

                        int index = MNodeIds[id];
                        Node CurrNode = VNodes[index];
                        TempStorage.push_back(CurrNode);
                        OsmReader.ReadEntity(OsmEnt, true);
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

                std::cout << TempStorage.size() << std::endl;
                
                // CHECK OUT OF BOUNDS (- 1 WORKS?)
                for (int i = 0; i < TempStorage.size() - 1; i++)
                {
                    int walking_speed = 3;
                    Edge connection;

                    connection.distance = HaversineDistance(TempStorage[i].location.first, TempStorage[i].location.second,
                                                            TempStorage[i + 1].location.first, TempStorage[i + 1].location.second);

                    //std::cout << "distance between " << TempStorage[i].NodeID << " and " << TempStorage[i + 1].NodeID << " is: " << connection.distance << std::endl;
                    connection.walking_time = connection.distance/walking_speed;
                    connection.bus_time = connection.distance/speedlimit;
                    connection.speed = speedlimit;
                    connection.oneway = oneway;

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
        TStopID stopid = std::stoul(TempStops[1]);
        MBusRoutes[BusName].push_back(stopid);
    }

    for(auto kv : MBusRoutes){
        Vstops = kv.second
        std::vector< TNodeID > temppath;
        for(int i = 0; i < Vstops.size()-1; i++){

            TNodeID start = MTStopNodeIds[Vstops[i]];
            TNodeID dest = MTStopNodeIds[Vstops[i+1]];
            double fastest = dijkstras(start, dest, temppath, true);
            MStopSteps[std::make_tuple(start, dest, fastest)] = temppath;
        }
    }

    for(auto kv : MStopSteps){
        auto first = kv.first;
        std::vector< TNodeID > second = kv.second;
        TNodeID startid = std::get<0>(first);
        double temptime = std::get<2>(first); 
        Edge busedge;
        busedge.bus_time = temptime + 0.30;
        busedge.nodeid = std::get<1>(first);
        busedge.nodeindex = MNodeIds[std::get<1>(first)];
        busedge.busroute = true;
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
    // Your code HERE
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    TNodeIndex src_index = MNodeIds[src];
    TNodeIndex dest_index = MNodeIds[dest];

    typedef std::pair<double, TNodeIndex> npair; // pair of distance (weight) / TNodeIndex (Node)
    std::priority_queue<npair, std::vector<npair>, std::greater<npair>> pq;


    std::vector<double> dist(VNodes.size(), INFINITY);
    std::vector<double> prev(VNodes.size());

    pq.push(std::make_pair(0, src_index));
    dist[src_index] = 0;
    // prev[src_index] = -1 or something

    while (!pq.empty())
    {
        Node current = VNodes[pq.top().second];
        TNodeIndex current_index = MNodeIds[current.NodeID];
        pq.pop();

        for (auto edge : current.vedges)
        {
            double altdist = dist[current_index] + edge.distance;
            if (altdist <= dist[edge.nodeindex])
            {
                if (dist[edge.nodeindex] == INFINITY)
                {
                    pq.push(std::make_pair(altdist, edge.nodeindex));
                }
                dist[edge.nodeindex] = altdist;
                prev[edge.nodeindex] = current_index;
            }
        }

        // std::cout << "\n";

        // for (int i = 0; i < dist.size(); i++)
        // {
        //     std::cout << dist[i] << " ";
        // }

        // std::cout << "\n";

        // for (int i = 0; i < prev.size(); i++)
        // {
        //     std::cout << prev[i] << " ";
        // }

        // std::cout << "\n";
    }


    path.clear();
    TNodeIndex Dindex = dest_index; 

    path.insert(path.begin(), VNodes[Dindex].NodeID);
    while (Dindex != src_index)
    {
        path.insert(path.begin(), VNodes[prev[Dindex]].NodeID);
        Dindex = prev[Dindex];
    }


    std::cout << "\n";
    for (int i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << " ";
    }
    std::cout << "\n";

    // for (int i = 0; i < dist.size(); i++)
    // {
    //     std::cout << dist[i] << " ";
        
    // }

    // std::cout << "\n";

    // for (int i = 0; i < prev.size(); i++)
    // {
    //     std::cout << prev[i] << " ";
    // }

    // std::cout << "\n";

    return dist[dest_index];
}

double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path){
    
    
}

bool CMapRouter::GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const{
    // Your code HERE
}

double dijkstras(TNodeID src, TNodeID dest, std::vector< TNodeID > &path, bool findfastest){
    TNodeIndex src_index = MNodeIds[src];
    TNodeIndex dest_index = MNodeIds[dest];

    typedef std::pair<double, TNodeIndex> npair; // pair of distance (weight) / TNodeIndex (Node)
    std::priority_queue<npair, std::vector<npair>, std::greater<npair>> pq;


    std::vector<double> dist(VNodes.size(), INFINITY);
    std::vector<double> prev(VNodes.size());

    pq.push(std::make_pair(0, src_index));
    dist[src_index] = 0;
    // prev[src_index] = -1 or something

    while (!pq.empty())
    {
        Node current = VNodes[pq.top().second];
        TNodeIndex current_index = MNodeIds[current.NodeID];
        pq.pop();

        for (auto edge : current.vedges)
        {
            double altdist = dist[current_index] + (findfastest ? edge.bus_time : edge.distance)
            if (altdist <= dist[edge.nodeindex])
            {
                if (dist[edge.nodeindex] == INFINITY)
                {
                    pq.push(std::make_pair(altdist, edge.nodeindex));
                }
                dist[edge.nodeindex] = altdist;
                prev[edge.nodeindex] = current_index;
            }
        }
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
