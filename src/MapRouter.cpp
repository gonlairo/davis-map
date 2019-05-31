#include "MapRouter.h"
#include <cmath>
#include <iostream>
#include "CSVReader.h"
#include "CSVWriter.h"
#include "XMLReader.h"
#include "XMLEntity.h"



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

                for (int i = 0; i < TempStorage.size(); i++)
                {
                    int walking_speed = 3;
                    Edge connection;

                    connection.distance = HaversineDistance(TempStorage[i].location.first, TempStorage[i].location.second,
                                                            TempStorage[i + 1].location.first, TempStorage[i + 1].location.second);
                    connection.time = connection.distance/walking_speed;
                    connection.speed = speedlimit;
                    connection.oneway = oneway;

                    TNodeID current = TempStorage[i].NodeID;
                    TNodeIndex current_index = MNodeIds[current];
                    TNodeID next = TempStorage[i + 1].NodeID;
                    TNodeIndex next_index = MNodeIds[next];

                    connection.nodeid = next;
                    connection.nodeindex = next_index;
                    TempStorage[i].vedges.push_back(connection);

                    if (oneway == false)
                    {
                        connection.nodeid = current;
                        connection.nodeindex = current_index;
                        TempStorage[i + 1].vedges.push_back(connection);
                    }
                }
            }
        }
    }

    CCSVReader StopsReader(stops);
    CCSVReader RoutesReader(routes);

    std::vector <std::string> TempStops;
    std::vector <std::string> TempRoutes;
    
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
    // Your code HERE
}

CMapRouter::TNodeID CMapRouter::GetNodeIDByStopID(TStopID stopid) const{
    // Your code HERE
}

size_t CMapRouter::RouteCount() const{
    // Your code HERE
}

std::string CMapRouter::GetSortedRouteNameByIndex(size_t index) const{
    // Your code HERE
}

bool CMapRouter::GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops){
    // Your code HERE
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    // Your code HERE
}

double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path){
    // Your code HERE
}

bool CMapRouter::GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const{
    // Your code HERE
}
