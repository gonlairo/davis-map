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

CMapRouter::Node &CMapRouter::Node::operator=(const Node &node)
{
  this->NodeID  = node.NodeID;
  this->location = node.location;
  this->vedges = node.vedges; //this->vedges.operator=(node.vedges);
  return *this;
}

CMapRouter::Edge &CMapRouter::Edge::operator^=(const Edge &edge)
{
    this->oneway = edge.oneway;
    this->distance = edge.distance;
    this->busroute = edge.busroute;
    this->speed = edge.speed;
    this->time = edge.time;
    return *this;
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

bool CMapRouter::LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes){
    std::cout << __LINE__ << std::endl;
    CXMLReader OsmReader(osm);

    SXMLEntity OsmEnt;
    OsmReader.ReadEntity(OsmEnt, true);
    if ((OsmEnt.DType != SXMLEntity::EType::StartElement) or  (OsmEnt.DNameData != "osm"))
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
                std::cout << __LINE__ << std::endl;
                std::vector<Node> TempStorage;
                double speedlimit = 25;
                bool oneway = true;
                
                OsmReader.ReadEntity(OsmEnt, true); // skipping way tag
                OsmEnt.print_attributes();
                OsmReader.ReadEntity(OsmEnt, true); // grabbing start location
                OsmEnt.print_attributes();
                OsmReader.ReadEntity(OsmEnt, true); // grabbing start location
                OsmEnt.print_attributes();
                OsmReader.ReadEntity(OsmEnt, true); // grabbing start location
                OsmEnt.print_attributes();

                TNodeID id = std::stoul(OsmEnt.DAttributes[0].second); //std::stoul(OsmEnt.AttributeValue("ref"));
                int index = MNodeIds[id];
                Node CurrNode = VNodes[index]; // if statement if we dont find it => ignore it
                Edge connection;

                while (OsmEnt.DType != SXMLEntity::EType::EndElement)
                {
                    std::cout << __LINE__ << std::endl;
                    OsmReader.ReadEntity(OsmEnt, true);
                    if (OsmEnt.DNameData == "nd")
                    {
                        TNodeID id = std::stoul(OsmEnt.AttributeValue("ref"));
                        int index = MNodeIds[id];
                        Node DestNode = VNodes[index];
                        connection.nodeid = std::stoul(OsmEnt.AttributeValue("ref"));
                        connection.nodeindex = MNodeIds[connection.nodeid];
                        connection.distance = HaversineDistance(CurrNode.location.first, CurrNode.location.second,
                                                                DestNode.location.first, DestNode.location.second);
                        connection.speed = speedlimit;
                        connection.oneway = oneway;
                        CurrNode.vedges.push_back(connection);

                        // move to the next node
                        TempStorage.push_back(CurrNode);
                        CurrNode = DestNode;
                    }
                    else if (OsmEnt.DNameData == "tag")
                    {
                        if(OsmEnt.AttributeValue("k") == "maxspeed")
                        {
                            speedlimit = std::stod(OsmEnt.AttributeValue("v"));
                        }
                        if (OsmEnt.AttributeValue("k") == "oneway")
                        {
                            oneway = OsmEnt.AttributeValue("v") == "yes" ? true : false;
                        }
                    }
                }
                
                if (speedlimit != 25 || oneway == false)
                {
                    int count = 0;
                    for (int i = 0; i < TempStorage.size(); i++)
                    {
                        if (TempStorage[i].vedges.back().nodeid == TempStorage[i + 1].NodeID)
                        {
                            TempStorage[i].vedges.back().speed = speedlimit;
                            TempStorage[i].vedges.back().oneway = oneway;
                            
                            if (TempStorage.size() - 1 == i)
                            {
                                TempStorage[i].vedges.back().speed = speedlimit;
                                TempStorage[i].vedges.back().oneway = oneway;
                                break;
                            }
                            
                        }
                        else
                        {
                            /* what do we do if we have a cycle? */
                        }
                        
                    }
                }

                if (oneway == false)
                {
                    for (int i = TempStorage.size() - 1; i < 0; i--)
                    {
                        auto CurrNodeBW = TempStorage[i];
                        auto Destination = TempStorage[i - 1];
                        Edge TempEdge;
                        TempEdge ^= TempStorage[i].vedges.back();
                        TempEdge.nodeid = Destination.NodeID;
                        TempEdge.nodeindex = MNodeIds[Destination.NodeID];
                        CurrNode.vedges.push_back(TempEdge);                        
                    }
                    
                }     

            }

        }
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
