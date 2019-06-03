#ifndef MAPROUTER_H
#define MAPROUTER_H

#include <vector>
#include <istream>
#include <map>

// ways: if we cant find a node in a way, just ignore it.
// important tags: speed / oneway

// nodecount length of the vector
class CMapRouter{
    public:
        using TNodeID = unsigned long;
        using TStopID = unsigned long;
        using TLocation = std::pair<double, double>; // longitude and latitude
        using TPathStep = std::pair<std::string, TNodeID>;
        using TNodeIndex = int;
        static const TNodeID InvalidNodeID;
    
    private:

        
        class Edge
        {
            public:
            TNodeID nodeid;
            TNodeIndex nodeindex;
            double distance; 
            double walking_time;
            double bus_time;
            double speed;
            bool oneway;
            bool busroute;
        };

        class Node
        {
            public:
            TNodeID NodeID;
            TLocation location;
            std::vector<Edge> vedges;  
            TStopID stop;

            // operator overloading
            Node &operator=(const Node &node);
        };

        std::vector<Node> VNodes;
        std::map<TNodeID, TNodeIndex> MNodeIds; //key=nodeid, value=index
        std::vector<TNodeID> VSortedIds;

        // bus information
        std::map <TStopID, TNodeID> MTStopNodeIds;
        std::map <TNodeID, TStopID> MTNodeStopIds;
        std::map <std::string, std::vector <TStopID>> MBusRoutes;
        std::map < std::make_tuple(TNodeID, TNodeID, double) , std::vector<TNodeID> > MStopSteps; 

    public:
        CMapRouter();
        ~CMapRouter();
        
        static double HaversineDistance(double lat1, double lon1, double lat2, double lon2);
        static double CalculateBearing(double lat1, double lon1,double lat2, double lon2);
        
        bool LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes);
        size_t NodeCount() const;
        // MapID.size();
        TNodeID GetSortedNodeIDByIndex(size_t index) const;
        TLocation GetSortedNodeLocationByIndex(size_t index) const;
        TLocation GetNodeLocationByID(TNodeID nodeid) const;
        TNodeID GetNodeIDByStopID(TStopID stopid) const;
        size_t RouteCount() const;
        std::string GetSortedRouteNameByIndex(size_t index) const;
        bool GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops);
        
        double FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path);
        double FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path);
        bool GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const;


};

#endif
