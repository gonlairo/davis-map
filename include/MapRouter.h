#ifndef MAPROUTER_H
#define MAPROUTER_H

#include <vector>
#include <istream>
#include <map>
#include <tuple>


class CMapRouter{
    public:
        using TNodeID = unsigned long;
        using TStopID = unsigned long;
        using TLocation = std::pair<double, double>; // longitude and latitude
        using TPathStep = std::pair<std::string, TNodeID>;
        using TNodeIndex = int;
        static const TNodeID InvalidNodeID;
    
    private:

        class BusEdgeInfo
        {
        public:
            double distance;
            double time;
            std::vector<TNodeID> path;
            std::vector <std::string> RouteNames;
        };

        class Edge
        {
        public:
            TNodeID nodeid;
            TNodeIndex nodeindex;
            double distance; 
            double time;
            double speed;
            bool oneway;
            bool busedge;  
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
        typedef std::tuple <TNodeID, TNodeID> tuple;
        std::map<tuple, BusEdgeInfo> MStopSteps;

    public:
        CMapRouter();
        ~CMapRouter();
        
        static double HaversineDistance(double lat1, double lon1, double lat2, double lon2);
        static double CalculateBearing(double lat1, double lon1,double lat2, double lon2);
        
        bool LoadMapAndRoutes(std::istream &osm, std::istream &stops, std::istream &routes);
        size_t NodeCount() const;
        TNodeID GetSortedNodeIDByIndex(size_t index) const;
        TLocation GetSortedNodeLocationByIndex(size_t index) const;
        TLocation GetNodeLocationByID(TNodeID nodeid) const;
        TNodeID GetNodeIDByStopID(TStopID stopid) const;
        size_t RouteCount() const;
        std::string GetSortedRouteNameByIndex(size_t index) const;
        bool GetRouteStopsByRouteName(const std::string &route, std::vector< TStopID > &stops);
        
        double FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path);
        double FindFastestPath(TNodeID src, TNodeID dest, std::vector< TPathStep > &path);
        double dijkstras(TNodeID src, TNodeID dest, std::vector<TNodeID> &path, int method);
        bool GetPathDescription(const std::vector< TPathStep > &path, std::vector< std::string > &desc) const;
        std::vector<TNodeID> unique_vector(std::vector<TNodeID> &v);

        //delete
        void print_vector(std::vector<TStopID> v);
        void print_vector_double(std::vector<double> v);
        void print_vector_string(std::vector<std::string> v);
        void print_vector_bool(std::vector<bool> v);
        void print_vector_vector(std::vector<std::vector<std::string>> v);
};

#endif
