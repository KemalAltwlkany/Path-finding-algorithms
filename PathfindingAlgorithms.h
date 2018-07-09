#include "Node.h"
#include <vector>
#include <utility>
#include <list>

class PathfindingAlgorithms{
    protected:
        std::vector<std::vector<Node> > mapp; // matrix of Nodes
        int m,n; //map dimensions
        int start_x, start_y; //start coordinates
        int goal_x, goal_y;  //goal coordinates
        Node* goalptr; 
        Node* startptr;
        std::list<Node*> open_list; // list of nodes currently under consideration
        std::list<Node*> closed_list; //list of already expanded nodes
        std::list<Node*> neighbours; //list of neighbours of the currently expanding node
    public:
        //Constructor that enables map construction. Does not set obstacles or start/goal info.
        inline PathfindingAlgorithms(int p, int q): m(p), n(q), start_x(0), start_y(0), goal_x(0), goal_y(0) { std::vector<std::vector<Node> > temp(m, std::vector<Node> (n)); mapp=std::move(temp); }
        
        //Actually constructs map and expects start/goal and obstacle info inputs.
        void Generate_mapp();

         //Simple map printing function.
        void Print_mapp();

        //This function will compute the shortest path using the defined algorithm
        void Compute_shortest_path();

        void Update_neighbours( Node* &s );

        void Filter_obstacles( Node* &s );

        virtual void Update_vertex( Node* &s, Node* &sp )=0;

        void Erase_from_openlist( Node* &s );

        void Erase_from_closedlist( Node* &s );

        bool In_openlist( Node* &s ) const;

        bool In_closedlist( Node* &s ) const;

        void Extract_path();

};