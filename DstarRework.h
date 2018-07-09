#include "Theta.h"

class DstarLite : public PathfindingAlgorithms{
    double km;
    std::list<Node*> Pred;
    std::list<Node*> Succ;
    Node* lastptr;
    public:
        inline DstarLite( int p, int q ): PathfindingAlgorithms(p,q), km(0) {}

        void Generate_mapp_DS();

        void Update_vertex( Node* &s, Node* &sp ) override; //not used, had to be overriden for A* and Theta*

        void Compute_shortest_path_DS();

        void Compute_forever();

        DSKey Calculate_DSKey( Node* &s );

        void Get_Pred( Node* &s );

        void Get_Succ( Node* &s );

        void Update_vertex_DS( Node* &u );
        
};