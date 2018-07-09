#include "Theta.h"

class DstarLite : public PathfindingAlgorithms{
    protected:
        double km;
        std::list<Node*> openlist_DS;
        std::list<Node*> closedlist_DS;
        std::list<Node*> Succ;
        std::list<Node*> Pred;
        Node* lastptr;
    public:
        inline DstarLite( int p, int q ): PathfindingAlgorithms(p,q), km(0) {}

        void Update_vertex( Node* &s, Node* &sp ) override; //not used, had to be overriden for A* and Theta*

};