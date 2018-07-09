#include "Astar.h"

class Theta : public PathfindingAlgorithms{
    public:
        //Constructor that enables map construction. Does not set obstacles or start/goal info.
        inline Theta(int p, int q): PathfindingAlgorithms(p,q) {}

        void Update_vertex( Node* &s, Node* &sp ) override;

        bool Line_of_sight( Node* &p0, Node* &p1);

};