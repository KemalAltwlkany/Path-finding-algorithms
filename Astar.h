#include "PathfindingAlgorithms.h"

class Astar : public PathfindingAlgorithms{
    public:
        //Constructor that enables map construction. Does not set obstacles or start/goal info.
        inline Astar(int p, int q): PathfindingAlgorithms(p,q) {}

        void Update_vertex( Node* &s, Node* &sp ) override;
};