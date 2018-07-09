#include <limits>

const double inf = std::numeric_limits<double>::infinity();

class DSKey{
    public:
        double k1;
        double k2;
        inline DSKey(): k1(0), k2(0) {}
        inline DSKey(double p, double q): k1(p), k2(q) {} 
};

class Node{
    friend class PathfindingAlgorithms;
    friend class Astar;
    friend class Theta;
    friend class DstarLite;
    protected:
        bool not_traversable, traversable, g_inf, f_inf, in_open, in_closed, rhs_inf;
        double gvalue, fvalue, rhs;
        Node* parent;
        int x,y,traverse_cost;
        DSKey kold; // used only by D*-Lite
        DSKey knew; //used only by D*-Lite
    public:
        inline Node(): not_traversable(false), traversable(true), g_inf(true), f_inf(true), gvalue(inf), fvalue(inf), parent(nullptr), traverse_cost(0), x(0), y(0), in_open(false), in_closed(false), kold(), knew(), rhs(inf), rhs_inf(true) {}

        inline Node(bool pom): not_traversable(!pom), traversable(pom), g_inf(true), f_inf(true), gvalue(inf), fvalue(inf), parent(nullptr), traverse_cost(0), x(0), y(0), in_open(false), in_closed(false), kold(), knew(), rhs(inf), rhs_inf(true) {}

        void Print_Node_info() const;

        void Calculate_fvalue(const int& xg, const int& xy);

        friend bool operator==(const Node& s1, const Node& s2);

        friend bool operator!=(const Node& s1, const Node& s2);
        
        friend bool Compare( Node* s1, Node* s2);

        friend bool Compare_DS( Node* s1, Node* s2);
        
        friend double Compute_cost( Node* &u, Node* &s ); 

};