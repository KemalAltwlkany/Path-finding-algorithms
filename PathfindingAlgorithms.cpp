#include "DstarLite.h"
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <iterator>

/* Heuristika koja se koristi je udaljenost (sqrt) */

const double eps = 0.000001;

double Compute_cost( Node* &u, Node* &s ){
    if ( u->not_traversable || s->not_traversable ){
        return inf;
    }
    return std::sqrt( (u->x-s->x)*(u->x-s->x) + (u->y-s->y)*(u->y-s->y) );
}

bool operator < ( const DSKey& key1, const DSKey& key2){ // compares two instances of class DSKey
    if ( key1.k1 - key2.k1 < 0 ) return true;
    //else if ( (std::abs(key1.k1 - key2.k2) < eps ) && ( (key1.k2 - key2.k2 < 0) ||  ( std::abs(key1.k2 - key2.k2) < eps )  )) return true;
    else if ( (std::abs(key1.k1 - key2.k2) < eps ) &&  (key1.k2 - key2.k2 < 0) ) return true;
    return false;
}

bool Compare(Node* s1, Node* s2){
    if( s1->g_inf && !s2->g_inf ) return false;
    if( s2->g_inf && !s1->g_inf ) return true;
    if( s2->g_inf && s1->g_inf ) return false;
    return ( ( s1->fvalue - s2->fvalue ) < 0 );
}

//COMPARES OLD KEYS!!! NEW ONES ARE HELD IN MAPP WHICH IS A MATRIX OF NODES
bool Compare_DS( Node* s1, Node* s2){ //compares two nodes used for sorting in the priority list by their D*-keys. COMPARES OLD KEYS!!!
    return ( s1->kold < s2->kold );
}

bool Theta::Line_of_sight( Node* &p0, Node* &p1 ){
    int dx( p1->x - p0->x ), dy( p1->y - p0->y );
    int sign_x, sign_y;
    if ( dx < 0) sign_x=-1;
    else sign_x=1;
    if( dy < 0 ) sign_y=-1;
    else sign_y=1;
    dx=std::abs(dx);
    dy=std::abs(dy);
    int px( p0->x ), py( p0->y );
    int ix(0), iy(0);
    while(1){
        if( ( dy*(1+2*ix) ) == ( dx*(1+2*iy) ) ){
            //the next node is a diagonal neighbour of the current node
            px+=sign_x;
            py+=sign_y;
            ix++;
            iy++;
            if ( mapp[px][py].not_traversable ) return false;
        }
        else if( ( dy*(1+2*ix) ) <  ( dx*(1+2*iy) ) ){
            //the next node is a vertical neighbour of the current node
            px+=sign_x;
            ix++;
            if ( mapp[px][py].not_traversable ) return false;
        }
        else if( ( dy*(1+2*ix) ) >  ( dx*(1+2*iy) ) ){
            //the next node is a horizontal neighbour of the current node
            py+=sign_y;
            iy++;
            if ( mapp[px][py].not_traversable ) return false;
        }
        if( !(ix<dx || iy<dy) ){ break; }
    }
    return true;
}

void Theta::Update_vertex( Node* &s, Node* &sp){
    if ( s->parent != nullptr ){
        if ( Line_of_sight( s->parent, sp ) ){
            //path 2, considered by Theta only IFF there is line of sight between the parent of current node and observed node
            Node* par;
            par=s->parent;
            if ( par->g_inf ){ return; }

            if ( sp->g_inf ){
                sp->parent=par;
                sp->g_inf=false;        
                sp->gvalue = par->gvalue + std::sqrt( (sp->x - par->x )*(sp->x - par->x ) +  ( sp->y - par->y )*( sp->y - par->y ) );
                sp->Calculate_fvalue(goal_x, goal_y);
                if ( sp->in_open ){
                    (*this).Erase_from_openlist(sp);
                }
                open_list.insert( std::upper_bound(open_list.begin(), open_list.end(), sp, Compare), sp );
                //open_list.insert( open_list.begin(), sp);
                sp->in_open=true;
                return;
            }

            if (   ( ( par->gvalue + std::sqrt( (sp->x - par->x )*(sp->x - par->x ) +  ( sp->y - par->y )*( sp->y - par->y) )  ) - sp->gvalue ) < 0  ){
                sp->parent=par;
                sp->g_inf=false;        
                sp->gvalue = par->gvalue + std::sqrt( (sp->x - par->x )*(sp->x - par->x ) +  ( sp->y - par->y )*( sp->y - par->y ) );
                sp->Calculate_fvalue(goal_x, goal_y);
                if ( sp->in_open ){
                    (*this).Erase_from_openlist(sp);
                }
                open_list.insert( std::upper_bound(open_list.begin(), open_list.end(), sp, Compare), sp );
                //open_list.insert( open_list.begin(), sp);               
                sp->in_open=true;       
                return;
            }
        }
    }
    //path that Astar considers
    if ( s->g_inf ){ return; }
    if ( sp->g_inf ){
        sp->parent=s;
        sp->g_inf=false;        
        sp->gvalue = s->gvalue + std::sqrt( (sp->x - s->x )*(sp->x - s->x ) +  ( sp->y - s->y )*( sp->y - s->y ) );
        sp->Calculate_fvalue(goal_x, goal_y);
        if ( sp->in_open ){
            (*this).Erase_from_openlist(sp);
        }
        open_list.insert( std::upper_bound(open_list.begin(), open_list.end(), sp, Compare), sp );
        //open_list.insert( open_list.begin(), sp);
        sp->in_open=true;
        return;
    }
    if (   ( ( s->gvalue + std::sqrt( (sp->x - s->x )*(sp->x - s->x ) +  ( sp->y - s->y )*( sp->y - s->y) )  ) - sp->gvalue ) < 0  ){
        sp->parent=s;
        sp->g_inf=false;        
        sp->gvalue = s->gvalue + std::sqrt( (sp->x - s->x )*(sp->x - s->x ) +  ( sp->y - s->y )*( sp->y - s->y ) );
        sp->Calculate_fvalue(goal_x, goal_y);
        if ( sp->in_open ){
            (*this).Erase_from_openlist(sp);
        }
        open_list.insert( std::upper_bound(open_list.begin(), open_list.end(), sp, Compare), sp );
        //open_list.insert( open_list.begin(), sp);               
        sp->in_open=true;       
        return;
    }
}

void PathfindingAlgorithms::Extract_path(){
    Node* s;
    s=goalptr;
    while(1){
        s->traverse_cost=4;
        if ( s->parent == s ){ break; }
        s=s->parent;
    }
}

bool PathfindingAlgorithms::In_openlist( Node* &s ) const{
    for( auto it=open_list.rbegin(); it!=open_list.rend(); it++ ){
        if ( (*s) == (*(*it)) ){
            return true;
        }
    }
    return false;
}

bool PathfindingAlgorithms::In_closedlist( Node* &s ) const {
    for( auto it=closed_list.rbegin(); it!=closed_list.rend(); it++ ){
        if ( (*s) == (*(*it)) ){
            return true;
        }
    }
    return false;
}

void PathfindingAlgorithms::Erase_from_closedlist( Node* &s ){
    for( std::list<Node*>::iterator it=closed_list.begin(); it!=closed_list.end(); it++ ){
        if ( (*s) == (*(*it)) ){
            closed_list.erase( it );
            break;
        }
    }
} 

void PathfindingAlgorithms::Erase_from_openlist( Node* &s ){
    for( std::list<Node*>::iterator it=open_list.begin(); it!=open_list.end(); it++ ){
        if ( (*s) == (*(*it)) ){
            open_list.erase( it );
            break;
        }
    }
}

void Astar::Update_vertex( Node* &s, Node* &sp){
    if ( s->g_inf ){ return; }
    if ( sp->g_inf ){
        sp->parent=s;
        sp->g_inf=false;        
        sp->gvalue = s->gvalue + std::sqrt( (sp->x - s->x )*(sp->x - s->x ) +  ( sp->y - s->y )*( sp->y - s->y ) );
        sp->Calculate_fvalue(goal_x, goal_y);
        if ( sp->in_open ){
            (*this).Erase_from_openlist(sp);
        }
        open_list.insert( std::upper_bound(open_list.begin(), open_list.end(), sp, Compare), sp );
        //open_list.insert( open_list.begin(), sp);
        sp->in_open=true;
        return;
    }
    if (   ( ( s->gvalue + std::sqrt( (sp->x - s->x )*(sp->x - s->x ) +  ( sp->y - s->y )*( sp->y - s->y) )  ) - sp->gvalue ) < 0  ){
        sp->parent=s;
        sp->g_inf=false;        
        sp->gvalue = s->gvalue + std::sqrt( (sp->x - s->x )*(sp->x - s->x ) +  ( sp->y - s->y )*( sp->y - s->y ) );
        sp->Calculate_fvalue(goal_x, goal_y);
        if ( sp->in_open ){
            (*this).Erase_from_openlist(sp);
        }
        open_list.insert( std::upper_bound(open_list.begin(), open_list.end(), sp, Compare), sp );
        //open_list.insert( open_list.begin(), sp);               
        sp->in_open=true;       
        return;
    }
}

void PathfindingAlgorithms::Filter_obstacles( Node* &s ){
    std::list<Node*> pom;
    if ( neighbours.size() == 8 ){ //we disallow diagonal movements through 2 corner-tangent obstacles.
        //std::cout << "entered if ==8";
        bool upright(false), upleft(false), downright(false), downleft(false);   
            if( mapp[s->x-1][s->y].not_traversable && mapp[s->x][s->y-1].not_traversable  ){ //remove upper-left corner
                upleft=true;
                //std::cout << "upleft is true";
            }
            if( mapp[s->x-1][s->y].not_traversable && mapp[s->x][s->y+1].not_traversable ){ //remove upper-right corner
                upright=true;
                //std::cout << "upright is true";                
            }
            if( mapp[s->x][s->y-1].not_traversable && mapp[s->x+1][s->y].not_traversable ){ // remove down-left corner
                //std::cout << "downleft is true";
                downleft=true;
            }
            if( mapp[s->x][s->y+1].not_traversable && mapp[s->x+1][s->y].not_traversable ){ //remove down-right corner
                //std::cout << "downright is true";
                downright=true;
            }
        for( std::list<Node*>::iterator it=neighbours.begin(); it!=neighbours.end(); it++ ){
            if( upleft && mapp[s->x-1][s->y-1]==(*(*it)) ){
                continue;
            }
            if( upright && mapp[s->x-1][s->y+1]==(*(*it)) ){
                continue;
            }
            if( downleft && mapp[s->x+1][s->y-1]==(*(*it)) ){
                continue;
            }
            if( downright && mapp[s->x+1][s->y+1]==(*(*it)) ){
                continue;
            }
            if (  (*it)->traversable ){ 
                pom.insert( pom.begin(), (*it) );
            }
        }
        neighbours=pom;
        return;
    }            
    for( std::list<Node*>::iterator it=neighbours.begin(); it!=neighbours.end(); it++ ){
        if (  (*it)->traversable ){ 
            pom.insert( pom.begin(), (*it) );
            //(*it)->Print_Node_info();
        }
    }
    neighbours=pom;
}

void PathfindingAlgorithms::Update_neighbours( Node* &s ){
        // General case
        if ( !( s->x==0 || s->y==0 || s->x==m-1 || s->y==n-1)  ){
            //std::cout << "\ngeneral case happened";
            neighbours.insert( neighbours.end(), &mapp[s->x-1][s->y-1] );
            neighbours.insert( neighbours.end(), &mapp[s->x-1][s->y]   );
            neighbours.insert( neighbours.end(), &mapp[s->x-1][s->y+1] );
            neighbours.insert( neighbours.end(), &mapp[s->x][s->y-1]   );
            neighbours.insert( neighbours.end(), &mapp[s->x][s->y+1]   );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][s->y-1] );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][s->y]   );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][s->y+1] );
            return;        
        }
        //Node located in corner (0,0)
        if( s->x==0 && s->y==0 ){
            //std::cout << "\n 0,0 happened";
            neighbours.insert( neighbours.end(), &mapp[0][1] );
            neighbours.insert( neighbours.end(), &mapp[1][1] );
            neighbours.insert( neighbours.end(), &mapp[1][0] );
            return;
        }
        //Node located in corner (0,n-1)
        if( s->x==0 && s->y==n-1 ){
            neighbours.insert( neighbours.end(), &mapp[0][n-2]  );
            neighbours.insert( neighbours.end(), &mapp[1][n-2]  );
            neighbours.insert( neighbours.end(), &mapp[1][n-1]  );
            return;
        }
        //Node located in corner (m-1, 0)
        if( s->x==m-1 && s->y==0 ){
            neighbours.insert( neighbours.end(), &mapp[m-2][0]  );
            neighbours.insert( neighbours.end(), &mapp[m-2][1]  );
            neighbours.insert( neighbours.end(), &mapp[m-1][1]  );
            return;
        }
        //Node located in corner (m-1, n-1)
        if ( s->x==m-1 && s->y==n-1 ){
            neighbours.insert( neighbours.end(), &mapp[m-2][n-1] );
            neighbours.insert( neighbours.end(), &mapp[m-2][n-2] );
            neighbours.insert( neighbours.end(), &mapp[m-1][n-2] );
            return;
        }
        //Node located in first row
        if (s->x==0){
            neighbours.insert( neighbours.end(), &mapp[0][s->y-1]   );
            neighbours.insert( neighbours.end(), &mapp[0][s->y+1]   );
            neighbours.insert( neighbours.end(), &mapp[1][s->y-1] );
            neighbours.insert( neighbours.end(), &mapp[1][s->y]   );
            neighbours.insert( neighbours.end(), &mapp[1][s->y+1] );
            return;
        }
        //Node located in first column
        if ( s->y==0 ){
            neighbours.insert( neighbours.end(), &mapp[s->x-1][0]   );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][0]   );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][1] );
            neighbours.insert( neighbours.end(), &mapp[s->x][1]   );
            neighbours.insert( neighbours.end(), &mapp[s->x-1][1] );
            return;
        }
        //Node located in last row
        if ( s->x==m-1 ){
            neighbours.insert( neighbours.end(), &mapp[m-2][s->y-1] );
            neighbours.insert( neighbours.end(), &mapp[m-2][s->y]   );
            neighbours.insert( neighbours.end(), &mapp[m-2][s->y+1] );
            neighbours.insert( neighbours.end(), &mapp[m-1][s->y-1]  );
            neighbours.insert( neighbours.end(), &mapp[m-1][s->y+1]  );
            return;
        }
        //Node located in last column
        if ( s->y==n-1 ){
            neighbours.insert( neighbours.end(), &mapp[s->x-1][n-2] );
            neighbours.insert( neighbours.end(), &mapp[s->x][n-2] );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][n-2] );
            neighbours.insert( neighbours.end(), &mapp[s->x-1][n-1] );
            neighbours.insert( neighbours.end(), &mapp[s->x+1][n-1] );
            return;
        }        
        return;
}

void PathfindingAlgorithms::Compute_shortest_path(){
    open_list.clear();
    closed_list.clear();   
    //initialize start node
            (*startptr).gvalue=0;
            (*startptr).traversable=true;
            (*startptr).not_traversable=false;
            (*startptr).g_inf=false;
            (*startptr).x=start_x;
            (*startptr).y=start_y;
            (*startptr).parent= &mapp[start_x][start_y];
            (*startptr).Calculate_fvalue(goal_x,goal_y);
    open_list.insert(open_list.begin(),startptr);
    startptr->in_open=true;
    startptr->in_closed=false;    
    Node* s;
    while ( !open_list.empty() ){
        s=( *open_list.begin() );
        //s->Print_Node_info();
        open_list.erase( open_list.begin() );
        s->in_open=false;
        if ( (*s) == (*goalptr) ){
            std::cout << "\n        Path found!" << std::endl;
            (*this).Extract_path();
            (*this).Print_mapp();
            return;
        }
        //closed_list.insert( std::upper_bound(closed_list.begin(), closed_list.end(), s, Compare), s );
        closed_list.insert( closed_list.end(), s);
        s->in_closed=true;
        neighbours.clear();
        (*this).Update_neighbours(s);
        (*this).Filter_obstacles(s);
        for( std::list<Node*>::iterator it=neighbours.begin(); it!=neighbours.end(); it++){
           if( !( (*it)->in_closed ) ){
                //if( !( (*it)->in_open ) ){
                   // (*it)->g_inf=true;
                   // (*it)->f_inf=true;
                   // (*it)->parent=nullptr;
                //}
            (*this).Update_vertex(s, (*it));
           }
        }
        //open_list.sort(Compare);
    }
    std::cout << "\n        No path found!" << std::endl;
    (*this).Print_mapp();
    
}

void PathfindingAlgorithms::Print_mapp(){
    for(int i=0; i<m; i++){
            std::cout << "          ";
        for(int j=0; j<n; j++){
            std::cout << mapp[i][j].traverse_cost << "  ";
        }
        std::cout << std::endl;
    }
    return;
}

void PathfindingAlgorithms::Generate_mapp(){
    int c;
    for(int i=0; i<m; i++){
        for(int j=0; j<n; j++){
            mapp[i][j].x=i;
            mapp[i][j].y=j;
        }
    }
    std::cout << "\n     How would you like to generate obstacles: ";
    std::cout << "\n     (1) no obstacles ";
    std::cout << "\n     (2) a column-wise snake ";
    std::cout << "\n     (3) a row-wise snake ";
    std::cout << "\n     (4) random obstacles ";
    std::cout << "\n     (any other integer) manual insertion of obstacles ";
    std::cin >> c;
    switch(c){
        case 1:
            break;
        case 2:
            {
            bool pom(false);
            for(int j=0; j<n; j++){
                for(int i=0; i<m; i++){
                    if(j%2==1){
                        mapp[i][j].traversable=false;
                        mapp[i][j].not_traversable=true;
                        mapp[i][j].traverse_cost=1;
                    }
                }
                if( pom && j%2==1 ){
                    mapp[0][j].traversable=true;
                    mapp[0][j].not_traversable=false;
                    mapp[0][j].traverse_cost=0;
                    pom=false;
                }
                else if( !pom && j%2==1 ){
                    pom=true;
                    mapp[m-1][j].traversable=true;
                    mapp[m-1][j].not_traversable=false;
                    mapp[m-1][j].traverse_cost=0;
                }
            }
            }
            break;
        case 3:
            {
            bool pom2(false);
            for(int i=0; i<m; i++){
                for(int j=0; j<n; j++){
                    if(i%2==1){
                        mapp[i][j].traversable=false;
                        mapp[i][j].not_traversable=true;
                        mapp[i][j].traverse_cost=1;
                        
                    }
                }
                if( pom2 && i%2==1){
                    mapp[i][0].traversable=true;
                    mapp[i][0].not_traversable=false;
                    mapp[i][0].traverse_cost=0;
                    pom2=false;
                }
                else if( !pom2 && i%2==1 ){
                    mapp[i][n-1].traversable=true;
                    mapp[i][n-1].not_traversable=false;
                    mapp[i][n-1].traverse_cost=0;                    
                    pom2=true;
                }
            }
            break;
            }
        case 4:
            {
            int br(0),brmax(0);
            std::cout << "\n        How many obstacles do you want? ";
            std::cin >> brmax;
            if( brmax<0 || brmax>(m*n-2) ) std::cout << "\n      ERROR! Number of obstacles should be a positive integer and less than the map size!";
            int rand_x(0), rand_y(0);
            while( br < brmax ){
                rand_x= std::rand()%m;
                rand_y= std::rand()%n;
                if ( mapp[rand_x][rand_y].not_traversable ) br--;
                mapp[rand_x][rand_y].traversable=false;
                mapp[rand_x][rand_y].not_traversable=true;
                mapp[rand_x][rand_y].traverse_cost=1;
                
                br++;
            }
            break;
            }
        default:{
            std::cout << "\n        Input the x coordinate and afterwards the y coordinate of the node where you wish to place an obstacle. Enter a negative x coordinate to terminate";
            std::cout << std::endl;
            int p,q;
            while(1){
                std::cout << "      x coordinate:\n";
                std::cin >> p;
                std::cout << "      y coordinate:\n";
                std::cin >> q;
                if( p<0 || q<0 ) break;
                if( p>m-1 || q>n-1 ){
                        std::cout << "\n       ERROR! Index is not valid!";
                        break;
                }
                mapp[p][q].traversable=false;
                mapp[p][q].not_traversable=true;
                mapp[p][q].traverse_cost=1;
                
            }
            break;
        }
    };
    std::cout << "\n        Would you like a map preview? N - 0, Y - any";
    int preview;
    std::cin >> preview;
    if (preview){
        (*this).Print_mapp();
    }
    int x,y;
    while(1){
        std::cout << "\n        Enter the start node coordinates:";
        std::cout << "\n x=";
        std::cin >> x;
        std::cout << "\n y=";
        std::cin >> y;
        if ( x<0 || y<0 || x>m-1 || y>n-1 || mapp[x][y].not_traversable ){
            std::cout << "\n        ERROR! Coordinates invalid or chosen node contains an obstacle. Repeat.";
        }
        else break;
    }
    start_x=x;
    start_y=y;
    startptr=&mapp[x][y];
    startptr->x=start_x;
    startptr->y=start_y;
    while(1){
        std::cout << "\n        Enter the goal node coordinates:";
        std::cout << "\n x=";
        std::cin >> x;
        std::cout << "\n y=";
        std::cin >> y;
        if ( x<0 || y<0 || x>m-1 || y>n-1 || mapp[x][y].not_traversable ){
            std::cout << "\n        ERROR! Coordinates invalid or chosen node contains an obstacle. Repeat.";
        }
        else break;
    }
    goal_x=x;
    goal_y=y;
    goalptr=&mapp[x][y];
    goalptr->x=goal_x;
    goalptr->y=goal_y;
    return;
}

int main(){
    int pom_m, pom_n;
    std::cout << "\n        Enter number of rows: m=";
    std::cin >> pom_m;
    std::cout << "\n        Enter number of columns: n=";
    std::cin >> pom_n;
    std::cout << "\n        Running D*-Lite!";
   // DstarLite example3(pom_m, pom_n);
   //  example3.Generate_mapp_DS();
   // example3.Compute_forever();
    std::cout << "\n        Running Astar!";
    Astar example(pom_m,pom_n);
    example.Generate_mapp();
    example.Compute_shortest_path();
    std::cout << "\n        Now running Theta!";
    Theta example2(pom_m, pom_n);
    example2.Generate_mapp();
    example2.Compute_shortest_path();
    return 0;
}