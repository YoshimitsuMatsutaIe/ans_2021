/**
 * @file 
 * @brief 最短経路探索
 * @author matsuta
 * @note 
 */


#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>

using std::cout;
using std::endl;
using std::vector;
using std::map;
using std::pair;

/**
    @brief  Node
*/
struct Node{
    int x;  // x座標
    int y;  // y座標
    double cost;
    int parent;
};

struct Option{
    int dx;
    int dy;
    double dcost;
};


vector<Option> calc_options(int x, int y, int x_max, int y_max, vector<vector<bool>> gridmap){
    vector<Option> options;
    Option o;
    
    if (x+1 <= x_max){
        if (y+1 <= y_max && !gridmap[y+1][x+1]){
            o = {1, 1, sqrt(2)};
            options.push_back(o);
        }
        if (y-1 >= 0 && !gridmap[y-1][x+1]){
            o = {1, -1, sqrt(2)};
            options.push_back(o);
        }
        if (!gridmap[y][x+1]){
            o = {1, 0, 1};
            options.push_back(o);
        }
    }
    if (x-1 >= 0){
        if (y+1 <= y_max && !gridmap[y+1][x-1]){
            o = {-1, 1, sqrt(2)};
            options.push_back(o);
        }
        if (y-1 >= 0 && !gridmap[y-1][x-1]){
            o = {-1, -1, sqrt(2)};
            options.push_back(o);
        }
        if (!gridmap[y][x-1]){
            o = {-1, 0, 1};
            options.push_back(o);
        }
    }
    else{
        if (y+1 <= y_max && !gridmap[y+1][x]){
            o = {0, 1, 1};
            options.push_back(o);
        }
        if (y-1 >= 0 && !gridmap[y-1][x]){
            o = {0, -1, 1};
            options.push_back(o);
        }
    }
    
    return options;
}

int find_MinCostNode_id(map<int, Node> m){
    int min_cost_id;
    auto begin = m.begin();
    auto end = m.end();

    map<int, Node>::iterator pm;
    //int min_cost_id = m[begin->first];
    while (pm != m.end()){
        if (pm == m.begin()){
            min_cost_id = pm->first;
        }
        else if (pm->second.cost < m[min_cost_id].cost){
            min_cost_id = pm->first;
        }
    }
    return min_cost_id;
}


std::tuple<vector<int>, vector<int>, double, map<int, Node>> planning(int start_x, int start_y, int goal_x, int goal_y){
    Node start_node = {start_x, start_y, 0.0, -1};
    Node goal_node = {goal_x, goal_y, -1, -2};
    Node temp_node;
    int temp_x, temp_y, temp_id;

    map<int, Node> open_set;  // 未決定のノード
    map<int, Node> closed_set;  // 決定済みのノード

    int start_id = 0;
    open_set.insert(pair<int, Node>(start_id, start_node));

    while (temp_x == goal_node.x && temp_y == goal_node.y){
        temp_id = find_MinCostNode_id(open_set);
        temp_node = open_set[temp_id];

        if 
    }


}


int main(){
    cout << "running..." << endl;



    cout << "終了" << endl;
    return 0;
}