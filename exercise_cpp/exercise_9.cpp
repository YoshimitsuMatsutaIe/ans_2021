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
#include <tuple>

using std::cout;
using std::endl;
using std::vector;
using std::map;
using std::pair;


/**
 * @brief Nodeのキー
 */
struct NodeIndex {
    int x;  // x座標
    int y;  // y座標
};

bool operator<(const NodeIndex& a, const NodeIndex& b){
    return std::tie(a.x, a.y) < std::tie(b.x, b.y);
}

/**
    @brief  Node
*/
struct Node {
    int x;  // x座標
    int y;  // y座標
    double cost;
    NodeIndex parent;
};

struct Option {
    int dx;
    int dy;
    double dcost;
};


class Dijkstra {

    public:
        vector<vector<bool>> gridmap;
    
    private:
        int x_max;
        int y_max;
        Node start_node;
        Node goal_node;

    public:
        Dijkstra(vector<vector<bool>> gridmap_, int start_x_, int start_y_, int goal_x_, int goal_y_){
            gridmap = gridmap_;
            x_max = gridmap.size();
            y_max = gridmap[0].size();
            start_node = {start_x_, start_y_, 0.0, -1};
            goal_node = {goal_x_, goal_y_, -1, -2};
        }


    private:
        vector<Option> calc_options(int x, int y){
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


    private:
        /**
         * @brief コストが最小のノードを探す
         */
        NodeIndex find_MinCostNode_id(map<NodeIndex, Node> m){
            NodeIndex min_cost_id;
            auto begin = m.begin();
            auto end = m.end();

            map<NodeIndex, Node>::iterator pm;
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


    private:
    /**
     * @brief 決定済みノード集合から最短パスを構築
     */
        std::tuple<vector<int>, vector<int>, double> compute_optiomal_path(
            map<NodeIndex, Node> closed_set
        ){
            vector<int> rx, ry;
            NodeIndex parent = goal_node.parent;
            double cost = goal_node.cost;
            Node node;

            NodeIndex start_id = {start_node.x, start_node.y};

            while (parent.x != start_id.x && parent.y != start_id.y){
                node = closed_set[parent];
                rx.push_back(node.x);
                ry.push_back(node.y);
                cost += node.cost;
                parent = node.parent;
            }

            std::tuple<vector<int>, vector<int>, double> z = std::make_tuple(rx, ry, cost);
            return z;
        }


    private:
        map<NodeIndex, Node> planning(
        ){
            Node temp_node;
            int temp_x, temp_y;
            NodeIndex temp_id;
            vector<Option> os;
            Node node;
            NodeIndex node_id;

            map<NodeIndex, Node> open_set;  // 未決定のノード
            map<NodeIndex, Node> closed_set;  // 決定済みのノード

            NodeIndex start_id = {start_node.x, start_node.y};
            open_set.insert(pair<NodeIndex, Node>(start_id, start_node));

            while (1){
                temp_id = find_MinCostNode_id(open_set);
                temp_node = open_set[temp_id];

                if (temp_node.x == goal_node.x && temp_node.y == goal_node.y){
                    goal_node.parent = temp_node.parent;
                    goal_node.cost = temp_node.cost;
                    break;
                }

                open_set.erase(temp_id);  // 未決定の集合から削除
                closed_set[temp_id] = temp_node;

                os = calc_options(temp_node.x, temp_node.y);
                for (int i=0; i<os.size(); i++){
                    node = {
                        temp_node.x + os[i].dx,
                        temp_node.y + os[i].dy,
                        temp_node.cost + os[i].dcost
                    };
                    node_id = {node.x, node.y};

                    if (closed_set.count(node_id) == 1){
                        // 決定済み
                        continue;
                    }
                    else if(open_set.count(node_id) == 0){
                        // 未探索のとき
                        open_set[node_id] = node;
                    }
                    else{
                        // 探索済みだが未決定
                        if (open_set[node_id].cost >= node.cost){
                            open_set[node_id] = node;
                        }
                    }
                }
            }

            return closed_set;
        }
};



void ex1(){
    vector<vector<bool>> gridmap = {
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 1, 1, 1, 1, 0, 0, 0, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 1, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 1, 0, 0, 0, 1, 0},
        {0, 0, 0, 1, 1, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
    };  // 生マップ


    Dijkstra sim(gridmap, 0, 0, 9, 9);
    //map<NodeIndex, Node> closed_set = sim.planning(start_x, start_y, goal_x, goal_y, gridmap);
    //std::tuple<vector<int>, vector<int>, double> z = compute_optiomal_path();
    

}

int main(){
    cout << "running..." << endl;

    ex1();
    cout << "終了" << endl;
    return 0;
}