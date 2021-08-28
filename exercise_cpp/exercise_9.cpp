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
        NodeIndex start_parent_id;
        NodeIndex goal_parent_id;
        NodeIndex start_id;
        Node start_node;
        Node goal_node;
        map<NodeIndex, Node> closed_map;

    public:
        Dijkstra(vector<vector<bool>> gridmap_, int start_x_, int start_y_, int goal_x_, int goal_y_){
            cout << "constructor is called" << endl;
            gridmap = gridmap_;
            x_max = gridmap.size();
            y_max = gridmap[0].size();
            start_id = {start_x_, start_y_};
            start_parent_id = {-1, -1};
            goal_parent_id = {-2, -2};
            start_node = {start_x_, start_y_, 0.0, start_parent_id};
            goal_node = {goal_x_, goal_y_, INT_MAX, goal_parent_id};
        }


    private:
        vector<Option> calc_options(int x, int y){
            cout << "calc_options is called" << endl;

            vector<Option> options;
            Option o;
            
            if (x+1 < x_max){
                if (y+1 < y_max && !gridmap[y+1][x+1]){
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
                if (y+1 < y_max && !gridmap[y+1][x-1]){
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
                if (y+1 < y_max && !gridmap[y+1][x]){
                    o = {0, 1, 1};
                    options.push_back(o);
                }
                if (y-1 >= 0 && !gridmap[y-1][x]){
                    o = {0, -1, 1};
                    options.push_back(o);
                }
            }
            
            //cout << "o's size = " << options.size() << endl;
            cout << "calc_options finish" << endl;
            return options;
        }


    private:
        /**
         * @brief コストが最小のノードを探す
         */
        NodeIndex find_MinCostNode_id(map<NodeIndex, Node> m){
            cout << "find_MinCostNode_id called" << endl;
            NodeIndex min_cost_id;
            auto begin = m.begin();
            auto end = m.end();

            map<NodeIndex, Node>::iterator pm;
            pm = m.begin();
            //int i = 0;
            
            while (pm != end){
                //i += 1;
                //cout << (pm->first).x << ", " << (pm->first).y << endl;
                if (pm == begin){
                    min_cost_id = pm->first;
                }
                else if (pm->second.cost < m[min_cost_id].cost){
                    min_cost_id = pm->first;
                }
                ++pm;  // 次に動かす
            }
            cout << "find_MinCostNode_id quite" << endl;
            return min_cost_id;
        }




    private:
        void planning(
        ){
            cout << "run planning" << endl;
            Node temp_node;
            int temp_x, temp_y;
            NodeIndex temp_id;
            vector<Option> os;
            Node node;
            NodeIndex node_id;

            map<NodeIndex, Node> open_set;  // 未決定のノード
            map<NodeIndex, Node> closed_set;  // 決定済みのノード
            open_set.insert(pair<NodeIndex, Node>(start_id, start_node));
            
            cout << "while loop is running..." << endl;

            int counter = 0;
            while (1){
                counter += 1;
                cout << endl << "while count = " << counter << endl;
                temp_id = find_MinCostNode_id(open_set);
                
                temp_node = open_set[temp_id];
                cout << "temp = " << temp_id.x << ", " << temp_id.y << endl;
                //cout << "" << endl;
                if (temp_node.x == goal_node.x && temp_node.y == goal_node.y){
                    goal_node.parent = temp_node.parent;
                    goal_node.cost = temp_node.cost;
                    cout << "serching complete!" << endl;
                    cout << "goal_node_paretnt = " << goal_node.parent.x << ", " << goal_node.parent.y << endl;
                    break;
                }
                else{
                    cout << "don't complete. try again" << endl;
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
                        //cout << "pattern 1" << endl;
                        continue;
                    }
                    else if(open_set.count(node_id) == 0){
                        // 未探索のとき
                        //cout << "pattern 2" << endl;
                        open_set[node_id] = node;
                    }
                    else{
                        // 探索済みだが未決定
                        //cout << "pwttern 3" << endl;
                        if (open_set[node_id].cost >= node.cost){
                            open_set[node_id] = node;
                        }
                    }
                }
            }
            cout << "finish planning" << endl;
        }


    private:
    /**
     * @brief 決定済みノード集合から最短パスを構築
     */
        std::tuple<vector<int>, vector<int>, double> compute_optiomal_path(
        ){
            cout << "compute_optiomal_path is called" << endl;
            vector<int> rx, ry;
            double cost = goal_node.cost;
            NodeIndex parent = goal_node.parent;
            Node node;
            cout << parent.x << ", " << parent.y << endl;

            while (parent.x != start_id.x && parent.y != start_id.y){
                node = closed_map[parent];
                rx.push_back(node.x);
                ry.push_back(node.y);
                cost += node.cost;
                parent = node.parent;
            }
            cout << "passsize = "<< rx.size() << endl;
            std::tuple<vector<int>, vector<int>, double> z = std::make_tuple(rx, ry, cost);
            cout << "compute_optiomal_path is finished" << endl;
            return z;
        }

    public:
        void do_exercise(){
            map<NodeIndex, Node> closed_map;
            planning();
            std::tuple<vector<int>, vector<int>, double> z = compute_optiomal_path();
            vector<int>& rx = std::get<0>(z);
            vector<int>& ry = std::get<1>(z);
            double& cost = std::get<2>(z);
            cout << "optiomal path is " << endl << "(x, y) = " << endl;
            for (int i=0; i<rx.size(); i++){
                cout << rx[i] << ", " << ry[i] << endl;
            }
            cout << "total cost = " << cost  << "[m]" << endl;
        }
};




int main(){
    cout << "main is running..." << endl;

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
    sim.do_exercise();

    cout << "main is finished" << endl;
    return 0;
}