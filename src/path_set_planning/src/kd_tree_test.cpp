#include <stdlib.h>
#include <time.h>
#include <chrono>
#include <iostream>
#include <vector>
#include <queue>
#include "kd_tree.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;

RRTStarNode* FindNearestNodeByBFS(RRTStarNode* root, Point2f test_pt) {
    queue<RRTStarNode*> level;
    level.push(root);
    RRTStarNode* res;

    float min_dist_square = FLT_MAX;
    while (level.empty() == false) {
        int level_size = level.size();
        for (int i = 0; i < level_size; i++) {
            RRTStarNode* cur_node = level.front();
            level.pop();
            if (cur_node->left != nullptr)
                level.push(cur_node->left);
            if (cur_node->right != nullptr)
                level.push(cur_node->right);
            
            float cur_dist_square = SquaredNorm(cur_node->pos - test_pt);
            if (cur_dist_square < min_dist_square) {
                res = cur_node;
                min_dist_square = cur_dist_square;
            }
        }
    }
    return res;
}


int main() {
    srand(time(NULL));
    float x_range = 100, y_range = 100,
          div_x = RAND_MAX / x_range,
          div_y = RAND_MAX / y_range;
    float x_min = 40, x_max = 50, y_min = 90, y_max = 95;
    int node_num = 1000, range_search_cnt = 0;

    vector<RRTStarNode*> node_ptr_vec(node_num);
    for (int i = 0; i < node_num; i++) {
        float x = rand() / div_x, y = rand() / div_y;
        node_ptr_vec[i] = new RRTStarNode(Point2f(x, y));

        if (x_min <= x && x <= x_max && y_min <= y && y <= y_max)
            range_search_cnt++;
    }

    kdTree test_kd_tree;
    for (auto& ptr : node_ptr_vec)
        test_kd_tree.Add(ptr); 

    queue<RRTStarNode*> level;
    level.push(test_kd_tree.kd_tree_root_);

    cout << "kd-tree structure: \n";
    while (level.empty() == false) {
        int level_size = level.size();
        for (int i = 0; i < level_size; i++) {
            RRTStarNode* cur_node = level.front();
            level.pop();
            if (cur_node->left != nullptr)
                level.push(cur_node->left);
            if (cur_node->right != nullptr)
                level.push(cur_node->right);
            cout << cur_node->pos << "---";
        }
        cout << "\n";
    }

    int test_pt_num = 100;
    vector<Point2f> test_pt_vec(test_pt_num);
    vector<RRTStarNode*> correct_res_vec(test_pt_num), test_res_vec(test_pt_num);
    for (int i = 0; i < test_pt_num; i++) {
        float x = rand() / div_x, y = rand() / div_y;
        test_pt_vec[i] = Point2f(x, y);
    }

    auto start_1 = high_resolution_clock::now();
    for (int i = 0; i < test_pt_num; i++)
        correct_res_vec[i] = FindNearestNodeByBFS(test_kd_tree.kd_tree_root_, test_pt_vec[i]);
    auto stop_1 = high_resolution_clock::now();
    auto duration_1 = duration_cast<milliseconds>(stop_1 - start_1);

    auto start_2 = high_resolution_clock::now();
    for (int i = 0; i < test_pt_num; i++)
        test_res_vec[i] = test_kd_tree.FindNearestNode(test_pt_vec[i]);
    auto stop_2 = high_resolution_clock::now();
    auto duration_2 = duration_cast<milliseconds>(stop_2 - start_2);

    for (int i = 0; i < test_pt_num; i++)
        cout << correct_res_vec[i]->pos << "-" << test_res_vec[i]->pos << '\n';
    cout << "Running time for BFS is " << duration_1.count() << " ms\n";
    cout << "Running time for kd-tree is " << duration_2.count() << " ms\n";

    vector<RRTStarNode*> range_search_res = test_kd_tree.RanageSearch(x_min, x_max, y_min, y_max);
    cout << "Range search for " << x_min << " <= x <= " << x_max << ", " << y_min << " <= y <= " << y_max << ":\n";
    cout << range_search_res.size() << " point(s) are found out of " << range_search_cnt << "\n";
    for (auto ptr : range_search_res)
        cout << ptr->pos << " "; 
    cout << endl;

    return 0;
}