#ifndef RRTSTAR_HEADER_INCLUDED
#define RRTSTAR_HEADER_INCLUDED

#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "obstacles.hpp"
#include "kd_tree.hpp"

using namespace cv;

class RRTStarPlanner {
public:
    Point2f start_pos_;
    Point2f target_pos_;
    std::vector<PolygonObstacle> obstacles_;
    std::vector<std::vector<int>> pure_visibility_passage_pair_;
    std::vector<std::vector<Point2f>> pure_visibility_passage_pts_;
    std::vector<std::vector<int>> extended_visibility_passage_pair_;
    std::vector<std::vector<Point2f>> extended_visibility_passage_pts_;
    float step_len_;
    float error_dis_;
    float radius_;
    int cost_function_type_;
    bool use_extended_vis_check_;
    float passage_width_weight_;
    Size2f config_size_;
    RRTStarNode* start_node_;
    RRTStarNode* target_node_;
    kdTree kd_tree_;
    int MAX_GRAPH_SIZE = 10000;
    int CUR_GRAPH_SIZE = 0;
    int update_cost_cnt_ = 0;
    bool plan_success_ = false;

    RRTStarPlanner(): start_node_(nullptr), target_node_(nullptr) {}
    RRTStarPlanner(Point2f start, Point2f target, vector<PolygonObstacle> obs, float step_len = 18, 
                   float radius = 10, float error_dis = 10, 
                   Size2f config_size = Size2f(640, 480), 
                   int cost_function_type = 0,
                   float passage_width_weight = 100,
                   bool use_extended_vis_check = true): 
        start_pos_(start), 
        target_pos_(target), 
        obstacles_(obs),
        step_len_(step_len), 
        radius_(radius),
        error_dis_(error_dis),
        config_size_(config_size),
        cost_function_type_(cost_function_type),
        passage_width_weight_(passage_width_weight),
        use_extended_vis_check_(use_extended_vis_check) {
        start_node_ = new RRTStarNode(start);
        target_node_ = new RRTStarNode(target);
        kd_tree_.Add(start_node_);
        CUR_GRAPH_SIZE++;

        auto pure_visibility_check_res = PureVisibilityPassageCheck(obstacles_);
        pure_visibility_passage_pair_ = pure_visibility_check_res.first;
        pure_visibility_passage_pts_ = pure_visibility_check_res.second;

        auto extended_visibility_check_res = ExtendedVisibilityPassageCheck(obstacles_);
        extended_visibility_passage_pair_ = extended_visibility_check_res.first;
        extended_visibility_passage_pts_ = extended_visibility_check_res.second; 

        std::cout << "RRT* path planner instanced with cost functio type: " << cost_function_type_ 
                << "\n(Any value not equal to 1 or 2: Default cost function: len - weight * passed_min_passage_width"
                << "\n1: Ratio cost function: len / passed_min_passage_width)"
                << "\n2: Path length cost function with clearance limit\n";
        if (cost_function_type_ != 1 && cost_function_type_ != 2) {
            std::cout << "with a passage width weight of " << passage_width_weight_ << "\n\n";       
            start_node_->cost = -passage_width_weight * start_node_->min_passage_width;
        }
    }
    ~RRTStarPlanner();
    RRTStarPlanner(const RRTStarPlanner&);
    RRTStarPlanner& operator=(const RRTStarPlanner&);


    bool Plan(Mat source_img, float interior_delta = 0.01, bool plan_in_interior = false) {            
        srand(time(NULL));
        plan_success_ = false;
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height,
              min_cost = FLT_MAX;
        
        std::vector<PolygonObstacle> obstacles_without_boundary(obstacles_.size() - 4);
        for (int i = 0; i < obstacles_without_boundary.size(); i++)
            obstacles_without_boundary[i] = obstacles_[i + 4];

        Point2f rand_pos = Point2f(0, 0);
        int sample_times = 0;
        while (CUR_GRAPH_SIZE < MAX_GRAPH_SIZE) {
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;
            sample_times++;

            if (CUR_GRAPH_SIZE % (MAX_GRAPH_SIZE / 20) == 0) {
                rand_pos = target_pos_ + rand_pos / cv::norm(rand_pos);  // Instead of directly using rand_pos = target_pos_, add a small random shift to avoid resulting 
                CUR_GRAPH_SIZE++;                                        // in samples in the same position.
            } 
            // Note it is important to avoid identical sample positions if target biasing is used.
            // When assigning rand_pos as target_pos_, the same new node position will be generated if there is a nearest_node with a distance 
            // smaller than step_len_ / 2 to target_pos_. Nodes with the same position may form a loop of parent-child chain after rewiring BECAUSE
            // FLOAT LOSES PRECISION IN COMPUTATION. Suppose near_node and new_node have the same position. The cost updated by linking new_node and 
            // near_node should be equivalent to near_node->cost, and rewiring should not be performed. However, the computed updated cost may be 
            // a bit smaller than near_node->cost due to float precision problem. As such, parent-child relation is wrongly updated and possibly 
            // leading to a loop in the tree though the algorithm is sound.

            RRTStarNode* nearest_node = kd_tree_.FindNearestNode(rand_pos);           
            // if (normSqr(nearest_node->pos - rand_pos) < 1e-4)
            //    continue;

            RRTStarNode* new_node = GenerateNewNode(nearest_node, rand_pos);
            if (PathObstacleFree(nearest_node, new_node)) {
                if (plan_in_interior && cost_function_type_ == 2)
                    if (MinDistanceToObstaclesVec(obstacles_without_boundary, new_node->pos) < interior_delta) {
                        delete new_node;
                        continue;
                    }
                if (cost_function_type_ == 2)
                    RewireWithPathLengthCost(nearest_node, new_node, source_img);
                else
                    Rewire(nearest_node, new_node, source_img);
                
                kd_tree_.Add(new_node);
                if (normSqr(new_node->pos - target_pos_) <= error_dis_* error_dis_) {
                    if (new_node->cost < min_cost) {
                        target_node_->parent = new_node;
                        min_cost = new_node->cost;
                    }      
                    plan_success_ = true;
                }
                CUR_GRAPH_SIZE++;
                // circle(source_img, new_node->pos, 3, Scalar(0,255,0), -1);
            }
            else    
                delete new_node;

            if (cost_function_type_ == 2 && sample_times >= 20000 && plan_success_ == false)
                return false;

/*             circle(source_img, start_pos_, 4, Scalar(255,0,0), -1);
            circle(source_img, target_pos_, 4, Scalar(255,0,0), -1);
            imshow("RRT* path planning", source_img);
            waitKey(1);
            if (CUR_GRAPH_SIZE == MAX_GRAPH_SIZE)
                destroyWindow("RRT* path planning"); */
        }
        if (plan_success_ == false)
            std::cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path found.\n";
        else
            std::cout << "Path found with cost: " << min_cost
                 << "\nTotal sample number: " << sample_times
                 << '\n';   
        return plan_success_;
    }

    bool PlanHomotopicPath(Mat source_img, std::vector<Point2f>& reference_path, Point2f initial_pt, Point2f target_pt) {       
        srand(time(NULL));
        plan_success_ = false;
        float div_width = RAND_MAX / config_size_.width,
              div_height = RAND_MAX / config_size_.height,
              min_cost = FLT_MAX;

        std::vector<PolygonObstacle> obstacles_without_boundary(obstacles_.size() - 4);
        for (int i = 0; i < obstacles_without_boundary.size(); i++)
            obstacles_without_boundary[i] = obstacles_[i + 4];
        obstacles_ = obstacles_without_boundary;

        Point2f rand_pos = Point2f(0, 0);
        while (CUR_GRAPH_SIZE < MAX_GRAPH_SIZE) {
            // std::cout << CUR_GRAPH_SIZE << "\n";
            rand_pos.x = rand() / div_width;
            rand_pos.y = rand() / div_height;

            if (CUR_GRAPH_SIZE % (MAX_GRAPH_SIZE / 20) == 0) {
                rand_pos = target_pos_ + rand_pos / cv::norm(rand_pos);   
                CUR_GRAPH_SIZE++;                                       
            } 

            RRTStarNode* nearest_node = kd_tree_.FindNearestNode(rand_pos);
            RRTStarNode* new_node = GenerateNewNode(nearest_node, rand_pos);
            if (PathObstacleFree(nearest_node, new_node)) {
                if (PtPathHomotopyCheck(new_node->pos, reference_path, 2, 30) == false) {
                    delete new_node;
                    continue;
                }
                RewireWithPathLengthCost(nearest_node, new_node, source_img);

                kd_tree_.Add(new_node);
                if (normSqr(new_node->pos - target_pos_) <= error_dis_* error_dis_) {
                    if (new_node->cost < min_cost) {
                        target_node_->parent = new_node;
                        min_cost = new_node->cost;
                    }      
                    plan_success_ = true;
                }
                CUR_GRAPH_SIZE++;
                // circle(source_img, new_node->pos, 3, Scalar(0,255,0), -1);
            }
            else    
                delete new_node;

        }
        if (plan_success_ == false)
            std::cout << "MAX_GRAPH_SIZE: " << MAX_GRAPH_SIZE << " is achieved with no path found.\n";
        else
            std::cout << "Path found with cost: " << min_cost << '\n';   
        return plan_success_;
    }
    
    bool PtPathHomotopyCheck(const Point2f& test_pt, const std::vector<Point2f>& reference_path, float lower_bound = 2, float upper_bound = 25) {
        float min_dist_to_path = 1e5;
        Point2f min_dist_path_pt;
        for (auto& path_pt : reference_path) {
            float cur_distance_to_path = cv::norm(test_pt - path_pt);
            if (cur_distance_to_path < min_dist_to_path) {
                min_dist_to_path = cur_distance_to_path;
                min_dist_path_pt = path_pt;
            }
        }
        return min_dist_to_path > lower_bound && min_dist_to_path < upper_bound  && PtsObstacleFree(test_pt, min_dist_path_pt);
    }    

    RRTStarNode* GenerateNewNode(RRTStarNode* nearest_node, Point2f& rand_pos) {
        Point2f direction = (rand_pos - nearest_node->pos) / cv::norm((rand_pos - nearest_node->pos));
        Point2f new_pos = nearest_node->pos + step_len_ * direction;
        new_pos.x = std::max((float)0.0, new_pos.x);
        new_pos.x = std::min(config_size_.width, new_pos.x);
        new_pos.y = std::max((float)0.0, new_pos.y);
        new_pos.y = std::min(config_size_.height, new_pos.y);
        RRTStarNode* new_node = new RRTStarNode(new_pos);
        return new_node;
    }

    void Rewire(RRTStarNode* nearest_node, RRTStarNode* new_node, Mat source_img) {
        float gamma_star = 800,
              gamma = gamma_star * sqrt(log(CUR_GRAPH_SIZE) * 3.32 / CUR_GRAPH_SIZE),
              radius_alg = std::min(gamma, step_len_);
        float x_min = std::max((float)0.0, new_node->pos.x - radius_alg), x_max = std::min(new_node->pos.x + radius_alg, config_size_.width),
              y_min = std::max((float)0.0, new_node->pos.y - radius_alg), y_max = std::min(new_node->pos.y + radius_alg, config_size_.height); 

        std::vector<RRTStarNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);

        // find minimal-cost path
        RRTStarNode* min_cost_node = nearest_node;
        float min_cost = UpdatePassageEncodeCost(nearest_node, new_node);
        for (auto near_node : near_set) {     
            float cur_cost = UpdatePassageEncodeCost(near_node, new_node);       
            if (cur_cost < min_cost && PathObstacleFree(near_node, new_node)) {
                min_cost_node = near_node;
                min_cost = cur_cost;
            }
        }

        new_node->parent = min_cost_node;
        min_cost_node->children.push_back(new_node);
        new_node->cost = min_cost;
        UpdateNewNodeMinCurPassageWidth(min_cost_node, new_node);
        // line(source_img, min_cost_node->pos, new_node->pos, Scalar(0, 0, 200), 1.5);

        for (auto near_node : near_set) {
            if (near_node == min_cost_node)
                continue;

            float new_near_node_cost = UpdatePassageEncodeCost(new_node, near_node);
            if (new_near_node_cost < near_node->cost && PathObstacleFree(near_node, new_node)) {
                near_node->cost = new_near_node_cost;
                UpdateNewNodeMinCurPassageWidth(new_node, near_node);
                ChangeParent(near_node, new_node);
            }
        }
    }

    void RewireWithPathLengthCost(RRTStarNode* nearest_node, RRTStarNode* new_node, Mat source_img) {
        float gamma_star = 800,
              gamma = gamma_star * sqrt(log(CUR_GRAPH_SIZE) * 3.32 / CUR_GRAPH_SIZE),
              radius_alg = std::min(gamma, step_len_);
        // radius_alg = 20;
        float x_min = std::max((float)0.0, new_node->pos.x - radius_alg), x_max = std::min(new_node->pos.x + radius_alg, config_size_.width),
              y_min = std::max((float)0.0, new_node->pos.y - radius_alg), y_max = std::min(new_node->pos.y + radius_alg, config_size_.height); 

        std::vector<RRTStarNode*> near_set = kd_tree_.RanageSearch(x_min, x_max, y_min, y_max);

        // find minimal-cost path
        RRTStarNode* min_cost_node = nearest_node;
        float min_cost = min_cost_node->cost + cv::norm(new_node->pos - min_cost_node->pos);
        for (auto near_node : near_set) {     
            float cur_cost = near_node->cost + cv::norm(new_node->pos - near_node->pos);       
            if (cur_cost < min_cost && PathObstacleFree(near_node, new_node)) {
                min_cost_node = near_node;
                min_cost = cur_cost;
            }
        }

        new_node->parent = min_cost_node;
        min_cost_node->children.push_back(new_node);
        new_node->cost = min_cost;
        // line(source_img, min_cost_node->pos, new_node->pos, Scalar(0, 0, 200), 1.5);

        for (auto near_node : near_set) {
            float new_near_node_cost = new_node->cost + cv::norm(near_node->pos - new_node->pos);
            if (new_near_node_cost < near_node->cost && PathObstacleFree(near_node, new_node)) {
                near_node->cost = new_near_node_cost;
                ChangeParentPathLengthCost(near_node, new_node);
            }
        }
    }

    bool PathObstacleFree(RRTStarNode* near_node, RRTStarNode* new_node) {
        for (auto& obs : obstacles_)
            if (ObstacleFree(obs, near_node->pos, new_node->pos) == false)
                return false;
        return true;
    }

    bool PtsObstacleFree(const Point2f& pt_1, const Point2f& pt_2) {
        for (auto& obs : obstacles_)
            if (ObstacleFree(obs, pt_1, pt_2) == false)
                return false;
        return true;
    }

    float UpdatePassageEncodeCost(RRTStarNode* near_node, RRTStarNode* new_node) {
        float passed_passage_width;
        if (use_extended_vis_check_ == true)
            passed_passage_width = GetMinPassageWidthPassed(extended_visibility_passage_pts_, near_node->pos, new_node->pos);
        else
            passed_passage_width = GetMinPassageWidthPassed(pure_visibility_passage_pts_, near_node->pos, new_node->pos);

        float res = 0;
        if (passed_passage_width < 0) {
            if (cost_function_type_ != 1)
                res = near_node->cost + cv::norm(new_node->pos - near_node->pos);
            else
                res = near_node->cost + cv::norm(new_node->pos - near_node->pos) / near_node->min_passage_width;
        }
        else {
            if (cost_function_type_ != 1) {
                if (passed_passage_width >= near_node->min_passage_width)
                    res = near_node->cost + cv::norm(new_node->pos - near_node->pos);
                else
                    res = near_node->cost + cv::norm(new_node->pos - near_node->pos) + passage_width_weight_ * 
                        (near_node->min_passage_width - passed_passage_width); 
            }
            else {
                res = (near_node->cost * near_node->min_passage_width + cv::norm(new_node->pos - near_node->pos)) 
                        / std::min(near_node->min_passage_width, passed_passage_width); 
            }
        }
        return res;      
    }

    void UpdateNewNodeMinCurPassageWidth(RRTStarNode* parent_node, RRTStarNode* new_node) {
        new_node->min_passage_width = parent_node->min_passage_width;
        // float min_passage_width_new_edge_passes = GetMinPassageWidthPassed(obstacles_, parent_node->pos, new_node->pos);
        float min_passage_width_new_edge_passes;
        if (use_extended_vis_check_ == true)
            min_passage_width_new_edge_passes = GetMinPassageWidthPassed(extended_visibility_passage_pts_, parent_node->pos, new_node->pos);
        else
            min_passage_width_new_edge_passes = GetMinPassageWidthPassed(pure_visibility_passage_pts_, parent_node->pos, new_node->pos);

        new_node->cur_passage_width = min_passage_width_new_edge_passes;
        if (min_passage_width_new_edge_passes > 0) 
            new_node->min_passage_width = std::min(new_node->min_passage_width, min_passage_width_new_edge_passes);
    }

    void ChangeParent(RRTStarNode* child_node, RRTStarNode* new_parent) {
        RRTStarNode* old_parent = child_node->parent;
        if (old_parent != nullptr) 
            old_parent->children.remove(child_node);

        child_node->parent = new_parent;
        new_parent->children.push_back(child_node);
        
        // std::cout << "\rupdate cost counter: " << ++update_cost_cnt_;
        std::queue<RRTStarNode*> node_level;
        node_level.push(child_node);
        while (node_level.empty() == false) {
            RRTStarNode* cur_node = node_level.front();
            node_level.pop();

            for (auto& cur_child : cur_node->children) {
                if (cur_child->cur_passage_width > 0)
                    cur_child->min_passage_width = std::min(cur_child->cur_passage_width, cur_node->min_passage_width);
                else
                    cur_child->min_passage_width = cur_node->min_passage_width;

                if (cost_function_type_ != 1)
                    cur_child->cost = cur_node->cost + passage_width_weight_ * cur_node->min_passage_width + cv::norm(cur_node->pos - cur_child->pos)
                                    - passage_width_weight_ * cur_child->min_passage_width;
                else 
                    cur_child->cost = (cur_node->cost * cur_node->min_passage_width + cv::norm(cur_node->pos - cur_child->pos)) / cur_child->min_passage_width;

                node_level.push(cur_child);
            }
        }
    }

    void ChangeParentPathLengthCost(RRTStarNode* child_node, RRTStarNode* new_parent) {
        RRTStarNode* old_parent = child_node->parent;
        if (old_parent != nullptr) 
            old_parent->children.remove(child_node);

        child_node->parent = new_parent;
        new_parent->children.push_back(child_node);
        
        std::queue<RRTStarNode*> node_level;
        node_level.push(child_node);
        while (node_level.empty() == false) {
            RRTStarNode* cur_node = node_level.front();
            node_level.pop();

            for (auto& cur_child : cur_node->children) {
                cur_child->cost = cur_node->cost + cv::norm(cur_child->pos - cur_node->pos);
                node_level.push(cur_child);
            }
        }
    }

    std::vector<RRTStarNode*> GetPath() {
        std::vector<RRTStarNode*> res;
        if (!plan_success_) {
            std::cout << "No valid path is available.\n";
            return res;
        }
        RRTStarNode* reverse_node = target_node_;
        while (reverse_node) {
            res.push_back(reverse_node);
            reverse_node = reverse_node->parent;
        }
        reverse(res.begin(), res.end());
        return res;   
    }  

    std::vector<Point2f> GetPathInPts() {
        std::vector<RRTStarNode*> node_path = GetPath();
        std::vector<Point2f> res(node_path.size());

        for (int i = 0; i < node_path.size(); i++)
            res[i] = node_path[i]->pos;
        return res;
    }  
};


RRTStarPlanner::~RRTStarPlanner() {
    // delete start_node_;
    // delete target_node_;
}

RRTStarPlanner::RRTStarPlanner(const RRTStarPlanner& planner) {
    start_pos_ = planner.start_pos_;
    target_pos_ = planner.target_pos_;
    obstacles_ = planner.obstacles_;
    step_len_ = planner.step_len_;
    error_dis_ = planner.error_dis_;
    config_size_ = planner.config_size_;
    if (!start_node_)
        start_node_ = new RRTStarNode((*planner.start_node_));
    else
        *start_node_ = *(planner.start_node_);
    if (!target_node_)
        target_node_ = new RRTStarNode(*(planner.target_node_));
    else
        *target_node_ = *(planner.target_node_);
    MAX_GRAPH_SIZE = planner.MAX_GRAPH_SIZE;
    CUR_GRAPH_SIZE = planner.CUR_GRAPH_SIZE;
    plan_success_ = planner.plan_success_;        
}

RRTStarPlanner& RRTStarPlanner::operator=(const RRTStarPlanner& rhs) {
    start_pos_ = rhs.start_pos_;
    target_pos_ = rhs.target_pos_;
    obstacles_ = rhs.obstacles_;
    step_len_ = rhs.step_len_;
    error_dis_ = rhs.error_dis_;
    config_size_ = rhs.config_size_;
    if (!start_node_)
        start_node_ = new RRTStarNode((*rhs.start_node_));
    else
        *start_node_ = *(rhs.start_node_);
    if (!target_node_)
        target_node_ = new RRTStarNode(*(rhs.target_node_));
    else
        *target_node_ = *(rhs.target_node_);
    MAX_GRAPH_SIZE = rhs.MAX_GRAPH_SIZE;
    CUR_GRAPH_SIZE = rhs.CUR_GRAPH_SIZE;
    plan_success_ = rhs.plan_success_;
}

#endif
