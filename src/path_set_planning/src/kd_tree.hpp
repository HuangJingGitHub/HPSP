#ifndef KD_TREE_INCLUDED
#define KD_TREE_INCLUDED

#include<list>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//using namespace cv;
//using namespace std;

float SquaredNorm(const cv::Point2f& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

struct RRTStarNode {
    cv::Point2f pos;
    float cost = 0;
    float min_passage_width = 10000;
    float cur_passage_width = -1;  // The passage width passed by the edge parent node---current node
    RRTStarNode* parent;
    RRTStarNode* left;
    RRTStarNode* right;
    std::list<RRTStarNode*> children;
    // std::vector<RRTStarNode*> adjacency_list;
    RRTStarNode(): pos(cv::Point2f(0, 0)), cost(0), parent(nullptr), left(nullptr), right(nullptr) {}
    RRTStarNode(cv::Point2f initPos): pos(initPos), cost(0), parent(nullptr), left(nullptr), right(nullptr) {}
};

class kdTree{
private:
    const int kDimension_k_ = 2;
public:
    RRTStarNode* kd_tree_root_;

    kdTree(): kd_tree_root_(nullptr) {}
    kdTree(RRTStarNode* root_node): kd_tree_root_(root_node) {};
    ~kdTree();
    kdTree(const kdTree&);
    kdTree& operator=(const kdTree&);

    void AddWithRoot(RRTStarNode* root, RRTStarNode* new_node, int depth) {
        if (depth % kDimension_k_ == 0) {
            if (new_node->pos.x <= root->pos.x) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else 
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
        else {
            if (new_node->pos.y <= root->pos.y) {
                if (root->left == nullptr) 
                    root->left = new_node;
                else 
                    AddWithRoot(root->left, new_node, depth + 1);
            }
            else {
                if (root->right == nullptr) 
                    root->right = new_node;
                else
                    AddWithRoot(root->right, new_node, depth + 1);
            }
        }
    }

    void Add(RRTStarNode* new_node) {
        if (new_node == nullptr)
            return;

        if (kd_tree_root_ == nullptr)
            kd_tree_root_ = new_node;
        else
            AddWithRoot(kd_tree_root_, new_node, 0);
    }

    RRTStarNode* GetCloestInTwo(RRTStarNode* target, RRTStarNode* candidate_1, RRTStarNode* candidate_2) {
        if (candidate_1 == nullptr)
            return candidate_2;
        if (candidate_2 == nullptr)
            return candidate_1;

        if (SquaredNorm(target->pos - candidate_1->pos) <= SquaredNorm(target->pos - candidate_2->pos))
            return candidate_1;
        return candidate_2;
    }

    RRTStarNode* FindNearestNodeWithRoot(RRTStarNode* root, RRTStarNode* target, int depth) {
        if (root == nullptr)
            return nullptr;
        
        RRTStarNode *next_subtree = nullptr, *other_subtree = nullptr;
        if (depth % kDimension_k_ == 0) {
            if (target->pos.x <= root->pos.x) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }
        }
        else {
            if (target->pos.y <= root->pos.y) {
                next_subtree = root->left;
                other_subtree = root->right;
            }
            else {
                next_subtree = root->right;
                other_subtree = root->left;
            }  
        }
        RRTStarNode *temp_res = FindNearestNodeWithRoot(next_subtree, target, depth + 1),
                    *cur_best = GetCloestInTwo(target, temp_res, root);
        float cur_dist_square = SquaredNorm(target->pos - cur_best->pos), dist_to_boundary_square, dist_to_boundary;
        if (depth % kDimension_k_ == 0) 
            dist_to_boundary = target->pos.x - root->pos.x;
        else
            dist_to_boundary = target->pos.y - root->pos.y;
        dist_to_boundary_square = dist_to_boundary * dist_to_boundary;
        
        if (cur_dist_square >= dist_to_boundary_square) {
            temp_res = FindNearestNodeWithRoot(other_subtree, target, depth + 1);
            cur_best = GetCloestInTwo(target, temp_res, cur_best);
        }
        return cur_best;
    }

    RRTStarNode* FindNearestNode(RRTStarNode* target) {
        return FindNearestNodeWithRoot(kd_tree_root_, target, 0);
    } 
   
    RRTStarNode* FindNearestNode(const cv::Point2f& target_pos) {
        RRTStarNode* target_node = new RRTStarNode(target_pos);
        RRTStarNode* res = FindNearestNodeWithRoot(kd_tree_root_, target_node, 0);
        delete target_node;
        return res;
    }

    void RangeSearchWithRoot(RRTStarNode* root, RRTStarNode* parent, std::vector<RRTStarNode*>& res_pt_vec, 
                            const float& x_min, const float& x_max, 
                            const float& y_min, const float& y_max, int depth) {
        if (root == nullptr)
            return;
        
        if (depth % kDimension_k_ == 0 && parent != nullptr) {
            if (root->pos.y <= parent->pos.y && parent->pos.y < y_min)
                return;
            if (root->pos.y > parent->pos.y && parent->pos.y > y_max)
                return;
        }
        else if (parent != nullptr) {
            if (root->pos.x <= parent->pos.x && parent->pos.x < x_min)
                return;
            if (root->pos.x > parent->pos.x && parent->pos.x > x_max)
                return;        
        }

        if (root->pos.x >= x_min && root->pos.x <= x_max && root->pos.y >= y_min && root->pos.y <= y_max)
            res_pt_vec.push_back(root);
        RangeSearchWithRoot(root->left, root, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);
        RangeSearchWithRoot(root->right, root, res_pt_vec, x_min, x_max, y_min, y_max, depth + 1);           
    }

    std::vector<RRTStarNode*> RanageSearch(const float& x_min, const float& x_max, const float& y_min, float& y_max) {
        std::vector<RRTStarNode*> res;
        if (x_min > x_max || y_min > y_max) {
            std::cout << "Invalid range for range search.\n";
            return res;
        }
        RangeSearchWithRoot(kd_tree_root_, nullptr, res, x_min, x_max, y_min, y_max, 0);
        return res;
    }

    void deleteTree(RRTStarNode* root) {
        if (root == nullptr)
            return;
        
        deleteTree(root->left);
        deleteTree(root->right);
        delete root;
    }
};

kdTree::~kdTree() {
    kdTree::deleteTree(kd_tree_root_);
} 

kdTree::kdTree(const kdTree& copied_tree) {
    kd_tree_root_ = copied_tree.kd_tree_root_;
}

kdTree& kdTree::operator=(const kdTree& rhs) {
    kd_tree_root_ = rhs.kd_tree_root_;
}
#endif