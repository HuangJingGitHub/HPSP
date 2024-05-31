#include "path_processing.hpp"
#include <opencv2/viz/types.hpp>

void DrawPath(Mat img, const vector<Point2f>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, path[i] + shift, path[i + 1] + shift, color, thickness);
}

void DrawPath(Mat img, const vector<RRTStarNode*>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, path[i]->pos + shift, path[i + 1]->pos + shift, color, thickness);
}


int main(int argc, char** argv) {
    Mat back_img(Size(500, 300), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 20;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size());
    for (int i = 4; i < obs_num + 4; i++) {
        PolygonObstacle cur_obs = obs_vec[i];
        int cur_vertex_num = cur_obs.vertices.size();
        for (int j = 0; j < cur_vertex_num; j++)
            line(back_img, cur_obs.vertices[j], cur_obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
    }
    auto visibility_check_res = PureVisibilityPassageCheck(obs_vec);
    auto extended_visibility_check_res = ExtendedVisibilityPassageCheck(obs_vec);
    // for (int i = 0; i < visibility_check_res.first.size(); i++) 
    //     line(back_img, visibility_check_res.second[i][0], visibility_check_res.second[i][1], Scalar(0, 255, 0), 2);
    // for (int i = 0; i < extended_visibility_check_res.first.size(); i++)
    //     line(back_img, extended_visibility_check_res.second[i][0], extended_visibility_check_res.second[i][1], Scalar(0, 0, 0), 2);
    cout << "Visibility check passage res: " << visibility_check_res.first.size() 
         << "\nExtended visibility check passage res: " << extended_visibility_check_res.first.size() << '\n';
    
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    RRTStarPlanner planner_ratio_cost(start, end, obs_vec, 15, 20, 10, back_img.size(), 1),
                   planner_weight_cost_1(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 1),
                   planner_weight_cost_2(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 10),
                   planner_weight_cost_3(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 100);
    bool planned_ratio_cost = planner_ratio_cost.Plan(back_img),
         planned_weight_cost_1 = planner_weight_cost_1.Plan(back_img),
         planned_weight_cost_2 = planner_weight_cost_2.Plan(back_img),
         planned_weight_cost_3 = planner_weight_cost_3.Plan(back_img);
    vector<RRTStarNode*> planned_path_ratio_cost, planned_path_weight_1, planned_path_weight_2, planned_path_weight_3;
    if (planned_ratio_cost && planned_weight_cost_1 && planned_weight_cost_2 && planned_weight_cost_3) {
        planned_path_ratio_cost = planner_ratio_cost.GetPath();
        planned_path_weight_1 = planner_weight_cost_1.GetPath();
        planned_path_weight_2 = planner_weight_cost_2.GetPath();
        planned_path_weight_3 = planner_weight_cost_3.GetPath();
        auto smooth_path_ratio_cost = QuadraticBSplineSmoothing(planned_path_ratio_cost),
             smooth_path_weight_1 = QuadraticBSplineSmoothing(planned_path_weight_1),
             smooth_path_weight_2 = QuadraticBSplineSmoothing(planned_path_weight_2),
             smooth_path_weight_3 = QuadraticBSplineSmoothing(planned_path_weight_3);
        // cout << "path node num: " << planned_path_ratio_cost.size() << " smooth node num: " << smooth_path_ratio_cost.size() << "\n";
        vector<int> passage_indices = RetrievePassedPassages(planned_path_ratio_cost, planner_ratio_cost.extended_visibility_passage_pts_).first;
        for (int passage_idx : passage_indices)
            cout << planner_ratio_cost.extended_visibility_passage_pair_[passage_idx][0] << "---" << planner_ratio_cost.extended_visibility_passage_pair_[passage_idx][1] << '\n';

        DrawPath(back_img, smooth_path_ratio_cost, cv::viz::Color::blue());
        DrawPath(back_img, smooth_path_weight_1, cv::viz::Color::green());
        DrawPath(back_img, smooth_path_weight_2, cv::viz::Color::cyan());
        DrawPath(back_img, smooth_path_weight_3, cv::viz::Color::red());
    }

    imshow("RRT* path planning", back_img);
    waitKey(0); 
    return 0;    
}