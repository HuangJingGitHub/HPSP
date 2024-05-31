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

void DrawDshedLine(Mat img, const Point2f& initial_pt, const Point2f& end_pt, Scalar color = Scalar(0, 0, 0), int thickness = 2, float dash_len = 5) {
    float line_len = cv::norm(end_pt - initial_pt);
    Point2f line_direction = (end_pt -initial_pt) / line_len;
    int dash_num = line_len / dash_len;
    if (line_len < 2 * dash_len) 
        line(img, initial_pt, initial_pt + dash_len * line_direction, color, thickness);
    for (int i = 0; i <= dash_num; i += 2) 
        if (i == dash_num)
            line(img, initial_pt + i * dash_len * line_direction, end_pt, color, thickness);
        else
            line(img, initial_pt + i * dash_len * line_direction, initial_pt + (i + 1) * dash_len * line_direction, color, thickness);
}

int main(int argc, char** argv) {
    Mat back_img(Size(500, 300), CV_64FC3, Scalar(255, 255, 255)), back_img_2;
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
    for (int i = 0; i < visibility_check_res.first.size(); i++) 
        DrawDshedLine(back_img, visibility_check_res.second[i][0], visibility_check_res.second[i][1], Scalar(0.741, 0.447, 0), 1);
    for (int i = 0; i < extended_visibility_check_res.first.size(); i++)
        DrawDshedLine(back_img, extended_visibility_check_res.second[i][0], extended_visibility_check_res.second[i][1], Scalar(0, 0, 0), 2);
    cout << "Visibility check passage res: " << visibility_check_res.first.size() 
         << "\nExtended visibility check passage res: " << extended_visibility_check_res.first.size() << '\n';
    back_img_2 = back_img.clone();
    
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 51);
    vector<Point2f> init_pts{start, Point(1, 25), Point2f(1, 50)}, target_pts{end, end + Point2f(0, 25), end + Point2f(0, 50)};
    int pivot_idx = 1;
    RRTStarPlanner planner_weight_cost(init_pts[pivot_idx], target_pts[pivot_idx], obs_vec, 15, 20, 10, back_img.size(), 0, 10);
    bool path_planned = planner_weight_cost.Plan(back_img);
    vector<RRTStarNode*> planned_path_node;
    vector<Point2f> planned_path_pts;
    if (path_planned) {
        planned_path_node = planner_weight_cost.GetPath();
        planned_path_pts = planner_weight_cost.GetPathInPts();
        auto smooth_path = QuadraticBSplineSmoothing(planned_path_node);
        vector<int> passage_indices = RetrievePassedPassages(planned_path_node, planner_weight_cost.extended_visibility_passage_pts_).first;
        for (int passage_idx : passage_indices) {
            cout << planner_weight_cost.extended_visibility_passage_pair_[passage_idx][0] << "---" 
                << planner_weight_cost.extended_visibility_passage_pair_[passage_idx][1] << '\n';
            // auto intersection_pt = GetPathSetIntersectionsOnPassageLine(planned_path_pts, {start}, {end}, 0,  
            //                                                             planner_weight_cost.extended_visibility_passage_pts_[passage_idx]).first;
            // circle(back_img, intersection_pt[0], 4, Scalar(0, 0, 255), -1);
        }

        auto reposition_path = RepositionPivotPath(planned_path_pts, init_pts, target_pts, pivot_idx, planner_weight_cost.extended_visibility_passage_pts_, planner_weight_cost.obstacles_, back_img);
        DrawPath(back_img, reposition_path, cv::viz::Color::green());
        auto direct_path_set = GetTransferPathSet(planned_path_pts, init_pts, target_pts, pivot_idx);
        for (auto& cur_path : direct_path_set)
            DrawPath(back_img, cur_path, cv::viz::Color::blue());  

        auto add_general_passage_res = AddGeneralPassagesSingleSide(planner_weight_cost.extended_visibility_passage_pair_, planner_weight_cost.extended_visibility_passage_pts_, 
                                                                    planner_weight_cost.obstacles_, planned_path_pts);
        cout << "Passage number before vs. after: " << planner_weight_cost.extended_visibility_passage_pts_.size() 
             << " vs. " << add_general_passage_res.second.size() << "\n";
        
        //auto path_set = GeneratePathSetUpdated(planned_path_pts, init_pts, target_pts, pivot_idx, planner_weight_cost.extended_visibility_passage_pts_, planner_weight_cost.obstacles_, back_img_2);
        auto path_set = GeneratePathSetUpdated(planned_path_pts, init_pts, target_pts, pivot_idx, add_general_passage_res.second, planner_weight_cost.obstacles_, back_img_2);
        auto direct_path_set_reposition = GetTransferPathSet(reposition_path, init_pts, target_pts, pivot_idx);
        // for (auto cur_path : direct_path_set_reposition)
        //     DrawPath(back_img_2, cur_path, cv::viz::Color::blue());
        for (auto& cur_path : path_set)
            DrawPath(back_img_2, cur_path, cv::viz::Color::green());     
        for (int i = planner_weight_cost.extended_visibility_passage_pts_.size() + 1; i < add_general_passage_res.second.size(); i++) {
            line(back_img_2, add_general_passage_res.second[i][0], add_general_passage_res.second[i][1], Scalar(255, 0, 0), 2);                 
            cout << add_general_passage_res.second[i][0] << "---" << add_general_passage_res.second[i][1] << "\n";
        }
    }

    imshow("Repositioning path", back_img);
    imshow("Path set generation", back_img_2);
    waitKey(0); 
    return 0;
}