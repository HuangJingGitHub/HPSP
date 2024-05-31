#include <chrono>
#include <ctime>
#include <fstream>
#include "path_processing.hpp"
#include <opencv2/viz/types.hpp>

using namespace std::chrono;

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
    int obs_num = 30;
    float obs_side_len = 30;
    int test_num = 1;
    vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), obs_side_len);
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

    vector<float> time_pivot_path_planning_vec(test_num, 0),
                  time_transfer_method_vec(test_num, 0),
                  time_separately_planning_vec(test_num, 0);

    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 51);
    // vector<Point2f> init_pts{start, Point(1, 25), Point2f(1, 50)}, target_pts{end, end + Point2f(0, 25), end + Point2f(0, 50)};
    int pt_num = 3;
    int pivot_idx = pt_num / 2;
    vector<Point2f> init_pts(pt_num), target_pts(pt_num);
    float team_width = 50, dist_step = team_width / (pt_num - 1);
    for (int i = 0; i < pt_num; i++) {
        init_pts[i] = start + Point2f(0, i * dist_step);
     
        target_pts[i] = end + Point2f(0, i * dist_step);
    }
    
    for (int test_idx = 0; test_idx < test_num; test_idx++) {
        cout << "********** Test idx: " << test_idx << "**********\n";
        // obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), obs_side_len);

        RRTStarPlanner planner_weight_cost(init_pts[pivot_idx], target_pts[pivot_idx], obs_vec, 15, 20, 10, back_img.size(), 0, 10);
        auto start_time = high_resolution_clock::now();
        bool path_planned = planner_weight_cost.Plan(back_img);
        auto end_time = high_resolution_clock::now();
        auto duration_time = duration_cast<milliseconds>(end_time - start_time);
        float time_pivot_path_planning = duration_time.count();
        
        vector<RRTStarNode*> planned_path_node;
        vector<Point2f> planned_path_pts;
        float time_transfer_method;
        if (path_planned) {
            planned_path_node = planner_weight_cost.GetPath();
            planned_path_pts = planner_weight_cost.GetPathInPts();
            // auto smooth_path = QuadraticBSplineSmoothing(planned_path_node);
            vector<int> passage_indices = RetrievePassedPassages(planned_path_node, planner_weight_cost.extended_visibility_passage_pts_).first;

            auto reposition_path = RepositionPivotPath(planned_path_pts, init_pts, target_pts, pivot_idx, planner_weight_cost.extended_visibility_passage_pts_, planner_weight_cost.obstacles_, back_img);  
            auto path_set = GeneratePathSetUpdated(planned_path_pts, init_pts, target_pts, pivot_idx, planner_weight_cost.extended_visibility_passage_pts_, planner_weight_cost.obstacles_, back_img_2);
            
            end_time = high_resolution_clock::now();
            duration_time = duration_cast<milliseconds>(end_time - start_time);
            time_transfer_method = duration_time.count();

            for (auto& cur_path : path_set)
                DrawPath(back_img, cur_path, cv::viz::Color::blue());     
            DrawPath(back_img, reposition_path, cv::viz::Color::red());
        }
        
        float time_separately_planning;
        vector<vector<Point2f>> separately_planned_path_set(init_pts.size());
        if (planned_path_pts.size() > 0) {
            cout << "\n\n\n";\
            start_time = high_resolution_clock::now();
            for (int i = 0; i < init_pts.size(); i++) {
                if (i == pivot_idx) {
                    separately_planned_path_set[i] = planned_path_pts;
                    continue;
                }
                RRTStarPlanner cur_planner(init_pts[i], target_pts[i], obs_vec, 15, 20, 10, back_img_2.size(), 0, 10);
                bool cur_path_planned = cur_planner.PlanHomotopicPath(back_img_2, planned_path_pts, init_pts[i], target_pts[i]);
                if (cur_path_planned == true)
                    separately_planned_path_set[i] = cur_planner.GetPathInPts();   
            }
            end_time = high_resolution_clock::now();
            duration_time = duration_cast<milliseconds>(end_time - start_time);
        }
        time_separately_planning = duration_time.count() + time_pivot_path_planning;
        for (auto& path : separately_planned_path_set)
            DrawPath(back_img_2, path, cv::viz::Color::blue());
        DrawPath(back_img_2, planned_path_pts, cv::viz::Color::red());

        time_pivot_path_planning_vec[test_idx] = time_pivot_path_planning;
        time_transfer_method_vec[test_idx] = time_transfer_method;
        time_separately_planning_vec[test_idx] = time_separately_planning;

        cout << "Pivot path planning time: " << time_pivot_path_planning << " ms\n";
        cout << "Path set generation time using path transfer method --- separately planning method: " << time_transfer_method << "---" << time_separately_planning << " ms\n";
    }
    imshow("Separately Planning Method", back_img_2);
    imshow("Path Transfer Method", back_img);
    waitKey(0); 
    return 0;

    ofstream  data_save_os;
    string save_directory = "./src/path_set_planning/src/data/";
    string file_name_postfix, file_name, test_info;
    std::time_t cur_time = std::time(0);
    std::tm* cur_tm = std::localtime(&cur_time);
    file_name_postfix = to_string(cur_tm->tm_year + 1900) + "-"
                        + to_string(cur_tm->tm_mon + 1) + "-"
                        + to_string(cur_tm->tm_mday) + "_"
                        + to_string(cur_tm->tm_hour) + "-"
                        + to_string(cur_tm->tm_min) + ".txt";
    std::cout << file_name_postfix << '\n';
    file_name = "plan_set_planning_time_log_" + file_name_postfix;
    test_info = "Environment dimension: " + to_string(back_img.size().width) + " x " + to_string(back_img.size().height) + "\n" 
                + "Maximum valid sample number: " + to_string(10000) + "\n" 
                + "Obstacle number: " + to_string(obs_num) + "\n"
                + "Obstacle main dimension: " + to_string(obs_side_len) + "\n"
                + "Test number: " + to_string(test_num) + "\n"
                + "Path number: " + to_string(init_pts.size()) + "\n";
    std::cout << test_info << "\n";

    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }
    else {
        data_save_os << test_info;
        data_save_os << "Planning time using path transfer method (ms): \n";
        for (float time : time_transfer_method_vec)
            data_save_os << time << "\n";
        data_save_os << "Planning time using path separately planning method (ms): \n";
        for (float time : time_separately_planning_vec)
            data_save_os << time << "\n";            
        data_save_os << "Single pivot path planning time using passage-aware optimal path planning (ms): \n";
        for (float time : time_pivot_path_planning_vec)
            data_save_os << time << "\n";           
    }
    return 0;
}
