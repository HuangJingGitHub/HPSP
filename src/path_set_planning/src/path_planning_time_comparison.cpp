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

int main(int argc, char** argv) {
    Mat back_img(Size(500, 300), CV_64FC3, Scalar(255, 255, 255));
    int obs_num = 10;
    float obs_side_len = 30;
    Point2f start = Point2f(1, 1), end = Point2f(back_img.size().width - 1, back_img.size().height - 1);
    
    int test_num = 10;
    vector<float> planning_time_extended_vis(test_num, 0), planning_time_pure_vis(test_num, 0);

/*     vector<RRTStarPlanner> planner_vec(test_num);
    for (int i = 0; i < test_num; i++)
        planner_vec[i] = RRTStarPlanner(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 10); */

    for (int planning_test_idx = 0; planning_test_idx < test_num; planning_test_idx++) {
        vector<PolygonObstacle> obs_vec = GenerateRandomObstacles(obs_num, back_img.size(), obs_side_len);
        for (int i = 4; i < obs_num + 4; i++) {
            PolygonObstacle cur_obs = obs_vec[i];
            int cur_vertex_num = cur_obs.vertices.size();
            for (int j = 0; j < cur_vertex_num; j++)
                line(back_img, cur_obs.vertices[j], cur_obs.vertices[(j + 1) % cur_vertex_num], Scalar(0, 0, 0), 2);
        }

        RRTStarPlanner planner_extended_vis(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 10);
        RRTStarPlanner planner_pure_vis(start, end, obs_vec, 15, 20, 10, back_img.size(), 0, 10, false);
        
        auto start_time = high_resolution_clock::now();
        bool plan_success_extended = planner_extended_vis.Plan(back_img);      
        auto end_time = high_resolution_clock::now();
        auto duration_time_extended = duration_cast<milliseconds>(end_time - start_time);
        
        start_time = high_resolution_clock::now();
        bool plan_success_pure = planner_pure_vis.Plan(back_img);
        end_time = high_resolution_clock::now();
        auto duration_time_pure = duration_cast<milliseconds>(end_time - start_time);
        
        if (plan_success_extended == false || plan_success_pure == false) {
            planning_test_idx--;
            continue;
        }

/*         vector<Point2f> path_extended = planner_extended_vis.GetPathInPts(),
                        path_pure = planner_pure_vis.GetPathInPts();
        vector<int> passed_passages_extended = RetrievePassedPassages(path_extended, planner_extended_vis.pure_visibility_passage_pts_),
                    passed_passages_pure = RetrievePassedPassages(path_pure, planner_pure_vis.pure_visibility_passage_pts_);
        if (passed_passages_extended.size() != passed_passages_pure.size())
            cout << "Two found paths are not homotopic.\n";
        for (int i = 0; i < passed_passages_extended.size(); i++) {
            cout << "passed passage idx: " << passed_passages_extended[i] << " vs " << passed_passages_pure[i] << "\n";
        }

        DrawPath(back_img, path_extended, Scalar(0, 255, 0));
        DrawPath(back_img, path_pure, Scalar(255, 0, 0));
        imshow("RRT* path planning", back_img);
        if (waitKey(20000) == 27)
            break;      */    

        float cur_planning_time_extended = duration_time_extended.count(),
            cur_planning_time_pure = duration_time_pure.count();
        planning_time_extended_vis[planning_test_idx] = cur_planning_time_extended;
        planning_time_pure_vis[planning_test_idx] = cur_planning_time_pure;

        cout << "Planning time with extended---pure visibility check: " << cur_planning_time_extended << "---" << cur_planning_time_pure << " ms\n\n";
    }
    return 1;

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
    cout << file_name_postfix << '\n';
    file_name = "planning_time_log_" + file_name_postfix;
    test_info = "Environment dimension: " + to_string(back_img.size().width) + " x " + to_string(back_img.size().height) + "\n" 
                + "Maximum sample number: " + to_string(10000) + "\n" 
                + "Obstacle num: " + to_string(obs_num) + "\n"
                + "Obstacle main dimension: " + to_string(obs_side_len) + "\n"
                + "Planning test number: " + to_string(test_num) + "\n" 
                + "(Descendent cost update is added)" + "\n";
    cout << test_info << "\n";

    data_save_os.open(save_directory + file_name, std::ios::trunc);
    if (data_save_os.is_open() == false) {
        std::cerr << "Fail to open the data storage file!\n";
    }
    else {
        data_save_os << test_info;
        data_save_os << "Planning time using extended visibility check (ms): \n";
        for (float time : planning_time_extended_vis)
            data_save_os << time << "\n";
    data_save_os << "Corresponding planning time using pure visibility check (ms): \n";        
        for (float time : planning_time_pure_vis)
            data_save_os << time << "\n";
    }    
    return 0;    
}