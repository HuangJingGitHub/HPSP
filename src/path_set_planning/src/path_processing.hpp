#ifndef PATH_SMOOTHING_INCLUDED
#define PATH_SMOOTHING_INCLUDED

#include <eigen3/Eigen/Dense>
#include "RRTStar_DOM_kd_tree.hpp"

using namespace Eigen;

vector<Point2f> QuadraticBSplineSmoothing(const vector<RRTStarNode*>& node_path) {
    vector<Point2f> res;
    if (node_path.size() < 3) {
        cout << "No sufficient control points on the input path!\n"
             << "(At least 3 control points are needed for quadratic B-Spline smoothing.)\n";
        return res;
    }

    const float step_width = 0.05;   // For MatrixxXf.block() mathod needs constexpr as argument.
    const int step_num = 1 / step_width + 1,
              pts_num = node_path.size();

    Matrix3f coefficient_mat_1, coefficient_mat_2, coefficient_mat_3;
    MatrixXf parameter_var(3, step_num), 
             control_pts(2, pts_num);
    Matrix<float, 2, Dynamic> smoothed_path;

    coefficient_mat_1 << 1, -2, 1, -1.5, 2, 0, 0.5, 0, 0;
    coefficient_mat_2 << 0.5, -1, 0.5, -1, 1, 0.5, 0.5, 0, 0;
    coefficient_mat_3 << 0.5, -1, 0.5, -1.5, 1, 0.5, 1, 0, 0;
    
    for (int i = 0; i < step_num; i++) {
        float cur_var = step_width * i;
        parameter_var(0, i) = cur_var * cur_var;
        parameter_var(1, i) = cur_var;
        parameter_var(2, i) = 1;
    }
    for (int i = 0; i < pts_num; i++) {
        control_pts(0, i) = node_path[i]->pos.x;
        control_pts(1, i) = node_path[i]->pos.y;
    }

    MatrixXf cur_pts = control_pts.block<2, 3>(0, 0);
    MatrixXf cur_spline = cur_pts * coefficient_mat_1 * parameter_var;
    smoothed_path = cur_spline;

    for (int i = 1; i < pts_num - 3; i++) {
        cur_pts = control_pts.block<2, 3>(0, i);
        cur_spline = cur_pts * coefficient_mat_2 * parameter_var;
        smoothed_path.conservativeResize(2, smoothed_path.cols() + cur_spline.cols());
        smoothed_path.block<2, step_num>(0, smoothed_path.cols() - step_num) = cur_spline;
    }
    cur_pts = control_pts.block<2, 3>(0, pts_num - 3);
    cur_spline = cur_pts * coefficient_mat_3 * parameter_var;
    smoothed_path.conservativeResize(2, smoothed_path.cols() + cur_spline.cols());
    smoothed_path.block<2, step_num>(0, smoothed_path.cols() - step_num) = cur_spline;

/*     cout << "path smoothing:\n"
         << node_path.size() << ' '
         << smoothed_path.rows() << " x " << smoothed_path.cols() << '\n'
         << cur_spline.rows() << " x " << cur_spline.cols() << endl; */
    for (int i = 0; i < smoothed_path.cols(); i++)
        res.push_back(Point2f(smoothed_path(0, i), smoothed_path(1, i)));
    return res;
}


int SearchByDistance(vector<Point2f>& search_path, Point2f desired_pos) {
    int res_idx = 0;
    if (search_path.empty()) {
        cout << "An empty path is given." << endl;
        return res_idx;
    }

    Point2f trivial_sol = search_path.back();
    float ref_distance = norm(desired_pos - trivial_sol),
          cur_distance_dif;

    for (int i = search_path.size() - 1; i >= 0; i--) {
        cur_distance_dif = abs(norm(desired_pos - search_path[i]) - ref_distance);
        if (cur_distance_dif < 2)
            res_idx = i;
    }
    return res_idx;
}


vector<vector<Point2f>> GeneratePathSet(const vector<Point2f>& initial_feedback_pts, 
                                        const vector<Point2f>& target_feedback_pts, 
                                        int pivot_idx,
                                        float feedback_pts_radius,
                                        const vector<PolygonObstacle>& obs,
                                        Mat source_img) {
    vector<vector<Point2f>> res_path_set(initial_feedback_pts.size());                                            
    RRTStarPlanner planner(initial_feedback_pts[pivot_idx], target_feedback_pts[pivot_idx], obs);
    bool plan_success = planner.Plan(source_img, feedback_pts_radius, true);
    if (!plan_success) {
        cout << "Path planning failed! An empty path set is reutrned.\n";
        return {};
    }

    vector<RRTStarNode*> pivot_path = planner.GetPath();
    vector<RRTStarNode*> sparse_pivot_path;
    for (int i = 0; i < pivot_path.size(); i += 1)
        sparse_pivot_path.push_back(pivot_path[i]);
    if (sparse_pivot_path.back() != pivot_path.back())
        sparse_pivot_path.push_back(pivot_path.back());
    vector<Point2f> smooth_pivot_path = QuadraticBSplineSmoothing(sparse_pivot_path);
    res_path_set[pivot_idx] = smooth_pivot_path;
    
    for (int i = 0; i < initial_feedback_pts.size(); i++) {
        if (i != pivot_idx) {
            Point2f cur_dif = initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];
            res_path_set[i] = smooth_pivot_path;
            for (Point2f& pt : res_path_set[i])
                pt += cur_dif;
            
            if (normSqr(res_path_set[i].back() - target_feedback_pts[i]) > normSqr(res_path_set[i].front() - target_feedback_pts[i])){
                res_path_set[i].erase(res_path_set[i].begin() + 1, res_path_set[i].end());
            }
            else {
                int truncation_idx = SearchByDistance(res_path_set[i], target_feedback_pts[i]);
                res_path_set[i].erase(res_path_set[i].begin() + truncation_idx, res_path_set[i].end());
            }
            
            Point2f temp_end = res_path_set[i].back(), segment = target_feedback_pts[i] - temp_end;
            float segment_length = cv::norm(segment), temp_path_length = cv::norm(temp_end - res_path_set[i].front()), 
                  interpolation_ratio;
            int segment_pt_num;
            if (temp_path_length > 1)  // in case temp_path_length becomes 0
                segment_pt_num = segment_length / temp_path_length * res_path_set[i].size();
            else
                segment_pt_num = 100;
            for (int interpolation_idx = 1; interpolation_idx <= segment_pt_num; interpolation_idx++) {
                interpolation_ratio = (float)interpolation_idx / segment_pt_num;
                res_path_set[i].push_back(temp_end + segment * interpolation_ratio);
            }
            // res_path_set[i].push_back(target_feedback_pts[i]);
        }
    }
    return res_path_set; 
}


vector<Point2f> ProcessCollisionPath(const vector<Point2f>& pivot_path, 
                                    Point2f pivot_pt, Point2f cur_pt, 
                                    Point2f cur_target_pt,
                                    const PolygonObstacle& original_obstacle, 
                                    float safety_dis = 20) {
    vector<Point2f> inflated_obs_vertices = original_obstacle.vertices;
    Point2f vertices_centroid = Point2f(0, 0);
    for (Point2f& pt : inflated_obs_vertices)
        vertices_centroid += pt;
    vertices_centroid /= (float) inflated_obs_vertices.size();
    for (Point2f& pt : inflated_obs_vertices)
        pt = vertices_centroid + 1.1 * (pt - vertices_centroid);
    PolygonObstacle obstacle(inflated_obs_vertices);


    vector<Point2f> res_path(pivot_path.size());
    Point2f cur_dif = cur_pt - pivot_pt;
    float cur_dif_norm = cv::norm(cur_dif);
    vector<int> collision_indices;
    
    for (int i = 0; i < pivot_path.size(); i++) {
        Point2f ref_pt = pivot_path[i] + cur_dif;
        if (!ObstacleFree(obstacle, pivot_path[i], ref_pt))
            collision_indices.push_back(i);
    }
    
    // std::sort(collision_indices.begin(), collision_indices.end());
    int collision_start_idx = collision_indices.front(), 
        collision_end_idx = collision_indices.back(),
        reconnection_start_idx = collision_start_idx, reconnection_end_idx = collision_end_idx;
    float dis_ref = 3 * safety_dis;
    for (int i = collision_start_idx - 1; i >= 0; i--) {
        float dis_to_obs = MinDistanceToObstacle(obstacle, pivot_path[i] + cur_dif);
        if (dis_to_obs >= dis_ref) {
            reconnection_start_idx = i;
            break;
        }
    }
    if (reconnection_start_idx == collision_start_idx)
        reconnection_start_idx = 0;
    
    Point2f intersection_start_pt = GetClosestIntersectionPt(obstacle, pivot_path[collision_start_idx],
                            pivot_path[collision_start_idx] + cur_dif, pivot_path[collision_start_idx]),
            shift_pt_1 = intersection_start_pt - safety_dis / cur_dif_norm * cur_dif 
                         - (pivot_path[reconnection_start_idx] + cur_dif); 
    for (int i = collision_end_idx + 1; i < pivot_path.size(); i++) {
        float dis_to_obs = MinDistanceToObstacle(obstacle, pivot_path[i] + cur_dif);
        if (dis_to_obs >= dis_ref) {
            reconnection_end_idx = i;
            break;
        }
    }
    if (reconnection_end_idx == collision_end_idx)
        reconnection_end_idx = pivot_path.size() - 1;
    Point2f intersection_end_pt = GetClosestIntersectionPt(obstacle, pivot_path[collision_end_idx],
                            pivot_path[collision_end_idx] + cur_dif, pivot_path[collision_end_idx]),
            shift_pt_2 = pivot_path[reconnection_end_idx] + cur_dif - 
                        (intersection_end_pt - safety_dis / cur_dif_norm * cur_dif); 
                        
    for (int i = 0; i < reconnection_start_idx; i++)
        res_path[i] = pivot_path[i] + cur_dif;
    int step_num = collision_start_idx - reconnection_start_idx;
    for (int i = reconnection_start_idx; i < collision_start_idx; i++) {
        float shift_ratio = (float)(i - reconnection_start_idx) / (float)step_num;
        res_path[i] = pivot_path[reconnection_start_idx] + cur_dif + shift_ratio * shift_pt_1;
    }
    for (int i = collision_start_idx; i < collision_end_idx; i++) {
        Point2f intersection_pt = GetClosestIntersectionPt(obstacle, pivot_path[i], pivot_path[i] + cur_dif, pivot_path[i]);
        res_path[i] = intersection_pt - safety_dis / cur_dif_norm * cur_dif;
    }
    step_num = reconnection_end_idx - collision_end_idx;
    for (int i = collision_end_idx; i < reconnection_end_idx; i++) {
        float shift_ratio = (float) (i - collision_end_idx) / step_num;
        res_path[i] = intersection_end_pt - safety_dis / cur_dif_norm * cur_dif + shift_ratio * shift_pt_2;
    }
    for (int i = reconnection_end_idx; i < pivot_path.size(); i++)
        res_path[i] = pivot_path[i] + cur_dif;

    // postprocessing
    int truncation_idx = SearchByDistance(res_path, cur_target_pt);
    res_path.erase(res_path.begin() + truncation_idx, res_path.end());
    Point2f temp_end = res_path.back(), segment = cur_target_pt - temp_end;
    float segment_length = cv::norm(segment), temp_path_length = cv::norm(temp_end - res_path.front()), 
            interpolation_ratio;
    int segment_pt_num;
    if (temp_path_length > 1)  // in case temp_path_length becomes 0
        segment_pt_num = segment_length / temp_path_length * res_path.size();
    else
        segment_pt_num = 100;
    for (int interpolation_idx = 1; interpolation_idx <= segment_pt_num; interpolation_idx++) {
        interpolation_ratio = (float)interpolation_idx / segment_pt_num;
        res_path.push_back(temp_end + segment * interpolation_ratio);
    }
    // std::cout << "OK5\n";
    return res_path; 
}

vector<float>  GetLocalPathWidth2D( const vector<Point2f>& path, 
                                    vector<PolygonObstacle>& obstacles,
                                    Size2f config_size = Size2f(640, 480)) {
    vector<float> local_path_width(path.size(), 0);
    vector<int> path_width_type(path.size(), 0);  // 0: two-side free, 1: one-side free, one-side obstacle
                                                  // 2: two-side obstacle
    float slope, x_max = config_size.width, y_max = config_size.height,
          cur_x, cur_y,
          x_intercept, y_intercept,
          x_at_y_max, y_at_x_max;

    for (int i = 1; i < path.size() - 1; i++) {
        cur_x = path[i].x;
        cur_y = path[i].y;
        slope = -(path[i + 1].x - path[i - 1].x) / (path[i + 1].y - path[i - 1].y);
        vector<Point2f> boundary_insection_pts;

        y_intercept = cur_y + slope * (0 - cur_x);
        if (0 <= y_intercept && y_intercept <= y_max)
            boundary_insection_pts.push_back(Point2f(0, y_intercept));
        x_intercept = cur_x - cur_y / slope;
        if (0 <= x_intercept && x_intercept <= x_max)
            boundary_insection_pts.push_back(Point2f(x_intercept, 0));
        x_at_y_max = cur_x + (y_max - cur_y) / slope;
        if (0 <= x_at_y_max && x_at_y_max <= x_max)
            boundary_insection_pts.push_back(Point2f(x_at_y_max, y_max));
        y_at_x_max = cur_x + slope * (x_max - cur_x);
        if (0 <= y_at_x_max && y_at_x_max <= y_max)
            boundary_insection_pts.push_back(Point2f(x_max, y_at_x_max));
        
        // std::cout << "OK_1\n";
        if (boundary_insection_pts.empty()) {
            std::cout << "Error!\n";
            std::cout << path[i] << '\n' << "slope: " << slope << '\n';
        }
        Point2f end_pt1 = boundary_insection_pts[0], end_pt2 = boundary_insection_pts[1],
                direction1 = end_pt1 - path[i], direiction2 = end_pt2 - path[i];
        vector<Point2f> obs_intersection_pts, direction1_pts, direction2_pts;
        for (PolygonObstacle& cur_obs : obstacles) {
            if (ObstacleFree(cur_obs, end_pt1, end_pt2))
                continue;
            Point2f cur_obs_intersection_pt = GetClosestIntersectionPt(cur_obs, end_pt1, end_pt2, path[i]);
            obs_intersection_pts.push_back(cur_obs_intersection_pt);
        }
        
        if (obs_intersection_pts.empty())
            local_path_width[i] = cv::norm(end_pt1 - end_pt2);
        else {
            for (Point2f cur_pt : obs_intersection_pts) {
                Point2f cur_vec = cur_pt - path[i];
                if (cur_vec.x * direction1.x + cur_vec.y * direction1.y > 0)
                    direction1_pts.push_back(cur_pt);
                else
                    direction2_pts.push_back(cur_pt);
            }
            if (!direction1_pts.empty()) {
                path_width_type[i]++;
                for (Point2f& cur_direction1_pt : direction1_pts) {
                    if (normSqr(cur_direction1_pt - path[i]) < normSqr(end_pt1 - path[i]))
                        end_pt1 = cur_direction1_pt;
                }
            }
            if (!direction2_pts.empty()) {
                path_width_type[i]++;
                for (Point2f& cur_direction2_pt : direction2_pts)
                    if (normSqr(cur_direction2_pt - path[i]) < normSqr(end_pt2 - path[i]))
                        end_pt2 = cur_direction2_pt;
            }
            local_path_width[i] = cv::norm(end_pt1 - end_pt2);
        }
    }

    if (local_path_width.size() >= 2) {
        local_path_width.front() = local_path_width[1];
        local_path_width.back() = local_path_width[local_path_width.size() - 2];
    }
    return local_path_width;
}

vector<vector<int>> FilterRawPassagePairsByVisibility(const vector<PolygonObstacle>& obstacles, 
                                                    const vector<vector<int>>& raw_passage_pairs) {
    vector<vector<int>> res;
    vector<set<int>> valid_passage_pair_set(obstacles.size());
    for (const vector<int>& cur_passage_pair : raw_passage_pairs)
        valid_passage_pair_set[cur_passage_pair[0]].insert(cur_passage_pair[1]);

    for (int i = 0; i < valid_passage_pair_set.size(); i++) {
        for (auto it = valid_passage_pair_set[i].begin(); it != valid_passage_pair_set[i].end();) {
            if (ObstacleFreeVecForPassage(obstacles, i, *it) == false)
                it = valid_passage_pair_set[i].erase(it);  // Be careful when dynamically deleting a set element
            else
                it++;
        }
    }

    for (int i = 0; i < raw_passage_pairs.size(); i++) {
        if (valid_passage_pair_set[raw_passage_pairs[i][0]].find(raw_passage_pairs[i][1]) 
            != valid_passage_pair_set[raw_passage_pairs[i][0]].end())
            res.push_back(raw_passage_pairs[i]);
    }
    return res;
}

vector<vector<int>> FilterPassagePairsBySortedWidth(const vector<PolygonObstacle>& obstacles,
                                                    const vector<vector<int>>& raw_passage_pairs) {
    vector<vector<int>> res;
    vector<vector<int>> passage_pairs_mutual_map(obstacles.size());
    for (const vector<int>& cur_passage_pair : raw_passage_pairs) {
        passage_pairs_mutual_map[cur_passage_pair[0]].push_back(cur_passage_pair[1]);
        passage_pairs_mutual_map[cur_passage_pair[1]].push_back(cur_passage_pair[0]);
    }

    vector<int> obs_idx_2_candidate_vec(obstacles.size(), -1);
    for (int i = 0; i < obstacles.size(); i++) {
        if (passage_pairs_mutual_map[i].size() == 0)
            continue;
        vector<pair<float, int>> width_to_obs_idx;
        PolygonObstacle obs1 = obstacles[i];
        for (int j = 0; j < passage_pairs_mutual_map[i].size(); j++) {
            int obs_idx_2 = passage_pairs_mutual_map[i][j];
            PolygonObstacle obs2 = obstacles[obs_idx_2];
            vector<Point2f> cur_passage_inner_ends = GetPassageInnerEnds(obs1, obs2);
            float cur_passage_width = cv::norm(cur_passage_inner_ends[0] - cur_passage_inner_ends[1]);
            width_to_obs_idx.push_back(make_pair(cur_passage_width, obs_idx_2));
        }
        std::sort(width_to_obs_idx.begin(), width_to_obs_idx.end());
        obs_idx_2_candidate_vec[i] = width_to_obs_idx[0].second;
    }

    for (int i = 0; i < obstacles.size(); i++) {
        if (passage_pairs_mutual_map[i].size() == 0)
            continue;
        int obs_idx_2 = obs_idx_2_candidate_vec[i];
        if (obs_idx_2 > i && i == obs_idx_2_candidate_vec[obs_idx_2])
            res.push_back({i, obs_idx_2});
    }
    return res;    
}

vector<vector<int>> GetPassagesPathPasses(const vector<PolygonObstacle>& obstacles, 
                                        const vector<RRTStarNode*>& path) {
    // Raw, non-smoothed path is preferred for efficiency.
    vector<vector<int>> raw_passage_pairs;
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);
    
    for (int i = 0; i < path.size() - 1; i++) 
        for (int j = 0; j < obs_centroids.size() - 1; j++) 
            for (int k = j + 1; k < obs_centroids.size(); k++) 
                if (SegmentIntersection(path[i]->pos, path[i + 1]->pos, obs_centroids[j], obs_centroids[k]) == true) 
                    raw_passage_pairs.push_back({j, k});
    // Filter invalid passages
    vector<vector<int>> physical_valid_passage_pairs = FilterRawPassagePairsByVisibility(obstacles, raw_passage_pairs);
    return FilterPassagePairsBySortedWidth(obstacles, physical_valid_passage_pairs);
}

vector<vector<Point2f>> GetPassageIntersectionsOfPathSet(const vector<PolygonObstacle>& obstacles, 
                                                        const vector<vector<int>>& passage_pairs,
                                                        const vector<Point2f>& smooth_pivot_path, 
                                                        vector<vector<int>>& path_node_intersection_idx_log,
                                                        const vector<Point2f>& initial_feedback_pts, 
                                                        int pivot_idx) {
    int pts_num = initial_feedback_pts.size();
    vector<vector<Point2f>> res(pts_num, vector<Point2f>(passage_pairs.size()));
    path_node_intersection_idx_log = vector<vector<int>>(pts_num, vector<int>(passage_pairs.size(), 0));
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    for (int i = 0; i < pts_num; i++) {
        int path_node_idx = 0;
        Point2f pt_dif = initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];
        for (int j = 0; j < passage_pairs.size(); j++) {
            // cout << "Testing passage: " << passage_pairs[j][0] << "---" << passage_pairs[j][1] << "\n";
            Point2f obs_centroid_1 = obs_centroids[passage_pairs[j][0]],
                    obs_centroid_2 = obs_centroids[passage_pairs[j][1]],
                    direction_obs_1_2 = (obs_centroid_2 - obs_centroid_1) / cv::norm(obs_centroid_2 - obs_centroid_1),
                    extended_obs_centroid_1 = obs_centroid_1 - 300 * direction_obs_1_2,
                    extended_obs_centroid_2 = obs_centroid_2 + 300 * direction_obs_1_2;
            // cout << "Obstacle centroid 1: " << obs_centroid_1 << " Obstacle centroid 2: " << obs_centroid_2 << "\n";
            // cout << "Extended obstacle centroid 1: " << extended_obs_centroid_1 << " Extended obstacle centroid 2: " << extended_obs_centroid_2 << "\n";
            for (int k = path_node_idx; k < smooth_pivot_path.size() - 1; k++) {
                if (SegmentIntersection(extended_obs_centroid_1, extended_obs_centroid_2, 
                                        smooth_pivot_path[k] + pt_dif, smooth_pivot_path[k + 1] + pt_dif)) {
                    res[i][j] = GetSegmentsIntersectionPt(extended_obs_centroid_1, extended_obs_centroid_2, 
                                                        smooth_pivot_path[k] + pt_dif, smooth_pivot_path[k + 1] + pt_dif);
                    // cout << "Intersection detected with point:\n" << res[i][j] << '\n';
                    path_node_intersection_idx_log[i][j] = k;
                    // In theory, the same path segment can pass two or more passages, so do not use path_node_idx = k + 1.  
                    // Also, next intersection can be placed before the current one, simply setting path_node = 0 is ok. Here, locality search is used.
                    path_node_idx = (k - (int)smooth_pivot_path.size() / 5) < 0 ? 0 : k - (int)smooth_pivot_path.size() / 5; 
                    break;
                }
            }
        }
    }
    cout << "Intersection pts of path set:\n";
    for (auto pt_vec : res) {
        for (auto pt : pt_vec) 
            cout << pt << ", ";
        cout << "\n";
    }
    return res;
}

vector<Point2f> GetPivotPathRepositionPts(const vector<PolygonObstacle>& obstacles, 
                                        const vector<vector<int>>& passage_pairs, 
                                        const vector<vector<Point2f>>& intersection_points, 
                                        int pivot_idx) {
    vector<Point2f> chord_ends, passage_inner_ends, res(passage_pairs.size());
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    int pts_num = intersection_points.size();
    for (int i = 0; i < passage_pairs.size(); i++) {
        vector<Point2f> cur_passage_intersection_pts(pts_num);
        for (int j = 0; j < pts_num; j++)
            cur_passage_intersection_pts[j] = intersection_points[j][i];
        chord_ends = GetEndsOfColinearPts(cur_passage_intersection_pts);
        passage_inner_ends = GetPassageInnerEnds(obstacles[passage_pairs[i][0]], obstacles[passage_pairs[i][1]]);
/*         cout << "Passage inner ends:\n"
             << passage_inner_ends[0] << "-" << passage_inner_ends[1] << '\n';
        cout << "Chord ends:\n"
             << chord_ends[0] << "-" << chord_ends[1] << '\n';
        cout << "Passage intersection point:\n"
             << cur_passage_intersection_pts[pivot_idx] << '\n'; */

        Point2f reposition_intersection_pt;
        if (normSqr(chord_ends[1] - chord_ends[0]) < normSqr(passage_inner_ends[1] - passage_inner_ends[0])
            && OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[0])
            && OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[1])) {
                // cout << "case 1\n";
                res[i] = cur_passage_intersection_pts[pivot_idx];
                continue;
            }
        else if (normSqr(chord_ends[1] - chord_ends[0]) < normSqr(passage_inner_ends[1] - passage_inner_ends[0])) {
            if (OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[0])) {
                if (normSqr(passage_inner_ends[0] - chord_ends[1]) <= normSqr(passage_inner_ends[1] - chord_ends[1]))
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[0] - chord_ends[1]);
                else 
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[1] - chord_ends[1]);
            }
            else {
                if (normSqr(passage_inner_ends[0] - chord_ends[0]) <= normSqr(passage_inner_ends[1] - chord_ends[0]))
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[0] - chord_ends[0]);
                else 
                    reposition_intersection_pt = cur_passage_intersection_pts[pivot_idx] + 1.2 * (passage_inner_ends[1] - chord_ends[0]);                
            }
            // cout << "Reposition intersection point:\n" 
            //      << reposition_intersection_pt << '\n';
            res[i] = reposition_intersection_pt;
            continue;
        }
        
        Point2f chord_center = (chord_ends[0] + chord_ends[1]) / 2,
                passage_inner_center = (passage_inner_ends[0] + passage_inner_ends[1]) / 2,
                chord_to_passage_center_dif = passage_inner_center - chord_center,
                pivot_path_intersection_pt = cur_passage_intersection_pts[pivot_idx];
        reposition_intersection_pt = pivot_path_intersection_pt + chord_to_passage_center_dif;
        if (OnSegment(passage_inner_ends[0], passage_inner_ends[1], reposition_intersection_pt) == false) {
            if (normSqr(reposition_intersection_pt - passage_inner_ends[0]) <= normSqr(reposition_intersection_pt - passage_inner_ends[1]))
                reposition_intersection_pt = passage_inner_ends[0] + 0.2 * (passage_inner_ends[1] - passage_inner_ends[0]);
            else 
                reposition_intersection_pt = passage_inner_ends[1] + 0.2 * (passage_inner_ends[0] - passage_inner_ends[1]);
        }
        // cout << "Reposition intersection point:\n" 
        //         << reposition_intersection_pt << '\n';        
        res[i] = reposition_intersection_pt;
    }
    return res;
}

vector<vector<Point2f>> AdjustRepositionPtsForPathSet(const vector<PolygonObstacle>& obstacles, 
                                                    const vector<vector<int>>& passage_pairs,
                                                    const vector<vector<Point2f>>& path_set_intersection_pts,
                                                    int pivot_idx) {
    vector<vector<Point2f>> res = path_set_intersection_pts;
    int pts_num = res.size();
    vector<Point2f> chord_ends, passage_inner_ends;
    for (int i = 0; i < passage_pairs.size(); i++) {
        vector<Point2f> cur_passage_intersection_pts(pts_num);
        for (int j = 0; j < pts_num; j++)
            cur_passage_intersection_pts[j] = res[j][i];
        chord_ends = GetEndsOfColinearPts(cur_passage_intersection_pts);
        passage_inner_ends = GetPassageInnerEnds(obstacles[passage_pairs[i][0]], obstacles[passage_pairs[i][1]]); 
        
        if (OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[0])
            && OnSegment(passage_inner_ends[0], passage_inner_ends[1], chord_ends[1])) {
                continue;
            }
        
        Point2f passage_direction_1 = passage_inner_ends[0] - res[pivot_idx][i],
                passage_direction_2 = passage_inner_ends[1] - res[pivot_idx][i];
        float safety_interior_distance = 15,
              interior_distance_1 = cv::norm(passage_direction_1),
              interior_distance_2 = cv::norm(passage_direction_2),
              max_dif_length_1 = 1e-5,
              max_dif_length_2 = 1e-5;
        if (interior_distance_1 > safety_interior_distance)
            interior_distance_1 -= safety_interior_distance;
        if (interior_distance_2 > safety_interior_distance)
            interior_distance_2 -= safety_interior_distance;
            
        for (int j = 0; j < pts_num; j++) {
            if (j == pivot_idx)
                continue;
            Point2f intersection_pt_dif = res[j][i] - res[pivot_idx][i];
            if (passage_direction_1.dot(intersection_pt_dif) > 0) {
                max_dif_length_1 = max(max_dif_length_1, (float)cv::norm(intersection_pt_dif));
            }
            else {
                max_dif_length_2 = max(max_dif_length_2, (float)cv::norm(intersection_pt_dif));
            }
        }
        float compression_ritio = min(interior_distance_1 / max_dif_length_1, interior_distance_2 / max_dif_length_2);
        for (int j = 0; j < pts_num; j++) {
            if (j == pivot_idx)
                continue;
            // cout << "Original intersection pt: " << res[j][i] << " ";
            res[j][i] = res[pivot_idx][i] + (res[j][i] - res[pivot_idx][i]) * compression_ritio;
            // cout << "Repositioned intersction pt: " << res[j][i] << '\n';
        }
    }
    return res;
}

void DeformPath(vector<Point2f>& path, 
                const vector<Point2f>& intersection_pts, 
                const vector<Point2f>& reposition_intersection_pts, 
                vector<int> path_intersection_idx) {
    vector<float> accumulated_path_length(path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) {
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(path[i] - path[i - 1]);
    }
        
    vector<int> reposition_idx(path_intersection_idx.size() + 2, 0);
    reposition_idx.back() = path.size() - 1;
    for (int i = 0; i < path_intersection_idx.size(); i++)
        reposition_idx[i + 1] = path_intersection_idx[i];

    float total_path_length = accumulated_path_length.back();
    Point2f cur_shift, next_shift, pre_shift;
    for (int i = 0; i < reposition_idx.size() - 1; i++) {
        int cur_path_idx = reposition_idx[i], next_path_idx = reposition_idx[i + 1];
        float cur_path_length_parameter = accumulated_path_length[cur_path_idx] / total_path_length,
              next_path_length_parameter = accumulated_path_length[next_path_idx] / total_path_length,
              path_segment_length_parameter = next_path_length_parameter - cur_path_length_parameter;
        if (cur_path_idx == 0) 
            cur_shift = Point2f(0, 0);
        else
            cur_shift = reposition_intersection_pts[i - 1] - intersection_pts[i - 1];
        if (next_path_idx == path.size() - 1) 
            next_shift = Point2f(0, 0);
        else
            next_shift = reposition_intersection_pts[i] - intersection_pts[i];
        
        for (int j = cur_path_idx; j < next_path_idx; j++) {
            float path_length_parameter = accumulated_path_length[j] / total_path_length,
                  ratio = (path_length_parameter - cur_path_length_parameter) / path_segment_length_parameter;
            path[j] = path[j] + ratio * next_shift + (1 - ratio) * cur_shift;
            // cout << path_length_parameter << ", " << ratio << ", " << next_shift << ", " << cur_shift << '\n';
        }
    }
}

void DeformPathWithTargetPt(vector<Point2f>& path, 
                            const vector<Point2f>& intersection_pts, 
                            const vector<Point2f>& reposition_intersection_pts, 
                            vector<int> path_intersection_idx) {
    vector<float> accumulated_path_length(path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) {
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(path[i] - path[i - 1]);
    }
        
    vector<int> reposition_idx(path_intersection_idx.size() + 2, 0);
    reposition_idx.back() = path.size() - 1;
    for (int i = 0; i < path_intersection_idx.size(); i++)
        reposition_idx[i + 1] = path_intersection_idx[i];

    float total_path_length = accumulated_path_length.back();
    Point2f cur_shift, next_shift, pre_shift;
    for (int i = 0; i < reposition_idx.size() - 1; i++) {
        int cur_path_idx = reposition_idx[i], next_path_idx = reposition_idx[i + 1];
        float cur_path_length_parameter = accumulated_path_length[cur_path_idx] / total_path_length,
              next_path_length_parameter = accumulated_path_length[next_path_idx] / total_path_length,
              path_segment_length_parameter = next_path_length_parameter - cur_path_length_parameter;
        if (cur_path_idx == 0) 
            cur_shift = Point2f(0, 0);
        else
            cur_shift = reposition_intersection_pts[i - 1] - intersection_pts[i - 1];
        // if (next_path_idx == path.size() - 1) 
        //   next_shift = Point2f(0, 0);
        // else
        next_shift = reposition_intersection_pts[i] - intersection_pts[i];
        
        for (int j = cur_path_idx; j < next_path_idx; j++) {
            float path_length_parameter = accumulated_path_length[j] / total_path_length,
                  ratio = (path_length_parameter - cur_path_length_parameter) / path_segment_length_parameter;
            path[j] = path[j] + ratio * next_shift + (1 - ratio) * cur_shift;
            // cout << path_length_parameter << ", " << ratio << ", " << next_shift << ", " << cur_shift << '\n';
        }
    }
    path.back() = reposition_intersection_pts.back();
}

vector<vector<Point2f>> GeneratePathSetInGeneralCondition(const vector<Point2f>& initial_feedback_pts, 
                                                          const vector<Point2f>& target_feedback_pts, 
                                                          int pivot_idx,
                                                          float feedback_pts_radius,
                                                          const vector<PolygonObstacle>& obs,
                                                          Mat source_img) {
    // vector<vector<Point2f>> res_path_set(initial_feedback_pts.size());   

    RRTStarPlanner planner(initial_feedback_pts[pivot_idx], target_feedback_pts[pivot_idx], obs, 25);
    bool plan_success = planner.Plan(source_img, feedback_pts_radius, true);
    if (!plan_success) {
        cout << "Path planning failed! An empty path set is reutrned.\n";
        return {};
    }

    vector<RRTStarNode*> pivot_path = planner.GetPath();
    vector<RRTStarNode*> sparse_pivot_path;
    for (int i = 0; i < pivot_path.size(); i += 1)
        sparse_pivot_path.push_back(pivot_path[i]);
    if (sparse_pivot_path.back() != pivot_path.back())
        sparse_pivot_path.push_back(pivot_path.back());
    vector<Point2f> smooth_pivot_path = QuadraticBSplineSmoothing(sparse_pivot_path);

    vector<vector<int>> passage_passed = GetPassagesPathPasses(obs, pivot_path);
    vector<vector<int>> intersection_idx; 
    vector<vector<Point2f>> passage_intersection_pts = GetPassageIntersectionsOfPathSet(obs, passage_passed, smooth_pivot_path, 
                                                                                        intersection_idx, initial_feedback_pts, pivot_idx);
    vector<Point2f> reposition_points = GetPivotPathRepositionPts(obs, passage_passed, passage_intersection_pts, pivot_idx);
    DeformPath(smooth_pivot_path, passage_intersection_pts[pivot_idx], reposition_points, intersection_idx[pivot_idx]); 

    passage_intersection_pts = GetPassageIntersectionsOfPathSet(obs, passage_passed, smooth_pivot_path, intersection_idx, initial_feedback_pts, pivot_idx);
    vector<vector<Point2f>> adjusted_passage_intersection_pts = AdjustRepositionPtsForPathSet(obs, passage_passed, passage_intersection_pts, pivot_idx);

    vector<vector<Point2f>> res_path_set(initial_feedback_pts.size(), smooth_pivot_path);
    for (int i = 0; i < initial_feedback_pts.size(); i++) {
        if (i == pivot_idx) 
            continue;
        for (Point2f& path_node : res_path_set[i])
            path_node += initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];

        passage_intersection_pts[i].push_back(res_path_set[i].back());
        adjusted_passage_intersection_pts[i].push_back(target_feedback_pts[i]);
        DeformPathWithTargetPt(res_path_set[i], passage_intersection_pts[i], adjusted_passage_intersection_pts[i], intersection_idx[i]);
        //res_path_set[i].back() = target_feedback_pts[i];
    }    
    return res_path_set; 
}

/****************************/
/*         Upgraded         */
/****************************/
pair<vector<int>, vector<int>> RetrievePassedPassages(const vector<RRTStarNode*>& raw_pivot_path, 
                                                    const vector<vector<Point2f>>& passage_pts) {
    vector<int> res_passage_indices, res_path_intersection_indices;
    for (int path_node_idx = 0; path_node_idx < raw_pivot_path.size() - 1; path_node_idx++) {
        for (int passage_idx = 0; passage_idx < passage_pts.size(); passage_idx++) {
            if (SegmentIntersection(raw_pivot_path[path_node_idx]->pos, raw_pivot_path[path_node_idx + 1]->pos, 
                                    passage_pts[passage_idx][0], passage_pts[passage_idx][1])) {
                res_passage_indices.push_back(passage_idx);
                res_path_intersection_indices.push_back(path_node_idx);
            }
        }
    }
    return make_pair(res_passage_indices, res_path_intersection_indices);
}

pair<vector<int>, vector<int>> RetrievePassedPassages(const vector<Point2f>& raw_pivot_path, 
                                                    const vector<vector<Point2f>>& passage_pts) {
    vector<int> res_passage_indices, res_path_intersection_indices;
    for (int path_idx = 0; path_idx < raw_pivot_path.size() - 1; path_idx++) {
        for (int passage_idx = 0; passage_idx < passage_pts.size(); passage_idx++) {
            if (SegmentIntersection(raw_pivot_path[path_idx], raw_pivot_path[path_idx + 1], 
                                    passage_pts[passage_idx][0], passage_pts[passage_idx][1]) == true) {
                res_passage_indices.push_back(passage_idx);
                res_path_intersection_indices.push_back(path_idx);
            }
        }
    }
    return make_pair(res_passage_indices, res_path_intersection_indices);
}

vector<vector<Point2f>> GetTransferPathSet(const vector<Point2f>& pivot_path, 
                                            const vector<Point2f>& initial_pts, 
                                            const vector<Point2f>& target_pts, 
                                            const int pivot_idx) {
    if (initial_pts.size() != target_pts.size()) {
        cout << "Initial points and target points do not share the same point number.\n";
        return {};
    }
    if (pivot_idx < 0 || pivot_idx >= initial_pts.size()) {
        cout << "Invalid pivot index.\n";
        return {};
    }
    vector<vector<Point2f>> res(initial_pts.size(), pivot_path);
    vector<float> accumulated_path_len(pivot_path.size(), 0);
    for (int i = 1; i < pivot_path.size(); i++)
        accumulated_path_len[i] = accumulated_path_len[i - 1] + cv::norm(pivot_path[i] - pivot_path[i - 1]);
    for (int i = 0; i < initial_pts.size(); i++) {
        if (i == pivot_idx)
            continue;
        Point2f initial_shift = initial_pts[i] - initial_pts[pivot_idx], 
                target_shift = target_pts[i] - target_pts[pivot_idx];
        for (int j = 0; j < pivot_path.size(); j++) {
            float cur_path_parameter = accumulated_path_len[j] / accumulated_path_len.back();
            res[i][j] = pivot_path[j] + cur_path_parameter * target_shift + (1 - cur_path_parameter) * initial_shift;
        }
    }
    return res;
}

pair<vector<Point2f>, vector<int>> GetPathSetIntersectionsOnPassageLine(const vector<Point2f>& pivot_path, 
                                                                        const vector<Point2f>& initial_pts, 
                                                                        const vector<Point2f>& target_pts, 
                                                                        const int pivot_idx, 
                                                                        const vector<Point2f>& single_passage_pts) {
    vector<Point2f> res_pts(initial_pts.size());
    vector<int> res_indices(initial_pts.size(), 0);
    vector<float> accumulated_path_length(pivot_path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) 
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(pivot_path[i] - pivot_path[i - 1]);
    
    // Extend the passage segment to be sufficiently long
    float extend_len = 200;
    vector<Point2f> extended_single_passage_pts = single_passage_pts;
    Point2f passage_direction_1_2 = (single_passage_pts[1] - single_passage_pts[0]) / cv::norm(single_passage_pts[1] - single_passage_pts[0]);
    extended_single_passage_pts[0] = extended_single_passage_pts[0] - extend_len * passage_direction_1_2;
    extended_single_passage_pts[1] = extended_single_passage_pts[1] + extend_len * passage_direction_1_2;

    // Need to ensure the intersection point of the pivot path is found within the non-extended passage.
    for (int path_idx = 0; path_idx < pivot_path.size() - 1; path_idx++) {
        if (SegmentIntersection(pivot_path[path_idx], pivot_path[path_idx + 1], single_passage_pts[0], single_passage_pts[1]) == true) {
            res_pts[pivot_idx] = GetSegmentsIntersectionPt(pivot_path[path_idx], pivot_path[path_idx + 1], 
                                                        single_passage_pts[0], single_passage_pts[1]);
            res_indices[pivot_idx] = path_idx;
            break;
        }  
    }

    for (int pt_idx = 0; pt_idx < initial_pts.size(); pt_idx++) {
        if (pt_idx == pivot_idx)
            continue;
            
        vector<Point2f> candidate_pt;
        vector<int> candidate_idx;
        Point2f initial_diff = initial_pts[pt_idx] - initial_pts[pivot_idx], 
                target_diff = target_pts[pt_idx] - target_pts[pivot_idx];
        for (int path_idx = 0; path_idx < pivot_path.size() - 1; path_idx++) {
            float cur_parameter_1 = accumulated_path_length[path_idx] / accumulated_path_length.back(),
                  cur_parameter_2 = accumulated_path_length[path_idx + 1] / accumulated_path_length.back();
            Point2f cur_path_pos_1 = pivot_path[path_idx] + (1 - cur_parameter_1) * initial_diff + cur_parameter_1 * target_diff,
                    cur_path_pos_2 = pivot_path[path_idx + 1] + (1 - cur_parameter_2) * initial_diff + cur_parameter_2 * target_diff;
            if (SegmentIntersection(cur_path_pos_1, cur_path_pos_2, extended_single_passage_pts[0], extended_single_passage_pts[1]) == true) {
                Point2f cur_intersection_pt = GetSegmentsIntersectionPt(cur_path_pos_1, cur_path_pos_2, 
                                                            extended_single_passage_pts[0], extended_single_passage_pts[1]);
                candidate_pt.push_back(cur_intersection_pt);
                candidate_idx.push_back(path_idx);
            }
        }
        if (candidate_idx.size() == 0)
            continue;

        int ref_idx = res_indices[pivot_idx], min_idx_diff = INT_MAX;
        for (int i = 0; i < candidate_idx.size(); i++)
            if (abs(candidate_idx[i] - ref_idx) < min_idx_diff) {
                min_idx_diff = abs(candidate_idx[i] - ref_idx);
                res_indices[pt_idx] = candidate_idx[i];
                res_pts[pt_idx] = candidate_pt[i];
            }
    }
    return make_pair(res_pts, res_indices);
}

pair<vector<int>, vector<Point2f>> GetIntersectionIdxPtsWithPivotRef(const vector<Point2f>& pivot_path, 
                                                                const vector<float>& accumulated_path_length,
                                                                const vector<Point2f>& initial_pts, 
                                                                const vector<Point2f>& target_pts, 
                                                                const int pivot_idx,
                                                                const vector<Point2f>& passage_pts,
                                                                Point2f pivot_intersection_pt, 
                                                                int pivot_intersection_path_idx) {
    int pt_num = initial_pts.size();
    vector<int> res_idx(pt_num, 0);
    vector<Point2f> res_intersection_pts(pt_num);
    res_idx[pivot_idx] = pivot_intersection_path_idx;
    res_intersection_pts[pivot_idx] = pivot_intersection_pt;

    Point2f passage_direction_1_2 = (passage_pts[1] - passage_pts[0]) / cv::norm(passage_pts[1] - passage_pts[0]),
            extended_passage_pt_1 = passage_pts[0] - 100 * passage_direction_1_2,
            extended_passage_pt_2 = passage_pts[1] + 100 * passage_direction_1_2;

    for (int pt_idx = 0; pt_idx < initial_pts.size(); pt_idx++) {
        if (pt_idx == pivot_idx)
            continue;

        Point2f initial_diff = initial_pts[pt_idx] - initial_pts[pivot_idx], 
                target_diff = target_pts[pt_idx] - target_pts[pivot_idx];
        int left_idx = pivot_intersection_path_idx, 
            right_idx = pivot_intersection_path_idx + 1;
        for ( ; left_idx >= 0 || right_idx < pivot_path.size() - 1; ) {
            if (left_idx >= 0) {
                float cur_parameter_1 = accumulated_path_length[left_idx] / accumulated_path_length.back(),
                      cur_parameter_2 = accumulated_path_length[left_idx + 1] / accumulated_path_length.back();
                Point2f cur_path_pos_1 = pivot_path[left_idx] + (1 - cur_parameter_1) * initial_diff + cur_parameter_1 * target_diff,
                        cur_path_pos_2 = pivot_path[left_idx + 1] + (1 - cur_parameter_2) * initial_diff + cur_parameter_2 * target_diff;
                if (SegmentIntersection(cur_path_pos_1, cur_path_pos_2, extended_passage_pt_1, extended_passage_pt_2) == true) {
                    Point2f cur_intersection_pt = GetSegmentsIntersectionPt(cur_path_pos_1, cur_path_pos_2, 
                                                                extended_passage_pt_1, extended_passage_pt_2);
                    res_idx[pt_idx] = left_idx;
                    res_intersection_pts[pt_idx] = cur_intersection_pt;
                    break;
                }
                left_idx--;
            }
            if (right_idx < pivot_path.size() - 1) {
                float cur_parameter_1 = accumulated_path_length[right_idx] / accumulated_path_length.back(),
                      cur_parameter_2 = accumulated_path_length[right_idx + 1] / accumulated_path_length.back();
                Point2f cur_path_pos_1 = pivot_path[right_idx] + (1 - cur_parameter_1) * initial_diff + cur_parameter_1 * target_diff,
                        cur_path_pos_2 = pivot_path[right_idx + 1] + (1 - cur_parameter_2) * initial_diff + cur_parameter_2 * target_diff;
                if (SegmentIntersection(cur_path_pos_1, cur_path_pos_2, extended_passage_pt_1, extended_passage_pt_2) == true) {
                    Point2f cur_intersection_pt = GetSegmentsIntersectionPt(cur_path_pos_1, cur_path_pos_2, 
                                                                extended_passage_pt_1, extended_passage_pt_2);
                    res_idx[pt_idx] = right_idx;
                    res_intersection_pts[pt_idx] = cur_intersection_pt;
                    break;
                }
                right_idx++;
            }            
        }                       
    }                                                        
    return make_pair(res_idx, res_intersection_pts);
}

pair<vector<vector<int>>, vector<vector<Point2f>>> AddGeneralPassagesSingleSide(const vector<vector<int>>& checked_passage_pair,
                                                                            const vector<vector<Point2f>>& checked_passage_pts,
                                                                            const vector<PolygonObstacle>& obstacles,
                                                                            const vector<Point2f>& pivot_path) {
    vector<vector<int>> augment_passage_pair = checked_passage_pair;
    vector<vector<Point2f>> augment_passage_pts = checked_passage_pts;
    auto passage_passing_res = RetrievePassedPassages(pivot_path, checked_passage_pts);
    vector<int> passed_passage_indices = passage_passing_res.first,
                pivot_path_intersection_indices = passage_passing_res.second;
    int passed_passage_num = passed_passage_indices.size();
    for (int i = 0; i < passed_passage_num; i++) {
        int pivot_path_intersection_idx = pivot_path_intersection_indices[i];
        Point2f path_segment = pivot_path[pivot_path_intersection_idx + 1] - pivot_path[pivot_path_intersection_idx],
                path_segment_direction = path_segment / cv::norm(path_segment),
                extended_path_segment_pt_1 = pivot_path[pivot_path_intersection_idx] - path_segment_direction * 5,
                extended_path_segment_pt_2 = pivot_path[pivot_path_intersection_idx + 1] + path_segment_direction * 5,
                extended_path_segment = extended_path_segment_pt_2 - extended_path_segment_pt_1;
        
        int passage_pair_idx = passed_passage_indices[i],
            cur_obs_idx_1 = checked_passage_pair[passage_pair_idx][0],
            cur_obs_idx_2 = checked_passage_pair[passage_pair_idx][1];
        PolygonObstacle cur_obs_1 = obstacles[cur_obs_idx_1],
                        cur_obs_2 = obstacles[cur_obs_idx_2];
        Point2f obs_reference_pt_1 = cur_obs_1.vertices[0],
                obs_extended_pt_dist_1 = obs_reference_pt_1 - extended_path_segment_pt_1;
        
        float cross_product_1 = obs_extended_pt_dist_1.x * extended_path_segment.y - obs_extended_pt_dist_1.y * extended_path_segment.x;
        PolygonObstacle objective_obs;
        int objective_obs_idx;
        if (cross_product_1 > 0) {
            objective_obs = cur_obs_1;
            objective_obs_idx = cur_obs_idx_1;
        }
        else {
            objective_obs = cur_obs_2;
            objective_obs_idx = cur_obs_idx_2;
        }
        
        vector<int> containing_passage_indices;
        vector<Point2f> passage_pts_on_cur_obs,
                        passage_pts_on_other_obs;
        for (int j = 0; j < passed_passage_num; j++) {
            int cur_passed_passage_idx = passed_passage_indices[j];
            if (checked_passage_pair[cur_passed_passage_idx][0] == objective_obs_idx) {
                containing_passage_indices.push_back(cur_passed_passage_idx);
                passage_pts_on_cur_obs.push_back(checked_passage_pts[cur_passed_passage_idx][0]);
                passage_pts_on_other_obs.push_back(checked_passage_pts[cur_passed_passage_idx][1]);
            }
            else if (checked_passage_pair[cur_passed_passage_idx][1] == objective_obs_idx) {
                containing_passage_indices.push_back(cur_passed_passage_idx);
                passage_pts_on_cur_obs.push_back(checked_passage_pts[cur_passed_passage_idx][1]);
                passage_pts_on_other_obs.push_back(checked_passage_pts[cur_passed_passage_idx][0]);                
            }
        }
        
        int cur_obs_vertex_num = objective_obs.vertices.size();
        vector<bool> is_vertex_occupied(cur_obs_vertex_num, false);
        for (int j = 0; j < cur_obs_vertex_num; j++) {
            for (int k = 0; k < passage_pts_on_cur_obs.size(); k++)
                if (cv::norm(objective_obs.vertices[j] - passage_pts_on_cur_obs[k]) < 0.1) {
                    is_vertex_occupied[j] = true;
                    break;
                }
        }

        for (int j = 0; j < cur_obs_vertex_num; j++) {
            if (is_vertex_occupied[j] == true)
                continue;
            
            float min_dist_to_vertex = FLT_MAX;
            for (int k = 0; k < passage_pts_on_cur_obs.size(); k++) {
                Point2f candidate_passage_direction = passage_pts_on_other_obs[k] - passage_pts_on_cur_obs[k];
                candidate_passage_direction = candidate_passage_direction / cv::norm(candidate_passage_direction);
                float passage_pt_to_vertex_dist = cv::norm(passage_pts_on_cur_obs[k] - objective_obs.vertices[j]);
                
                Point2f candidate_passage_virtual_start = objective_obs.vertices[j] + 5 * candidate_passage_direction,
                        candidate_passage_end = objective_obs.vertices[j] + 200 * candidate_passage_direction;
                if (ObstacleFree(objective_obs, candidate_passage_virtual_start, candidate_passage_end) == false)
                    continue;
                
                if (passage_pt_to_vertex_dist < min_dist_to_vertex - 0.1) {
                    min_dist_to_vertex = passage_pt_to_vertex_dist;
                    pair<int, Point2f> temp_res = FindPassageEndonObstacles(obstacles, candidate_passage_virtual_start, candidate_passage_end);
                    vector<int> cur_res_passage_pair{objective_obs_idx, temp_res.first};
                    vector<Point2f> cur_res_passage_pts{objective_obs.vertices[j], temp_res.second};
                    
                    if (temp_res.second.x > 0) {
                        augment_passage_pair.push_back(cur_res_passage_pair);
                        augment_passage_pts.push_back(cur_res_passage_pts);
                    }
                }
            }
        } 
    }
    
    return make_pair(augment_passage_pair, augment_passage_pts);
}

pair<vector<int>, vector<Point2f>> UpdateChordByTubeGeometry(const vector<Point2f>& pivot_path, 
                                        const vector<float>& accumulated_path_length,
                                        const vector<Point2f>& initial_pts, 
                                        const vector<Point2f>& target_pts, 
                                        const int pivot_idx,
                                        const vector<Point2f>& passage_pts,
                                        Point2f pivot_intersection_pt, 
                                        int pivot_intersection_path_idx) {
    vector<int> res_idx(initial_pts.size());
    vector<Point2f> res_pts(initial_pts.size());
    res_idx[pivot_idx] = pivot_idx;
    res_pts[pivot_idx] = pivot_intersection_pt;

    Point2f prev_pivot_path_pt = pivot_path[pivot_intersection_path_idx],
            next_pivot_path_pt = pivot_path[pivot_intersection_path_idx + 1],
            local_tangent = (next_pivot_path_pt - prev_pivot_path_pt) / cv::norm(next_pivot_path_pt - prev_pivot_path_pt),
            local_normal = Point2f(-local_tangent.y, local_tangent.x);
    vector<Point2f> extended_local_width_pt{pivot_intersection_pt + 100 * local_normal,
                                            pivot_intersection_pt - 100 * local_normal};
    
    for (int pt_idx = 0; pt_idx < initial_pts.size(); pt_idx++) {
        if (pt_idx == pivot_idx)
            continue;

        Point2f initial_diff = initial_pts[pt_idx] - initial_pts[pivot_idx], 
                target_diff = target_pts[pt_idx] - target_pts[pivot_idx];
        int left_idx = pivot_intersection_path_idx, 
            right_idx = pivot_intersection_path_idx + 1;
        for ( ; left_idx >= 0 || right_idx < pivot_path.size() - 1; ) {
            if (left_idx >= 0) {
                float cur_parameter_1 = accumulated_path_length[left_idx] / accumulated_path_length.back(),
                      cur_parameter_2 = accumulated_path_length[left_idx + 1] / accumulated_path_length.back();
                Point2f cur_path_pos_1 = pivot_path[left_idx] + (1 - cur_parameter_1) * initial_diff + cur_parameter_1 * target_diff,
                        cur_path_pos_2 = pivot_path[left_idx + 1] + (1 - cur_parameter_2) * initial_diff + cur_parameter_2 * target_diff;
                if (SegmentIntersection(cur_path_pos_1, cur_path_pos_2, extended_local_width_pt[0], extended_local_width_pt[1]) == true) {
                    Point2f cur_intersection_pt = GetSegmentsIntersectionPt(cur_path_pos_1, cur_path_pos_2, 
                                                                extended_local_width_pt[0], extended_local_width_pt[1]);
                    res_idx[pt_idx] = left_idx;
                    res_pts[pt_idx] = cur_intersection_pt;
                    break;
                }
                left_idx--;
            }
            if (right_idx < pivot_path.size() - 1) {
                float cur_parameter_1 = accumulated_path_length[right_idx] / accumulated_path_length.back(),
                      cur_parameter_2 = accumulated_path_length[right_idx + 1] / accumulated_path_length.back();
                Point2f cur_path_pos_1 = pivot_path[right_idx] + (1 - cur_parameter_1) * initial_diff + cur_parameter_1 * target_diff,
                        cur_path_pos_2 = pivot_path[right_idx + 1] + (1 - cur_parameter_2) * initial_diff + cur_parameter_2 * target_diff;
                if (SegmentIntersection(cur_path_pos_1, cur_path_pos_2, extended_local_width_pt[0], extended_local_width_pt[1]) == true) {
                    Point2f cur_intersection_pt = GetSegmentsIntersectionPt(cur_path_pos_1, cur_path_pos_2, 
                                                                extended_local_width_pt[0], extended_local_width_pt[1]);
                    res_idx[pt_idx] = right_idx;
                    res_pts[pt_idx] = cur_intersection_pt;
                    break;
                }
                right_idx++;
            }            
        }                       
    }

    vector<Point2f> tube_ends = GetEndsOfColinearPts(res_pts);
    Point2f passage_direction_1_2 = (passage_pts[1] - passage_pts[0]) / cv::norm(passage_pts[1] - passage_pts[0]),
            tube_direction_1_2 = (tube_ends[1] - tube_ends[0]) / cv::norm(tube_ends[1] - tube_ends[0]);
    float direction_inner_product = passage_direction_1_2.dot(tube_direction_1_2);
    if (direction_inner_product < 0) {
        direction_inner_product *= -1;
        tube_direction_1_2 *= -1;
    }

    /*** Note that the image frame makes y point down. 
     *   Check the rihgt-hand rule and rotation relation ***/
    bool counter_clock_wise_rotate = true;
    float direction_cross_product = tube_direction_1_2.x * passage_direction_1_2.y - tube_direction_1_2.y * passage_direction_1_2.x;
    if (direction_cross_product > 0)
        counter_clock_wise_rotate = false;
    
    float angle_cos = direction_inner_product, angle_sin = sqrt(1 - angle_cos * angle_cos);
    for (Point2f& tube_pt : res_pts) {
        tube_pt -= pivot_intersection_pt;
        float cur_x = tube_pt.x, cur_y = tube_pt.y; 
        if (counter_clock_wise_rotate == true) {
            tube_pt.x = angle_cos * cur_x + angle_sin * cur_y;
            tube_pt.y = -angle_sin * cur_x + angle_cos * cur_y;
        }
        else {
            tube_pt.x = angle_cos * cur_x - angle_sin * cur_y;
            tube_pt.y = angle_sin * cur_x + angle_cos * cur_y;            
        }
        tube_pt += pivot_intersection_pt;
    }
    return make_pair(res_idx, res_pts);
}

vector<Point2f> RepositionPivotPath(const vector<Point2f>& pivot_path, 
                                    const vector<Point2f>& initial_pts, 
                                    const vector<Point2f>& target_pts, 
                                    const int pivot_idx,
                                    const vector<vector<Point2f>>& passage_pts, 
                                    vector<PolygonObstacle>& obstacles,
                                    Mat& back_img) {
    vector<float> accumulated_path_length(pivot_path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) 
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(pivot_path[i] - pivot_path[i - 1]);     
    
    vector<int> passed_passage_indices = RetrievePassedPassages(pivot_path, passage_pts).first;
    vector<Point2f> adjust_reference_pts(passed_passage_indices.size(), Point2f(-1, -1)), 
                    adjust_intersection_pts(passed_passage_indices.size(), Point2f(-1, -1));
    vector<int> adjust_path_indices(passed_passage_indices.size(), -1);
    vector<int> adjust_types(passed_passage_indices.size(), 0);
    float adjust_clearance_small = 2, adjust_clearance_large = 5;

    for (int i = 0; i < passed_passage_indices.size(); i++) {
        vector<Point2f> cur_passage_pts = passage_pts[passed_passage_indices[i]];
        pair<vector<Point2f>, vector<int>> cur_intersection_pts_indices = GetPathSetIntersectionsOnPassageLine(pivot_path, initial_pts, 
                                                                                                            target_pts, pivot_idx, cur_passage_pts);
        vector<Point2f> cur_intersection_pts = cur_intersection_pts_indices.first;
        vector<Point2f> chord_ends = GetEndsOfColinearPts(cur_intersection_pts);
 
        
        bool some_intersection_pt_not_found = false;
        for (int& path_idx : cur_intersection_pts_indices.second)
            if (path_idx == 0) {
                some_intersection_pt_not_found = true;
                break;
            } 

        // Directly set true to use the tube length
        if (true || some_intersection_pt_not_found == true) {
            Point2f pivot_intersection_pt = cur_intersection_pts[pivot_idx];
            int pivot_intersection_idx = cur_intersection_pts_indices.second[pivot_idx];
            pair<vector<int>, vector<Point2f>> tube_check_res = UpdateChordByTubeGeometry(pivot_path, accumulated_path_length, 
                                                            initial_pts, target_pts, pivot_idx, 
                                                            cur_passage_pts, 
                                                            pivot_intersection_pt, pivot_intersection_idx);
            cur_intersection_pts = tube_check_res.second;
            // cur_intersection_pts_indices.second = tube_check_res.first;
            chord_ends = GetEndsOfColinearPts(cur_intersection_pts);
        }
        // line(back_img, chord_ends[0], chord_ends[1], Scalar(0, 0, 255), 2);
        if (OnSegment(cur_passage_pts[0], cur_passage_pts[1], chord_ends[0]) && OnSegment(cur_passage_pts[0], cur_passage_pts[1], chord_ends[1])) {
            cout << "No change\n";
            adjust_path_indices[i] = cur_intersection_pts_indices.second[pivot_idx];
            adjust_intersection_pts[i] = cur_intersection_pts[pivot_idx];
            adjust_reference_pts[i] = cur_intersection_pts[pivot_idx];
/*             circle(back_img, chord_ends[0], 4, Scalar(0, 0, 0), -1);
            circle(back_img, chord_ends[1], 4, Scalar(0, 0, 0), -1);    
            line(back_img, chord_ends[0], chord_ends[1], Scalar(0, 0, 0), 2);   */      
            continue;
        }

        Point2f passage_direction_1_2 = (cur_passage_pts[1] - cur_passage_pts[0]) / cv::norm(cur_passage_pts[1] - cur_passage_pts[0]),
                chord_direction_1_2 = (chord_ends[1] - chord_ends[0]) / cv::norm(chord_ends[1] - chord_ends[0]);
        float direction_inner_product = passage_direction_1_2.dot(chord_direction_1_2);
        if (direction_inner_product < 0)
            std::swap(chord_ends[0], chord_ends[1]);

        Point2f pivot_intersection_pt = cur_intersection_pts[pivot_idx];
        float passage_segment_len = cv::norm(cur_passage_pts[0] - cur_passage_pts[1]),
              chord_len = cv::norm(chord_ends[0] - chord_ends[1]), 
              dist_to_chord_end_1 = cv::norm(pivot_intersection_pt - chord_ends[0]);

        Point2f cur_reference_pt;
        if (passage_segment_len <= chord_len) {
            cout << "Compression\n";
            cur_reference_pt = cur_passage_pts[0] + adjust_clearance_small * passage_direction_1_2 
                                + (passage_segment_len - 2 * adjust_clearance_small) * dist_to_chord_end_1 / chord_len * passage_direction_1_2;
        }
        else {
            cout << "Translation\n";
            Point2f reference_passage_end = cur_passage_pts[0], reference_direction = passage_direction_1_2;
            float reference_distance = dist_to_chord_end_1;
            if (OnSegment(chord_ends[0], chord_ends[1], cur_passage_pts[1]) == true) {
                reference_passage_end = cur_passage_pts[1];
                reference_direction *= -1;
                reference_distance = cv::norm(pivot_intersection_pt - chord_ends[1]);
            }
            cur_reference_pt = reference_passage_end + adjust_clearance_large * reference_direction + reference_distance * reference_direction;            
        }
        adjust_path_indices[i] = cur_intersection_pts_indices.second[pivot_idx];
        adjust_intersection_pts[i] = cur_intersection_pts[pivot_idx];
        adjust_reference_pts[i] = cur_reference_pt;
        adjust_types[i] = 1;

/*         circle(back_img, chord_ends[0], 4, Scalar(0, 0, 0), -1);
        circle(back_img, chord_ends[1], 4, Scalar(0, 0, 0), -1);
        line(back_img, chord_ends[0], chord_ends[1], Scalar(0, 0, 0), 2);
        circle(back_img, cur_reference_pt, 4, Scalar(0, 0, 255), -1); */
    }
    // return adjust_reference_pts;

    vector<int> augment_adjust_path_indices{0};
    vector<Point2f> augment_adjust_reference_pts{pivot_path[0]}, augment_adjust_intersection_pts{pivot_path[0]};
    for (int i = 0; i < adjust_path_indices.size(); i++) {
        // Filter out potential jerky segments
        if (i > 0 && i < adjust_path_indices.size() - 1 && adjust_types[i] == 0 && adjust_types[i + 1] == 1) {
            if (obstacleFreeVec(obstacles, adjust_reference_pts[i - 1], adjust_reference_pts[i + 1]) == true)
                continue;
        }

        augment_adjust_path_indices.push_back(adjust_path_indices[i]);
        augment_adjust_reference_pts.push_back(adjust_reference_pts[i]);
        augment_adjust_intersection_pts.push_back(adjust_intersection_pts[i]);
    }
    augment_adjust_path_indices.push_back(pivot_path.size() - 1);
    augment_adjust_reference_pts.push_back(pivot_path.back());
    augment_adjust_intersection_pts.push_back(pivot_path.back()); 
    
    vector<Point2f> reposition_pivot_path(pivot_path.size());
    reposition_pivot_path[0] = pivot_path[0];
    int reposition_pivot_path_idx = 1;
    for (int i = 0; i < augment_adjust_path_indices.size() - 1; i++) {
        int start_pivot_path_idx = augment_adjust_path_indices[i] + 1,
            end_pivot_path_idx = augment_adjust_path_indices[i + 1];  // pivot path point index before the reference pt

        Point2f prev_reference_pt = augment_adjust_reference_pts[i], 
                next_reference_pt = augment_adjust_reference_pts[i + 1], 
                prev_intersection_pt = augment_adjust_intersection_pts[i],
                next_intersection_pt = augment_adjust_intersection_pts[i + 1]; 

        float  prev_path_len_parameter = accumulated_path_length[start_pivot_path_idx],
               next_path_len_parameter = accumulated_path_length[end_pivot_path_idx] + cv::norm(next_intersection_pt - pivot_path[end_pivot_path_idx]),
               path_len_diff = next_path_len_parameter - prev_path_len_parameter;
        Point2f prev_shift = prev_reference_pt - prev_intersection_pt, 
                next_shift = next_reference_pt - next_intersection_pt; 
        for (int j = start_pivot_path_idx; j <= end_pivot_path_idx; j++) {
            float cur_path_len_parameter = accumulated_path_length[j], 
                  cur_ratio = (cur_path_len_parameter - prev_path_len_parameter) / path_len_diff;
            Point2f reposition_pt = pivot_path[j] + cur_ratio *  next_shift + (1 - cur_ratio) * prev_shift;
            reposition_pivot_path[reposition_pivot_path_idx++] = reposition_pt;
        }
    }
    return reposition_pivot_path;
}

vector<vector<Point2f>> GeneratePathSetUpdated(const vector<Point2f>& pivot_path, 
                                                const vector<Point2f>& initial_pts, 
                                                const vector<Point2f>& target_pts, 
                                                const int pivot_idx,
                                                const vector<vector<Point2f>>& passage_pts, 
                                                vector<PolygonObstacle>& obstacles,
                                                Mat& back_img) {
    int pt_num = initial_pts.size();
    vector<Point2f> reposition_pivot_path = RepositionPivotPath(pivot_path, initial_pts, target_pts, pivot_idx, passage_pts, obstacles, back_img);

    vector<float> accumulated_path_length(reposition_pivot_path.size(), 0);
    for (int i = 1; i < accumulated_path_length.size(); i++) 
        accumulated_path_length[i] = accumulated_path_length[i - 1] + cv::norm(reposition_pivot_path[i] - reposition_pivot_path[i - 1]);     
    
    vector<int> passed_passage_indices = RetrievePassedPassages(reposition_pivot_path, passage_pts).first;
    int passed_passage_num = passed_passage_indices.size();
    vector<vector<Point2f>> adjust_reference_pts(passed_passage_num, vector<Point2f>(pt_num)), 
                            adjust_intersection_pts(passed_passage_num, vector<Point2f>(pt_num));
    vector<vector<int>> adjust_path_indices(passed_passage_num, vector<int>(pt_num, 0));
    vector<int> adjust_types(passed_passage_num, 0);
    float adjust_clearance_small = 2, adjust_clearance_large = 5;

    cout << "\n\nTotal passed passage number: " << passed_passage_num << "\n";
    int pivot_path_idx = 0;
    for (int passage_idx = 0; passage_idx < passed_passage_num; passage_idx++) {
        vector<Point2f> cur_passage_pts = passage_pts[passed_passage_indices[passage_idx]];
        Point2f pivot_intersection_pt;
        int pivot_intersection_idx = 0;
        for (; pivot_path_idx < reposition_pivot_path.size() - 1; pivot_path_idx++) 
            if (SegmentIntersection(reposition_pivot_path[pivot_path_idx], reposition_pivot_path[pivot_path_idx + 1],
                                    cur_passage_pts[0], cur_passage_pts[1]) == true) {
                pivot_intersection_pt = GetSegmentsIntersectionPt(reposition_pivot_path[pivot_path_idx], reposition_pivot_path[pivot_path_idx + 1],
                                    cur_passage_pts[0], cur_passage_pts[1]);
                pivot_intersection_idx = pivot_path_idx;
                auto intersection_info_res = GetIntersectionIdxPtsWithPivotRef(reposition_pivot_path, accumulated_path_length,
                                                                            initial_pts, target_pts, pivot_idx, 
                                                                            cur_passage_pts, pivot_intersection_pt,
                                                                            pivot_intersection_idx);
                adjust_path_indices[passage_idx] = intersection_info_res.first;
                adjust_intersection_pts[passage_idx] = intersection_info_res.second;
                break;
            }

        vector<Point2f> cur_tube_intersection_pts = UpdateChordByTubeGeometry(reposition_pivot_path, accumulated_path_length, 
                                                                            initial_pts, target_pts, pivot_idx, 
                                                                            cur_passage_pts, 
                                                                            pivot_intersection_pt, 
                                                                            pivot_intersection_idx).second;
        vector<Point2f> tube_chord_ends = GetEndsOfColinearPts(cur_tube_intersection_pts);  
        vector<Point2f> pure_chord_ends = GetEndsOfColinearPts(adjust_intersection_pts[passage_idx]);
        // line(back_img, tube_chord_ends[0], tube_chord_ends[1], Scalar(0, 0, 255), 2);
        // circle(back_img, tube_chord_ends[0], 4, Scalar(0, 0, 255), -1);   
        // circle(back_img, tube_chord_ends[1], 4, Scalar(0, 0, 255), -1);   
        // line(back_img, pure_chord_ends[0], pure_chord_ends[1], Scalar(0, 255, 0), 2);

        if (OnSegment(cur_passage_pts[0], cur_passage_pts[1], tube_chord_ends[0]) && OnSegment(cur_passage_pts[0], cur_passage_pts[1], tube_chord_ends[1])) {
            if (OnSegment(cur_passage_pts[0], cur_passage_pts[1], pure_chord_ends[0]) && OnSegment(cur_passage_pts[0], cur_passage_pts[1], pure_chord_ends[1])) {
                adjust_reference_pts[passage_idx] = adjust_intersection_pts[passage_idx];
                cout << "No change 1\n";
            }
            else {
                adjust_reference_pts[passage_idx] = cur_tube_intersection_pts; 
                cout << "No change 2\n";
            }
            continue;           
        }
        
        adjust_types[passage_idx] = 1;
        float passage_len = cv::norm(cur_passage_pts[1] - cur_passage_pts[0]),
              chord_len = cv::norm(tube_chord_ends[1] - tube_chord_ends[0]);
        Point2f passage_direction_1_2 = (cur_passage_pts[1] - cur_passage_pts[0]) / passage_len,
                chord_direction_1_2 = (tube_chord_ends[1] - tube_chord_ends[0]) / chord_len;
        float direction_inner_product = passage_direction_1_2.dot(chord_direction_1_2);
        if (direction_inner_product < 0)
            std::swap(tube_chord_ends[0], tube_chord_ends[1]);
        
        if (passage_len <= chord_len) {
            cout << "Compression\n";
            for (int pt_idx = 0; pt_idx < pt_num; pt_idx++) {
                Point2f dist_vec_to_chord_end_1 = cur_tube_intersection_pts[pt_idx] - tube_chord_ends[0];
                float dist_to_chord_end_1 = cv::norm(dist_vec_to_chord_end_1);
                Point2f cur_reference_pt = cur_passage_pts[0] + adjust_clearance_small * passage_direction_1_2 + 
                                        dist_to_chord_end_1 / chord_len * (passage_len - 2 * adjust_clearance_small) * passage_direction_1_2;
                adjust_reference_pts[passage_idx][pt_idx] = cur_reference_pt;
            }
        }
        else {
            cout << "Translation\n";
            Point2f reference_passage_end = cur_passage_pts[0], 
                    reference_chord_end = tube_chord_ends[0],
                    reference_direction = passage_direction_1_2;
            if (OnSegment(tube_chord_ends[0], tube_chord_ends[1], cur_passage_pts[1]) == true) {
                reference_passage_end = cur_passage_pts[1];
                reference_chord_end = tube_chord_ends[1];
                reference_direction *= -1;
            } 
            for (int pt_idx = 0; pt_idx < pt_num; pt_idx++) {
                float reference_distance = cv::norm(cur_tube_intersection_pts[pt_idx] - reference_chord_end);
                adjust_reference_pts[passage_idx][pt_idx] = reference_passage_end + adjust_clearance_large * reference_direction
                                                            + reference_distance * reference_direction;
            }
        }
    }

    // Augmenting vectors
    vector<vector<int>> augment_adjust_path_indices(passed_passage_num + 2, vector<int>(pt_num, 0));
    vector<vector<Point2f>> augment_adjust_intersection_pts(passed_passage_num + 2, vector<Point2f>(pt_num)),
                            augment_adjust_reference_pts(passed_passage_num + 2, vector<Point2f>(pt_num));
    for (int i = 0; i < pt_num; i++) {
        augment_adjust_path_indices.back()[i] = reposition_pivot_path.size() - 1;
        augment_adjust_intersection_pts[0][i] = initial_pts[i];
        augment_adjust_intersection_pts.back()[i] = target_pts[i];
        augment_adjust_reference_pts[0][i] = initial_pts[i];
        augment_adjust_reference_pts.back()[i] = target_pts[i];
    }
    for (int i = 0; i < passed_passage_num; i++) {
        augment_adjust_path_indices[i + 1] = adjust_path_indices[i];
        augment_adjust_intersection_pts[i + 1] = adjust_intersection_pts[i];
        augment_adjust_reference_pts[i + 1] = adjust_reference_pts[i];
    }

    vector<vector<Point2f>> res_path_set(pt_num, reposition_pivot_path);
    for (int pt_idx = 0; pt_idx < pt_num; pt_idx++) {
        cout << "\n";
        if (pt_idx == pivot_idx) 
            continue;

        Point2f initial_shift = initial_pts[pt_idx] - initial_pts[pivot_idx],
                target_shift = target_pts[pt_idx] - target_pts[pivot_idx];
        for (int path_idx = 0; path_idx < reposition_pivot_path.size(); path_idx++) {
            float cur_ratio = accumulated_path_length[path_idx] / accumulated_path_length.back();
            res_path_set[pt_idx][path_idx] = reposition_pivot_path[path_idx] + cur_ratio * target_shift 
                                            + (1 - cur_ratio) * initial_shift;
        }

        vector<float> cur_accumulated_path_length(res_path_set[pt_idx].size(), 0);
        for (int i = 1; i < cur_accumulated_path_length.size(); i++)
            cur_accumulated_path_length[i] = cur_accumulated_path_length[i - 1] + cv::norm(res_path_set[pt_idx][i] - res_path_set[pt_idx][i - 1]);

        for (int i = 0; i < augment_adjust_path_indices.size() - 1; i++) {
            int start_pivot_path_idx = augment_adjust_path_indices[i][pt_idx] + 1,
                end_pivot_path_idx = augment_adjust_path_indices[i + 1][pt_idx];  // path point index before the reference pt
            if (start_pivot_path_idx > end_pivot_path_idx)
                continue;
                
            Point2f prev_reference_pt = augment_adjust_reference_pts[i][pt_idx], 
                    next_reference_pt = augment_adjust_reference_pts[i + 1][pt_idx], 
                    prev_intersection_pt = augment_adjust_intersection_pts[i][pt_idx],
                    next_intersection_pt = augment_adjust_intersection_pts[i + 1][pt_idx]; 

            float  prev_path_len_parameter = cur_accumulated_path_length[start_pivot_path_idx],
                next_path_len_parameter = cur_accumulated_path_length[end_pivot_path_idx] 
                                        + cv::norm(next_intersection_pt - res_path_set[pt_idx][end_pivot_path_idx]),
                path_len_diff = next_path_len_parameter - prev_path_len_parameter;
            Point2f prev_shift = prev_reference_pt - prev_intersection_pt, 
                    next_shift = next_reference_pt - next_intersection_pt; 
            cout << "start idx: " << start_pivot_path_idx << " end idx: " << end_pivot_path_idx << '\n'
                 <<  "previous and next shift: " << prev_shift << " " << next_shift << "\n";                 
            for (int j = start_pivot_path_idx; j <= end_pivot_path_idx; j++) {
                float cur_path_len_parameter = cur_accumulated_path_length[j], 
                    cur_ratio = (cur_path_len_parameter - prev_path_len_parameter) / path_len_diff;
                Point2f reposition_pt = res_path_set[pt_idx][j] + cur_ratio *  next_shift + (1 - cur_ratio) * prev_shift;
                res_path_set[pt_idx][j] = reposition_pt;
            }
        }
        // cout << "path node num: " << res_path_set[pt_idx].size() << '\n';
    }
    return res_path_set;
}

#endif