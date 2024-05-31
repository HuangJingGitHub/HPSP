#ifndef OBSTACLES_INCLUDED
#define OBSTACLES_INCLUDED

#include <math.h>
#include <iostream>
#include <vector>
#include <set>
#include <algorithm>
#include <random>
#include <ctime>
// #include <tuple>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/Dense>

using namespace cv;
using namespace std;

float normSqr(const Point2f& pt) {
    return pt.x * pt.x + pt.y * pt.y;
}

struct PolygonObstacle {
    bool closed;
    vector<Point2f> vertices;
    Point2f min_distance_pt;
    PolygonObstacle(): closed(true) {};
    PolygonObstacle(vector<Point2f> polygon_vertices, bool is_close = true): closed(is_close), 
                                        vertices(polygon_vertices) {}
    PolygonObstacle(vector<Point> polygon_vertices, bool is_close = true) {
        closed = is_close;
        vertices = vector<Point2f>(polygon_vertices.size(), Point2f(0, 0));
        for (int i = 0; i < polygon_vertices.size(); i++) {
            vertices[i].x = (float) polygon_vertices[i].x;
            vertices[i].y = (float) polygon_vertices[i].y;
        }
            
    }                                                              
};

int Orientation(const Point2f& p1, const Point2f& p2, const Point2f& p3) {
    float threshold = 1e-5;
    float vec1_x = p2.x - p1.x, vec1_y = p2.y - p1.y,
          vec2_x = p3.x - p1.x, vec2_y = p3.y - p1.y;
    float cross_product = vec1_x * vec2_y - vec1_y * vec2_x;
    if (cross_product > threshold)  // > 0
        return 1;   // From segment p1-p2 to p1-p3, counterclockwise direction, CCW
    else if (cross_product < -threshold) // < 0
        return 2;    // clockwise direction
    else
        return 0;  
}

bool OnSegment(const Point2f& p1, const Point2f& p2, const Point2f& q) {  
// check if point q is on the segment p1-p2 when the three points are colinear
/*     if (q.x <= max(p1.x, p2.x) && q.x >= min(p1.x, p2.x)
        && q.y <= max(p1.y, p2.y) && q.y >= min(p1.y, p2.y))
            return true;
    return false; */
    Point2f vec_1 = p1 - q, vec_2 = p2 - q;
    float inner_product = vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    return inner_product <= 0;
}

bool SegmentIntersection(const Point2f& p1, const Point2f& p2, const Point2f& q1, const Point2f& q2) {
    int ori1 = Orientation(p1, p2, q1),  // segments: p1-p2, q1-q2
        ori2 = Orientation(p1, p2, q2),
        ori3 = Orientation(q1, q2, p1),
        ori4 = Orientation(q1, q2, p2);
    
    if (ori1 != ori2 && ori3 != ori4)
        return true;
    else if (ori1 == 0 && OnSegment(p1, p2, q1))
        return true;
    else if (ori2 == 0 && OnSegment(p1, p2, q2))
        return true;
    else if (ori3 == 0 && OnSegment(q1, q2, p1))
        return true;
    else if (ori4 == 0 && OnSegment(q1, q2, p2))
        return true;
    return false;
}

Point2f ClosestPtOnSegmentToPt(const Point2f& p1, const Point2f& p2, const Point2f& test_pt) {
    // Return the point on segment p1-p2 closest to test_pt
    Point2f p_1_to_test_pt_vec = test_pt - p1,
            segment_vec = p2 - p1,
            res_pt;
    float inner_product = p_1_to_test_pt_vec.x * segment_vec.x + p_1_to_test_pt_vec.y * segment_vec.y;
    float segment_len_square = segment_vec.x * segment_vec.x + segment_vec.y * segment_vec.y;
    float projection_segment_len_ratio = inner_product / segment_len_square;
    if (projection_segment_len_ratio < 0)
        res_pt = p1;
    else if (projection_segment_len_ratio > 1)
        res_pt = p2;
    else 
        res_pt = p1 + projection_segment_len_ratio * segment_vec;
    return res_pt;
}



Point2f GetSegmentsIntersectionPt(const Point2f& p1, const Point2f& p2, const Point2f& q1, const Point2f& q2) {
 // Segment p1-p2, q1-q2 intersect, otherwise will return intersection point of two lines.
    if (abs(p1.x - p2.x) < 1e-4 && abs(q1.x - q2.x) > 1e-4)
        return Point2f(p1.x, q1.y + (q1.y - q2.y) / (q1.x - q2.x) * (p1.x - q1.x));
    else if (abs(p1.x - p2.x) > 1e-4 && abs(q1.x - q2.x) < 1e-4)
        return Point2f(q1.x, p1.y + (p1.y - p2.y) / (p1.x - p2.x) * (q1.x - p1.x));
    else if (abs(p1.x - p2.x) < 1e-4 && abs(q1.x - q2.x) < 1e-4)
        return Point2f(0, 0);        
    
    float kp = (p2.y - p1.y) / (p2.x - p1.x),
          kq = (q2.y - q1.y) / (q2.x - q1.x),
          x_p1 = p1.x, y_p1 = p1.y,
          x_q1 = q1.x, y_q1 = q1.y;
    float x = (y_p1 - y_q1 - kp * x_p1 + kq * x_q1) / (kq - kp),
          y = y_p1 + kp * (x - x_p1);
    return Point2f(x, y);
}

vector<Point2f> GetEndsOfColinearPts(const vector<Point2f>& points) {
    if (points.size() == 2)
        return points;
    else if (points.size() == 1)
        return {points[0], points[0]};
    else if (points.size() == 0) {
        cout << "Warning: An empty point set is input for ends finding of colinear points\n";
        return {};
    }
    
    float x_min = points[0].x, x_max = x_min;
    for (int i = 1; i < points.size(); i++) {
        x_min = min(x_min, points[i].x);
        x_max = min(x_max, points[i].x);
    }

    vector<Point2f> res;
    if (x_max - x_min > 1e-2) {
        Point2f min_x_pt  = points[0], max_x_pt = points[0];
        for (int i = 1; i < points.size(); i++) {
            if (points[i].x < min_x_pt.x)
                min_x_pt = points[i];
            if (points[i].x > max_x_pt.x)
                max_x_pt = points[i];
        }
        res = vector<Point2f>{min_x_pt, max_x_pt};
    }
    else {
        Point2f min_y_pt  = points[0], max_y_pt = points[0];
        for (int i = 1; i < points.size(); i++) {
            if (points[i].y < min_y_pt.y)
                min_y_pt = points[i];
            if (points[i].y > max_y_pt.y)
                max_y_pt = points[i];
        }
        res = vector<Point2f>{min_y_pt, max_y_pt};
    }
    return res;
}

vector<Point2f> GetObstaclesCentroids(const vector<PolygonObstacle>& obstacles) {
    vector<Point2f> obs_centroids(obstacles.size());
    for (int i = 0; i < obstacles.size(); i++) {
        obs_centroids[i] = Point2f(0, 0);
        for (const Point2f& vertex : obstacles[i].vertices) 
            obs_centroids[i] += vertex;
        obs_centroids[i] /= (float) obstacles[i].vertices.size();
    } 
    return obs_centroids;   
}

bool ObstacleFree(const PolygonObstacle& obs, Point2f p1, Point2f p2) {
    if (obs.vertices.size() <= 1)
        return true;
    
    for (int i = 0; i < obs.vertices.size() - 1; i++) {
        if (SegmentIntersection(obs.vertices[i], obs.vertices[i + 1], p1, p2))
            return false;
    }
    if (obs.closed)
        if (SegmentIntersection(obs.vertices[0], obs.vertices.back(), p1, p2))
            return false;
    return true;
}

bool obstacleFreeVec(vector<PolygonObstacle>& obs, Point2f p1, Point2f p2) {
    for (PolygonObstacle& obstacle : obs)
        if (!ObstacleFree(obstacle, p1, p2))
            return false;
    return true;
}

bool ObstacleFreeVecForPassage(const vector<PolygonObstacle>& obs, int obs_idx_1, int obs_idx_2) {
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obs);
    Point2f obs_centroid_1 = obs_centroids[obs_idx_1],
            obs_centroid_2 = obs_centroids[obs_idx_2];
    for (int i = 0; i < obs.size(); i++) {
        if (i == obs_idx_1 || i == obs_idx_2)
            continue;
        if (!ObstacleFree(obs[i], obs_centroid_1, obs_centroid_2))
            return false;
    }
    return true;
}

bool AreObstaclesOverlapped(const PolygonObstacle& obs1, const PolygonObstacle& obs2) {
    int vertex_num_1 = obs1.vertices.size();
    for (int i = 0; i < vertex_num_1; i++) 
        if (ObstacleFree(obs2, obs1.vertices[i], obs1.vertices[(i + 1) % vertex_num_1]) == false)
            return true;    
    return false;
}

vector<Point2f> GetPassageInnerEnds(PolygonObstacle obs1, PolygonObstacle obs2) {
    Point2f obs_centroid_1 = GetObstaclesCentroids({obs1})[0],
            obs_centroid_2 = GetObstaclesCentroids({obs2})[0],
            inner_end_1, inner_end_2;

    if (obs1.closed)
        obs1.vertices.push_back(obs1.vertices.front());
    for (int i = 0; i < obs1.vertices.size() - 1; i++) {
        if (SegmentIntersection(obs1.vertices[i], obs1.vertices[i + 1], obs_centroid_1, obs_centroid_2)) {
            inner_end_1 = GetSegmentsIntersectionPt(obs1.vertices[i], obs1.vertices[i + 1], obs_centroid_1, obs_centroid_2);
            break;
        }
    }
    if (obs1.closed)
        obs1.vertices.pop_back();
    
    if (obs2.closed)
        obs2.vertices.push_back(obs2.vertices.front());
    for (int i = 0; i < obs2.vertices.size() - 1; i++) {
        if (SegmentIntersection(obs2.vertices[i], obs2.vertices[i + 1], obs_centroid_1, obs_centroid_2)) {
            inner_end_2 = GetSegmentsIntersectionPt(obs2.vertices[i], obs2.vertices[i + 1], obs_centroid_1, obs_centroid_2);
            break;
        }
    }
    if (obs2.closed)
        obs2.vertices.pop_back();

    return {inner_end_1, inner_end_2};
}

vector<Point2f> GetPassageSegmentPts(const PolygonObstacle& obs1, const PolygonObstacle& obs2) {
    int vertices_num_1 = obs1.vertices.size(), vertices_num_2 = obs2.vertices.size();
    float min_dist = FLT_MAX;
    Point2f res_pt_1, res_pt_2;
    for (int i = 0; i < vertices_num_1; i++)
        for (int j = 0; j < vertices_num_2; j++) {
            Point2f cur_pt_2 = ClosestPtOnSegmentToPt(obs2.vertices[j], obs2.vertices[(j + 1) % vertices_num_2], obs1.vertices[i]);
            float cur_dist = cv::norm(obs1.vertices[i] - cur_pt_2); 
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                res_pt_1 = obs1.vertices[i];
                res_pt_2 = cur_pt_2;
            }
        }
    for (int i = 0; i < vertices_num_2; i++)
        for (int j = 0; j < vertices_num_1; j++) {
            Point2f cur_pt_1 = ClosestPtOnSegmentToPt(obs1.vertices[j], obs1.vertices[(j + 1) % vertices_num_1], obs2.vertices[i]);
             float cur_dist = cv::norm(obs2.vertices[i] - cur_pt_1); 
            if (cur_dist < min_dist) {
                min_dist = cur_dist;
                res_pt_1 = cur_pt_1;
                res_pt_2 = obs2.vertices[i];
            }           
        }
    return {res_pt_1, res_pt_2};
}

Point2f GetClosestIntersectionPt(PolygonObstacle& obs, Point2f p1, Point2f p2, Point2f testPt) {
    Point2f res = Point2f(0, 0);
    if (obs.vertices.size() <= 1) {
        cout << "Obstacle vertex number less than 1.\n";
        return res;
    }

    if (obs.closed)
        obs.vertices.push_back(obs.vertices.front());

    float min_distance_sqr = FLT_MAX;
    for (int i = 0; i < obs.vertices.size() - 1; i++) {
        if (SegmentIntersection(p1, p2, obs.vertices[i], obs.vertices[i + 1])) {
            Point2f cur_intersection_pt = GetSegmentsIntersectionPt(p1, p2, obs.vertices[i], obs.vertices[i + 1]);
            if (normSqr(cur_intersection_pt - testPt) < min_distance_sqr) {
                res = cur_intersection_pt;
                min_distance_sqr = normSqr(cur_intersection_pt - testPt);
            } 
        }
    }
    
    if (obs.closed)
        obs.vertices.pop_back();
    return res;
} 

float MinDistanceToObstacle(const PolygonObstacle& obs, Point2f testPt) {
    float res = FLT_MAX;
    int obs_vertices_num = obs.vertices.size();
    for (int i = 0; i < obs_vertices_num; i++) {
        Point2f cur_closest_pt = ClosestPtOnSegmentToPt(obs.vertices[i], obs.vertices[(i + 1) % obs_vertices_num], testPt);
        float cur_distance = cv::norm(testPt - cur_closest_pt);
        res = min(res, cur_distance);
    }
    return res;
}

float MinDistanceToObstaclesVec(const vector<PolygonObstacle>& obstacles, Point2f testPt) {
    float res = FLT_MAX, distance_to_obs;
    for (PolygonObstacle obs : obstacles) {
        distance_to_obs = MinDistanceToObstacle(obs, testPt);
        if (distance_to_obs < res)
            res = distance_to_obs;
    }
    return res;
}

pair<int, Point2f> FindPassageEndonObstacles(const vector<PolygonObstacle>& obstacles, Point2f start_pt, Point2f virtual_end) {
    int res_obs_idx = 0;
    Point2f res_passage_end(-1, -1);
    float min_passage_len = FLT_MAX;
    for (int obs_idx = 0; obs_idx < obstacles.size(); obs_idx++) {
        int obs_vertices_num = obstacles[obs_idx].vertices.size();
        for (int i = 0; i < obs_vertices_num; i++) {
            if (SegmentIntersection(obstacles[obs_idx].vertices[i], 
                                    obstacles[obs_idx].vertices[(i + 1) % obs_vertices_num],
                                    start_pt, virtual_end) == true) {
                Point2f cur_passage_end = GetSegmentsIntersectionPt(obstacles[obs_idx].vertices[i], 
                                                                    obstacles[obs_idx].vertices[(i + 1) % obs_vertices_num],
                                                                    start_pt, virtual_end);
                float cur_passage_len = cv::norm(cur_passage_end - start_pt);
                if (cur_passage_len < min_passage_len) {
                    min_passage_len = cur_passage_len;
                    res_obs_idx = obs_idx;
                    res_passage_end = cur_passage_end;
                }
            }
        }
    }

    return make_pair(res_obs_idx, res_passage_end);
}

float GetMinPassageWidthPassed(const vector<PolygonObstacle>& obstacles, Point2f pt1, Point2f pt2) {
    // Return the min width of passages the segment pt1-pt2 passes. Return -1 if no passage is passed
    // Full search version
    float res = FLT_MAX;
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    for (int i = 0; i < obs_centroids.size() - 1; i++) {
        int j = i < 4 ? 4 : i + 1; // The first four are wall obstacles.
        for (; j < obs_centroids.size(); j++)
            if (SegmentIntersection(obs_centroids[i], obs_centroids[j], pt1, pt2)) {
                // vector<Point2f> passage_inner_ends = GetPassageInnerEnds(obstacles[i], obstacles[j]);
                vector<Point2f> passage_inner_ends = GetPassageSegmentPts(obstacles[i], obstacles[j]);
                float cur_passage_width  = cv::norm(passage_inner_ends[0] - passage_inner_ends[1]);
                res = min(res, cur_passage_width); 
            }
    }
    
    if (res == FLT_MAX)
        return -1.0;
    return res;
}

float GetMinPassageWidthPassed(const vector<vector<Point2f>>& passage_pts, Point2f pt1, Point2f pt2) {
    float res = FLT_MAX;

    for (int i = 0; i < passage_pts.size(); i++) {
        if (SegmentIntersection(passage_pts[i][0], passage_pts[i][1], pt1, pt2))
            res = min(res, (float)cv::norm(passage_pts[i][0] - passage_pts[i][1]));
    }
    
    if (res > FLT_MAX - 1)
        return -1.0;
    return res;
}

vector<PolygonObstacle> GenerateRandomObstacles(int obstacle_num, Size2f config_size = Size2f(640, 480), 
                                                float size_len = 30) {
    if (obstacle_num < 0) {
        cout << "The number of obstacles to be geenrated should be nonnegative.\n";
        return {};
    }

    // By default, add environment 4 walls as obstacles
    vector<PolygonObstacle> res_obs_vec(obstacle_num + 4);
    vector<Point2f> top_obs_vertices{Point2f(0, 0), Point2f(config_size.width, 0), Point2f(10, -10)},
                    bottom_obs_vertices{Point2f(0, config_size.height), Point2f(config_size.width, config_size.height), 
                               Point2f(10, config_size.height + 10)},
                    left_obs_vertices{Point2f(0, 0), Point2f(0, config_size.height), Point2f(-10, 10)},
                    right_obs_vertices{Point2f(config_size.width, 0), Point2f(config_size.width, config_size.height), 
                              Point2f(config_size.width + 10, 10)};
    PolygonObstacle top_obs(top_obs_vertices), bottom_obs(bottom_obs_vertices), 
                    left_obs(left_obs_vertices), right_obs(right_obs_vertices);
    res_obs_vec[0] = top_obs;
    res_obs_vec[1] = bottom_obs;
    res_obs_vec[2] = left_obs;
    res_obs_vec[3] = right_obs;
    
    //float size_len = 30;
    Eigen::Matrix<float, 2, 4> vertices_square, vertices_rectangle; 
                               vertices_square << -size_len / 2, size_len / 2, size_len / 2, -size_len / 2,
                                                 -size_len / 2, -size_len / 2, size_len / 2, size_len / 2;
                               vertices_rectangle << -size_len / 2, size_len / 2, size_len / 2, -size_len / 2,
                                                     -size_len / 4, -size_len / 4, size_len / 4, size_len / 4;
    Eigen::Matrix<float, 2, 3> vertices_triangle;                                                     
                               vertices_triangle << -size_len / 2, size_len / 2, 0,
                                                    -size_len * sqrt(3) / 6, -size_len * sqrt(3) / 6, size_len * sqrt(3) / 3;                                                
    
    vector<Eigen::MatrixXf> vertices_vec(3);
    vertices_vec[0] = vertices_square;
    vertices_vec[1] = vertices_rectangle;
    vertices_vec[2] = vertices_triangle;

    vector<Point2f> obs_center(obstacle_num);
    random_device rd_x, rd_y, rd_rotate_angle, rd_shape;
    mt19937 rd_engine_x(rd_x()), rd_engine_y(rd_y()), rd_engine_rotate_angle(rd_rotate_angle()), rd_engine_shape(rd_shape());
    uniform_real_distribution<> distribution_x(0, config_size.width),
                                distribution_y(0, config_size.height),
                                distribution_rotate_angle(0, 2 * M_PI);
    uniform_int_distribution<> distribution_shape(0, 2);

    for (int i = 4; i < obstacle_num + 4; i++) {
        float cur_x = distribution_x(rd_x), cur_y = distribution_y(rd_y), cur_angle = distribution_rotate_angle(rd_rotate_angle);
        int cur_shape_type = distribution_shape(rd_shape);
        Point2f cur_obs_center(cur_x, cur_y);
        Eigen::Matrix2f cur_rotation;
                        cur_rotation << cos(cur_angle), -sin(cur_angle), 
                                        sin(cur_angle), cos(cur_angle);
        Eigen::MatrixXf cur_rotated_vertices = cur_rotation * vertices_vec[cur_shape_type];
        PolygonObstacle cur_obs;
        for (int j = 0; j < cur_rotated_vertices.cols(); j++) {
            Point2f cur_vertex(cur_rotated_vertices(0, j) + cur_obs_center.x,
                               cur_rotated_vertices(1, j) + cur_obs_center.y);
            cur_obs.vertices.push_back(cur_vertex);
        }

        bool is_cur_obs_valid = true;
        for (int j = 0; j < i; j++) {
            if (AreObstaclesOverlapped(cur_obs, res_obs_vec[j]) == true) {
                is_cur_obs_valid = false;
                break;
            }
        }
        if (is_cur_obs_valid == false)
            i--;
        else
            res_obs_vec[i] = cur_obs;
    }

    return res_obs_vec;
}

pair<vector<vector<int>>, vector<vector<Point2f>>> PureVisibilityPassageCheck(const vector<PolygonObstacle>& obstacles) {
    vector<vector<int>> res_passage_pair;
    vector<vector<Point2f>> res_passage_pts;
    
    int start_idx = 0; // 4 if the first four wall obstacles are not considered.
    for (int i = start_idx; i < obstacles.size() - 1; i++) {
        int j = i < 4 ? 4 : i + 1;
        for (; j < obstacles.size(); j++) {
            vector<Point2f> cur_passage_segment_pts = GetPassageSegmentPts(obstacles[i], obstacles[j]);
            
            bool is_passage_segment_obstacle_free = true;
            for (int k = start_idx; k < obstacles.size(); k++) {
                if (k == i || k == j)
                    continue;
                if (ObstacleFree(obstacles[k], cur_passage_segment_pts[0], cur_passage_segment_pts[1]) == false) {
                    is_passage_segment_obstacle_free = false;
                    break;
                }
            }
            if (is_passage_segment_obstacle_free == true) {
                res_passage_pair.push_back({i, j});
                res_passage_pts.push_back(cur_passage_segment_pts);
            }
        }
    }
    return make_pair(res_passage_pair, res_passage_pts);
}

pair<vector<vector<int>>, vector<vector<Point2f>>> ExtendedVisibilityPassageCheck(const vector<PolygonObstacle>& obstacles) {
    vector<vector<int>> res_passage_pair;
    vector<vector<Point2f>> res_passage_pts;
    
    int start_idx = 0; // 4 if the first four wall obstacles are not considered.
    for (int i = start_idx; i < obstacles.size() - 1; i++) {
        int j = i < 4 ? 4 : i + 1;
        for (; j < obstacles.size(); j++) {
            vector<Point2f> cur_passage_segment_pts = GetPassageSegmentPts(obstacles[i], obstacles[j]);
            float cur_passage_length = cv::norm(cur_passage_segment_pts[0] - cur_passage_segment_pts[1]);

            bool is_passage_valid = true;
            for (int k = start_idx; k < obstacles.size(); k++) {
                if (k == i || k == j)
                    continue;
                if (ObstacleFree(obstacles[k], cur_passage_segment_pts[0], cur_passage_segment_pts[1]) == false) {
                    is_passage_valid = false;
                    break;
                }
                Point2f cur_passage_center = (cur_passage_segment_pts[0] + cur_passage_segment_pts[1]) / 2;
                float cur_obs_passage_center_dist = MinDistanceToObstacle(obstacles[k], cur_passage_center);
                if (cur_obs_passage_center_dist <= cur_passage_length / 2) {
                    is_passage_valid = false;
                    break;
                }
            }
            if (is_passage_valid == true) {
                res_passage_pair.push_back({i, j});
                res_passage_pts.push_back(cur_passage_segment_pts);
            }
        }
    }
    return make_pair(res_passage_pair, res_passage_pts);
}

#endif