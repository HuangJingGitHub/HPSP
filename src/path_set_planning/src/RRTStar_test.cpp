#include "path_processing.hpp"

void DrawPath(Mat img, const vector<Point2f>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, path[i] + shift, path[i + 1] + shift, color, thickness);
}

int main(int argc, char** argv) {
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 110), end = Point2f(540, 110);
    vector<Point2f> initial_feedback_pts{start, Point2f(100, 40)}, target_feedback_pts{end, Point2f(540, 40)};
    int pivot_idx = 0;
    vector<Point2f> vertices1{Point2f(200, 0), Point2f(240, 0), Point2f(240, 100), Point2f(200, 100)},    
                    vertices2{Point2f(200, 150), Point2f(240, 150), Point2f(240, 200), Point2f(200, 200)},      
                    vertices3{Point2f(200, 320), Point2f(240, 320), Point2f(240, 480), Point2f(200, 480)},
                    vertices4{Point2f(400, 0), Point2f(440, 0), Point2f(440, 100), Point2f(400, 100)},          
                    vertices5{Point2f(400, 185), Point2f(440, 185), Point2f(440, 235), Point2f(400, 235)},
                    vertices6{Point2f(400, 320), Point2f(440, 320), Point2f(440, 480), Point2f(400, 480)};
    PolygonObstacle obs1(vertices1), obs2(vertices2), obs3(vertices3), obs4(vertices4), obs5(vertices5), obs6(vertices6);
    vector<PolygonObstacle> obstacles{obs1, obs2, obs3, obs4, obs5, obs6};
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    RRTStarPlanner RRTStar_planer(start, end, obstacles, 15, 20, 5, Size2f(640, 480));
    bool planned = RRTStar_planer.Plan(backImg, 10, true);
    vector<RRTStarNode*> path = RRTStar_planer.GetPath(), sparse_path;
    vector<Point2f> smooth_path = QuadraticBSplineSmoothing(path);

    if (planned) {
        for (int i = 0; i < path.size(); i += 2)
            sparse_path.push_back(path[i]);
        if (sparse_path.back() != path.back())
            sparse_path.push_back(path.back());
        smooth_path = QuadraticBSplineSmoothing(sparse_path);
    }

    DrawPath(backImg, smooth_path, Scalar(0, 255, 0));
    circle(backImg, initial_feedback_pts[0], 4, Scalar(0, 0, 0), -1);
    circle(backImg, target_feedback_pts[0], 4, Scalar(0, 0, 0), -1); 
    circle(backImg, initial_feedback_pts[1], 4, Scalar(0, 0, 0), -1);
    circle(backImg, target_feedback_pts[1], 4, Scalar(0, 0, 0), -1); 


    vector<Point2f> smooth_path_copy = smooth_path;
    // std::cout << "Raw path node num: " << path.size() << ". Smoothed path node num: " << smooth_path.size() << "\n";
    vector<vector<int>> passage_passed = GetPassagesPathPasses(obstacles, path);  
    vector<vector<int>> intersection_idx; 
    vector<vector<Point2f>> passage_intersection_pts = GetPassageIntersectionsOfPathSet(obstacles, passage_passed, smooth_path, intersection_idx, initial_feedback_pts, pivot_idx);
    vector<Point2f> reposition_points = GetPivotPathRepositionPts(obstacles, passage_passed, passage_intersection_pts, pivot_idx);
    DeformPath(smooth_path, passage_intersection_pts[pivot_idx], reposition_points, intersection_idx[pivot_idx]); 

    // Deform transferred paths---> Write a function for it
    passage_intersection_pts = GetPassageIntersectionsOfPathSet(obstacles, passage_passed, smooth_path, intersection_idx, initial_feedback_pts, pivot_idx);
    vector<vector<Point2f>> adjusted_passage_intersection_pts = AdjustRepositionPtsForPathSet(obstacles, passage_passed, passage_intersection_pts, pivot_idx);
    
    vector<vector<Point2f>> path_set(initial_feedback_pts.size(), smooth_path);
    for (int i = 0; i < initial_feedback_pts.size(); i++) {
        if (i == pivot_idx) 
            continue;
        for (Point2f& path_node : path_set[i])
            path_node += initial_feedback_pts[i] - initial_feedback_pts[pivot_idx];
        DeformPath(path_set[i], passage_intersection_pts[i], adjusted_passage_intersection_pts[i], intersection_idx[i]);
    }

/*     for (vector<int>& passage : passage_passed) {
        cout << "Passage pair: " << passage[0] << "---" << passage[1] << '\n';
        line(backImg, obs_centroids[passage[0]], obs_centroids[passage[1]], Scalar(0, 0, 255), 2);
    }  */

    for (int i = 0; i < path_set.size(); i++) {
        if (i == pivot_idx) 
            for (int j = 0; j < path_set[i].size() - 1; j++)
                line(backImg, path_set[i][j], path_set[i][j + 1], Scalar(0, 255, 0), 2);
        else 
            for (int j = 0; j < path_set[i].size() - 1; j++)
                line(backImg, path_set[i][j], path_set[i][j + 1], Scalar(255, 0, 0), 2);
    }
    for (int i = 0; i < smooth_path_copy.size() - 1; i++)
        line(backImg, smooth_path_copy[i], smooth_path_copy[i + 1], Scalar(0, 255, 0), 2);
    for (int i  = 0; i < smooth_path_copy.size(); i += 15) 
       line(backImg, smooth_path_copy[i], smooth_path[i], Scalar(0, 0, 255), 2);  
    
    DrawPath(backImg, smooth_path, Scalar(0, 255, 0));
    circle(backImg, smooth_path.front(), 4, Scalar(0, 0, 0), -1);
    circle(backImg, smooth_path.back(), 4, Scalar(0, 0, 0), -1); 

    for (int i = 0; i < obstacles.size(); i++) {
        rectangle(backImg, obstacles[i].vertices[0], obstacles[i].vertices[2], Scalar(0, 0, 0), 2);
        putText(backImg, std::to_string(i + 1), obs_centroids[i] + Point2f(-10, 5), FONT_ITALIC, 1.2, Scalar(0, 0, 0), 2);
    } 

/*     rectangle(backImg, vertices1[0], vertices1[2], Scalar(0, 0, 0), 2);
    putText(backImg, "1", Point(250, 50), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);
    rectangle(backImg, vertices2[0], vertices2[2], Scalar(0, 0, 0), 2);
    putText(backImg, "2", Point(250, 180), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);  
    rectangle(backImg, vertices3[0], vertices3[2], Scalar(0, 0, 0), 2);
    putText(backImg, "3", Point(250, 400), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2);
    rectangle(backImg, vertices4[0], vertices4[2], Scalar(0, 0, 0), 2);
    putText(backImg, "4", Point(370, 50), FONT_ITALIC, 1.2, Scalar(255, 0, 0), 2); */
    // namedWindow("RRT* path planning", WINDOW_AUTOSIZE);
    imshow("RRT* path planning", backImg);
    waitKey(0); 
    return 0;
}