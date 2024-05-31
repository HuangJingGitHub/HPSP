#include "path_processing.hpp"

void DrawPath(Mat img, const vector<Point2f>& path, 
            Scalar color = Scalar(255, 0, 0), 
            int thickness = 2,
            Point2f shift = Point2f(0, 0)) {
    for (int i = 0; i < path.size() - 1; i++)
        line(img, path[i] + shift, path[i + 1] + shift, color, thickness);
}

void GetRotatedVertices(vector<Point2f>& vertices) {
    if (vertices.size() < 4)
        return;
    float width = vertices[1].x - vertices[0].x, 
          height = vertices[2].y - vertices[1].y;
    Point2f centroid = Point2f(0, 0);
    for (int i = 0; i < 4; i++)
        centroid += vertices[i];
    centroid /= 4;
    float rand_rotate_angle = rand() % 30;
    if (rand_rotate_angle < 15)
        rand_rotate_angle = 180 - rand_rotate_angle;

    RotatedRect rRect = RotatedRect(centroid, Size2f(width, height), rand_rotate_angle);
    Point2f vertices7_array[4];
    rRect.points(vertices7_array);
    vertices.clear();
    vertices = vector<Point2f>(vertices7_array, vertices7_array + 4);
}


int main(int argc, char** argv) {
    srand(time(NULL));
    Mat backImg(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Mat backImg_temp(Size(640, 480), CV_64FC3, Scalar(255, 255, 255));
    Point2f start = Point2f(100, 110), end = Point2f(540, 110);
    // 8 pt
/*     vector<Point2f> initial_feedback_pts{Point2f(100, 30), Point2f(100, 60), Point2f(100, 90), Point2f(90, 130),
                                        Point2f(100, 170), Point2f(90, 210), Point2f(100, 250), Point2f(90, 290)}, 
                    target_feedback_pts{Point2f(540, 80), Point2f(550, 100), Point2f(540, 120), Point2f(550, 140),
                                        Point2f(540, 160), Point2f(550, 180), Point2f(540, 200), Point2f(550, 220)};
    int pivot_idx = 2;
    vector<Point2f> vertices1{Point2f(150, 0), Point2f(190, 0), Point2f(190, 100), Point2f(150, 100)},    
                    vertices2{Point2f(180, 150), Point2f(220, 150), Point2f(220, 200), Point2f(180, 200)},      
                    vertices3{Point2f(150, 320), Point2f(190, 320), Point2f(190, 480), Point2f(150, 480)},
                    vertices4{Point2f(400, 0), Point2f(440, 0), Point2f(440, 100), Point2f(400, 100)},          
                    vertices5{Point2f(430, 210), Point2f(470, 210), Point2f(470, 260), Point2f(430, 260)},
                    vertices6{Point2f(430, 320), Point2f(470, 320), Point2f(470, 480), Point2f(430, 480)},
                    vertices7{Point2f(280, 360), Point2f(370, 360), Point2f(370, 400), Point2f(280, 400)};  */
    vector<Point2f> initial_feedback_pts{Point2f(100, 50), Point2f(100, 90), Point2f(100, 140), Point2f(90, 180)}, 
                    target_feedback_pts{Point2f(540, 100), Point2f(540, 130), Point2f(540, 190), Point2f(540, 250)};
    int pivot_idx = 1;
    vector<Point2f> vertices1{Point2f(200, 0), Point2f(240, 0), Point2f(240, 100), Point2f(200, 100)},    
                    vertices2{Point2f(180, 150), Point2f(220, 150), Point2f(220, 200), Point2f(180, 200)},      
                    vertices3{Point2f(180, 320), Point2f(220, 320), Point2f(220, 480), Point2f(180, 480)},
                    vertices4{Point2f(420, 0), Point2f(460, 0), Point2f(460, 100), Point2f(420, 100)},          
                    vertices5{Point2f(400, 210), Point2f(440, 210), Point2f(440, 260), Point2f(400, 260)},
                    vertices6{Point2f(440, 320), Point2f(480, 320), Point2f(480, 480), Point2f(440, 480)},
                    vertices7{Point2f(280, 360), Point2f(370, 360), Point2f(370, 400), Point2f(280, 400)};                     

    GetRotatedVertices(vertices1);
    GetRotatedVertices(vertices2);
    GetRotatedVertices(vertices3);
    GetRotatedVertices(vertices4);
    GetRotatedVertices(vertices5);
    GetRotatedVertices(vertices6);
    GetRotatedVertices(vertices7);

    PolygonObstacle obs1(vertices1), obs2(vertices2), obs3(vertices3), obs4(vertices4), obs5(vertices5), obs6(vertices6), obs7(vertices7);
    vector<PolygonObstacle> obstacles{obs1, obs2, obs3, obs4, obs5, obs6, obs7};
    vector<Point2f> obs_centroids = GetObstaclesCentroids(obstacles);

    RRTStarPlanner RRTStar_planer(start, end, obstacles, 20, 20, 5, Size2f(640, 480));
    bool planned = RRTStar_planer.Plan(backImg, 10, true);

    vector<vector<Point2f>> path_set = GeneratePathSetInGeneralCondition(initial_feedback_pts, target_feedback_pts, 0, 15, obstacles, backImg);

/*     for (vector<int>& passage : passage_passed) {
        cout << "Passage pair: " << passage[0] << "---" << passage[1] << '\n';
        line(backImg, obs_centroids[passage[0]], obs_centroids[passage[1]], Scalar(0, 0, 255), 2);
    }  */
    cout << backImg.size[0] << " x " << backImg.size[1] << endl;           
    for (int i = 0; i < path_set.size(); i++) {
        if (i == pivot_idx) 
            DrawPath(backImg, path_set[i], Scalar(0, 255, 0));
        else 
            DrawPath(backImg, path_set[i], Scalar(255, 0, 0));
        circle(backImg, path_set[i].front(), 4, Scalar(0, 0, 0), -1); 
        circle(backImg, path_set[i].back(), 4, Scalar(0, 0, 0), -1);             
    } 
    for (int i = 0; i < obstacles.size(); i++) {
        if (i > -1) {
            for (int j = 0; j < 4; j++)
                line(backImg, obstacles[i].vertices[j], obstacles[i].vertices[(j + 1) % 4], Scalar(0, 0, 0), 2);
            putText(backImg, std::to_string(i + 1), obs_centroids[i] + Point2f(-10, 12), FONT_ITALIC, 1.2, Scalar(0, 0, 0), 2);
        }
        else {
            rectangle(backImg, obstacles[i].vertices[0], obstacles[i].vertices[2], Scalar(0, 0, 0), 2);
            putText(backImg, std::to_string(i + 1), obs_centroids[i] + Point2f(-10, 10), FONT_ITALIC, 1.2, Scalar(0, 0, 0), 2);
        }
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