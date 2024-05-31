#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp> // Rodrigues()

using namespace cv;

const int max_value_H = 180;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object & Aruco Marker Detection";
int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;

Mat camera_matrix = (Mat_<double>(3,3) <<   517.3257, 0, 310.8539,
                                            0, 517.4208, 236.6973,
                                            0, 0, 1);
Mat dist_coeffs =  (Mat_<double>(1, 4) << 0.0868, -0.2677, 0, 0);
Mat marker_to_base_rot = (Mat_<double>(3,3) <<  -1, 0, 0,
                                                0, -1, 0,
                                                0, 0, 1);


static void on_low_H_thresh_trackbar(int, void *) {
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *) {
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *) {
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *) {
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *) {
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *) {
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", window_detection_name, high_V);
}

int main(int argc, char* argv[]) {
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
    
    cv::Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_50);
    
    Mat cam_frame, frame_HSV, frame_threshold, cam_to_base;
    VideoCapture cam((argc > 1 ? atoi(argv[1]) : 0));
    if (!cam.isOpened()) {
        std::cerr << "Unable to open the camera!\n";
        return -1; 
    }

    int counter = 0;
    bool marker_detelected = false;
    std::vector<double> pre_cam_to_base_items(9, 0);
    std::vector<int> pre_HSV_thresholds{low_H, low_S, low_V, high_H, high_S, high_V};
    while (true) {
        cam >> cam_frame;
        if(cam_frame.empty()) {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(cam_frame, frame_HSV, COLOR_BGR2HSV);
        inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);

        std::vector<int> marker_ids;
        std::vector<std::vector<Point2f>> marker_corners;
        aruco::detectMarkers(cam_frame, dictionary, marker_corners, marker_ids);

        if (marker_ids.size() > 0) {
            marker_detelected = true;
            aruco::drawDetectedMarkers(cam_frame, marker_corners, marker_ids);
            std::vector<Vec3d> rvecs, tvecs;
            Mat rot_mat;
            aruco::estimatePoseSingleMarkers(marker_corners, 0.1, camera_matrix, dist_coeffs, rvecs, tvecs);

            for (int i = 0; i < marker_ids.size(); i++){
                aruco::drawAxis(cam_frame, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], 0.1);
                Rodrigues(rvecs[i], rot_mat);
                cam_to_base = marker_to_base_rot * rot_mat.t();
                std::cout   << "\nrotation vectors (angle-axis):\n" << rvecs[i]
                            << "\nmarker_to_camera rotation:\n" << rot_mat
                            << "\nmarker_to_base rotation:\n" << marker_to_base_rot
                            << "\ncamera_to_base rotation:\n" << cam_to_base << "\n";
            }
        }

        // log data every 40 iterations, approximately 1 s
        if (counter == 40) {
            counter = 0;
            std::vector<double> cam_to_base_items;
            std::vector<int> HSV_thresholds{low_H, low_S, low_V, high_H, high_S, high_V};
            std::ofstream save_file;

            if (marker_detelected) {
                marker_detelected = false;
                for (int row = 0; row < 3; row++)
                    for (int col = 0; col < 3; col++)
                        cam_to_base_items.push_back(cam_to_base.at<double>(row, col));
                
                double variance = 0;
                for (int i = 0; i < cam_to_base_items.size(); i++)
                    variance += abs(cam_to_base_items[i] - pre_cam_to_base_items[i]);
                pre_cam_to_base_items = cam_to_base_items;

                if (variance > 0.01) {
                    save_file.open("./src/path_set_planning/src/data/camera_extrinsic_matrix.txt", std::ios::trunc);
                    if (save_file.is_open() == false) {
                        std::cerr << "Can not open the save file!\n";
                        return -2;
                    }
                    for (float num : cam_to_base_items)
                        save_file << num << " ";
                    save_file.close();
                }
            }

            if (HSV_thresholds != pre_HSV_thresholds) {
                pre_HSV_thresholds = HSV_thresholds;
                save_file.open("./src/path_set_planning/src/data/HSV_thresholds.txt", std::ios::trunc);
                if (save_file.is_open() == false) {
                    std::cerr << "Can not open the save file!\n";
                    return -2;
                }
                for (int num : HSV_thresholds)
                    save_file << num << " ";
                save_file.close();
            }
        }

        counter++;
        imshow(window_capture_name, cam_frame);
        imshow(window_detection_name, frame_threshold);
        char key = (char) waitKey(25);
        if (key == 'q' || key == 27) {
            break;
        }
    }
    return 0;
}