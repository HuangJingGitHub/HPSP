#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

Mat camera_matrix = (Mat_<double>(3,3) <<   517.3257, 0, 310.8539,
                                            0, 517.4208, 236.6973,
                                            0, 0, 1);
Mat dist_coeffs =  (Mat_<double>(1, 4) << 0.0868, -0.2677, 0, 0);

class Webcam {
public:
    Webcam(int cam_idx) {
        VideoCapture monocular_cam(cam_idx);
        if (!monocular_cam.isOpened()) {
            cout <<  "Cannot open the webcam by index " << cam_idx << endl;
            return;
        }
        double img_width  = monocular_cam.get(cv::CAP_PROP_FRAME_WIDTH); 
        double img_height = monocular_cam.get(cv::CAP_PROP_FRAME_HEIGHT); 
        monocular_cam.set(CAP_PROP_FPS, 30);
        int FPS = monocular_cam.get(CAP_PROP_FPS);
        cout << "This is a 2D view.\n"
             << "Frame size of camera: " << img_width << " x " << img_height << '\n'
             << "FPS of monocamera: "  << FPS << '\n';

        ros::NodeHandle node_handle;
        image_transport::ImageTransport image_transport(node_handle);
        image_transport::Publisher publisher = image_transport.advertise("cameras/source_camera/image", 50);
        sensor_msgs::ImagePtr msg_image_ptr;
        ros::Rate publish_rate(30);

        Mat cam_frame, undistorted_frame;
        namedWindow("Source Video Stream", WINDOW_AUTOSIZE);
        // namedWindow("Undistorted Image", WINDOW_AUTOSIZE);
        while (true) {
            bool read_success = monocular_cam.read(cam_frame);   
            if (!read_success) {
                cout << "ERROR: Cannot read a frame from the video stream!\n";
                break;
            }
            undistort(cam_frame, undistorted_frame, camera_matrix, dist_coeffs);
            // imshow("Source Video Stream", cam_frame);
            // imshow("Undistorted Image", undistorted_cam_frame);
            // waitKey(2);

            msg_image_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", undistorted_frame).toImageMsg();
            if (ros::ok()) {
                publisher.publish(msg_image_ptr);
                publish_rate.sleep();
            }
            else {
                monocular_cam.release();
                cout << "\nTerminated by user.\n";
                break;
            }         
        }
    }

    ~Webcam() {
        destroyWindow("Source Video Stream"); 
        // destoryWindow("Undistorted Image");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Source_Camera");
    if (argc == 1) {
        Webcam webcam_object(0);
    }
    else if (argc == 2) {
        int cam_idx = atoi(argv[1]);
        Webcam webcam_object(cam_idx);
    }
    else {
        ROS_INFO("ERROR: Unrecognized arguements");
        return 1;
    }
}