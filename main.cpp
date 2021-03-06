#include "LandmarkCoreIncludes.h"
//#include "ros/ros.h"

#include <opencv2/sfm.hpp>
#include <Face_utils.h>
#include <FaceAnalyser.h>
#include <GazeEstimation.h>
#include <RecorderOpenFace.h>
#include <RecorderOpenFaceParameters.h>
#include <SequenceCapture.h>
#include <Visualizer.h>
#include <VisualizationUtils.h>

#include <RotationHelpers.h>
#include <kinematic.h>

#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <pthread.h>

// Arduino Communication Settings
#include "serial.h"
#include "sprotocol.h"
#include "head_neck_ctl.h"

#ifndef CONFIG_DIR
#define CONFIG_DIR "~"
#endif

#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl


struct pan_tilt_ctrl pt_ctrl = {

		.pan_left = 1500,
		.tilt_left = 1500,
		.pan_right = 1450,
		.tilt_right = 1500,
		.pan_neck = 1520,
		.tilt_neck = 1450
};
extern struct h2l_state_pan_tilt* state_msg;

int stop_serial_tx = 0;
int stop_serial_rx = 0;
int stop_js = 0;

int period = 100000; // 100 ms

void vergence_servo_ctrl( float *q ){
	
    // PID controller
    float rad2pwm = 1000./PI;
    float kpneck = 0.09;
    float kptilt = 0.01;
    float kpverg = 0.01;
	if (!isnan(q[0]) && !isnan(q[1]) && !isnan(q[2])) {

		float out_neck = kpneck * (1500 + q[0] * rad2pwm - pt_ctrl.pan_neck);
		float out_vg = kpverg * (1480 - q[1] * rad2pwm - pt_ctrl.pan_left);
		float out_tilt = kptilt * (1500 - q[2] * rad2pwm - pt_ctrl.tilt_left);

		// CAUTION: Be careful on the servo direction!!!
		int neck_pan = (int) 1500 + q[0] * rad2pwm;
		if(neck_pan >= 1600) pt_ctrl.pan_neck = 1600;
		else if (neck_pan <= 1400) pt_ctrl.pan_neck = 1400;
		else  pt_ctrl.pan_neck = neck_pan;

        pt_ctrl.pan_left = 1480 - q[1] * rad2pwm;
        pt_ctrl.pan_right= 1450 + q[1] * rad2pwm;
        pt_ctrl.tilt_left= 1500 - q[2] * rad2pwm;
        pt_ctrl.tilt_right= 1500 - q[2] * rad2pwm;

	}
}
/*
pan_neck:
#ctl    1250 : 1800
#pos     192 : 277 
#ang     -80 : 90 

pan_left:
#ctl    1000 : 2000     
#pos     171 : 442
#ang     -45 : 45

tilt_left:
#ctl    1000 : 2000     
#pos     442 : 174
#ang      45 : -45

pan_right:
#ctl    1000 : 2000     
#pos     444 : 172
#ang      40 : -45

tilt_right:
#ctl    1000 : 2000     
#pos     172 : 440
#ang      50 : -40
*/

cv::Mat_<float> getT1L(){
	float deg2rad = PI / 180.;

    // Be aware of all rotation direction
    float q2 = (state_msg->pan_neck_pos - 237.) * 170. / (277. - 192.) * deg2rad;
	printf("pan_left_pos: %u, tilt_left_pos: %u \n\n", state_msg->pan_left_pos, state_msg->tilt_left_pos);
    float q3 = ((state_msg->pan_left_pos - 309.) * (45. + 45.)  / (442. - 171.)) * deg2rad;
    float q4 = ((state_msg->tilt_left_pos - 311.) * (45. + 45.) / (442. - 174.)) * deg2rad;
    printf("q2 = %4.5f, q3 = %4.5f, q4 = %4.5f \n", q2/deg2rad, q3/deg2rad, q4/deg2rad);
    float theta2 = q2; float theta3 = -PI/2 + q3; float theta4 = q4;
    float s2 = sinf(theta2); float c2 = cosf(theta2);
    float s3 = sinf(theta3); float c3 = cosf(theta3);
    float s4 = sinf(theta4); float c4 = cosf(theta4);
    // double angle formula
    float s23 = s2 * c3 + s3 * c2; float c23 = c2 * c3 - s2 * s3;

    // Derived from forward kinematic
    cv::Mat_<float> T1L = (cv::Mat_<float>(4, 4) <<-s23, -c23*s4, -c23*c4, -21.8*c23*c4-44.35*s23-136.91*c2,
                                                    c23, -s23*s4, -s23*c4, -21.8*s23*c4+44.35*c23-136.91*s2,
                                                    0.0, -c4, s4, 21.8*s4+53.5,
                                                    0.0, 0.0, 0.0, 1.);
    return T1L;
}

static void printErrorAndAbort(const std::string & error)
{
	std::cout << error << std::endl;
}

#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

using namespace std;

vector<string> get_arguments(int argc, char **argv)
{

	vector<string> arguments;

	// First argument is reserved for the name of the executable
	for (int i = 0; i < argc; ++i)
	{
		arguments.push_back(string(argv[i]));
	}
	return arguments;
}
string get_nodeName(vector<string> argus){
	for (int i = 0; i < argus.size(); ++i)
	{
		if(argus[i].compare("left") == 0 || argus[i].compare("right") == 0){
			return argus[i];
		}
	}
	return "left";
}

// Generally useful 3D functions
static void Pjt(cv::Mat_<float>& dest, const cv::Mat_<float>& mesh, float fx, float fy, float cx, float cy)
{
	dest = cv::Mat_<float>(mesh.rows, 2, 0.0);

	int num_points = mesh.rows;

	float X, Y, Z;


	cv::Mat_<float>::const_iterator mData = mesh.begin();
	cv::Mat_<float>::iterator projected = dest.begin();

	for (int i = 0; i < num_points; i++)
	{
		// Get the points
		X = *(mData++);
		Y = *(mData++);
		Z = *(mData++);

		float x;
		float y;

		// if depth is 0 the projection is different
		if (Z != 0)
		{
			x = ((X * fx / Z) + cx);
			y = ((Y * fy / Z) + cy);
		}
		else
		{
			x = X;
			y = Y;
		}

		// Project and store in dest matrix
		(*projected++) = x;
		(*projected++) = y;
	}

}


int main(int argc, char **argv)
{
    vector<string> arguments = get_arguments(argc, argv);
	string nodeName = get_nodeName(arguments);
	cout<<nodeName<<endl<<endl<<endl;

    // Write a log file
    freopen("log.txt", "w", stderr);
    int ret = 0;

	int sfd = serial_init("/dev/ttyACM0", 115200);
	if (sfd < 0) {
		printf("Unable to open serial port!!!\n");
		exit(-1);
	}
    sleep(1);

	// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	pthread_t tx_serial;
	pthread_t rx_serial;

    ret = pthread_create(&tx_serial, NULL, serial_send_thread, &sfd);
	if (ret != 0) {
		fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
		exit(EXIT_FAILURE);
	}
	ret = pthread_create(&rx_serial, NULL, serial_recv_thread, &sfd);
	if (ret != 0) {
		fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
		exit(EXIT_FAILURE);
	}


	// no arguments: output usage
	if (arguments.size() == 1)
	{
		cout << "For command line arguments see:" << endl;
		cout << " https://github.com/TadasBaltrusaitis/OpenFace/wiki/Command-line-arguments";
		return 0;
	}

	// Load the modules that are being used for tracking and face analysis
	// Load face landmark detector
	LandmarkDetector::FaceModelParameters det_parameters(arguments);
	// Always track gaze in feature extraction
	LandmarkDetector::CLNF face_model(det_parameters.model_location);

	if (!face_model.loaded_successfully)
	{
		cout << "ERROR: Could not load the landmark detector" << endl;
		return 1;
	}

	if (!face_model.eye_model)
	{
		cout << "WARNING: no eye model found" << endl;
	}

	Utilities::SequenceCapture sequence_reader;

	// A utility for visualizing the results
	Utilities::Visualizer visualizer(arguments);

	// Tracking FPS for visualization
	Utilities::FpsTracker fps_tracker;
	fps_tracker.AddFrame();
	//bool a = true;
	while (true) // this is not a for loop as we might also be reading from a webcam
	{
		// The sequence reader chooses what to open based on command line arguments provided
		if (!sequence_reader.Open(arguments))
			break;

		INFO_STREAM("Device or file opened");

		if (sequence_reader.IsWebcam())
		{
			INFO_STREAM("WARNING: using a webcam, forcing visualization of tracking to allow quitting the application (press q)");
			visualizer.vis_track = true;
		}

		cv::Mat captured_image;

		captured_image = sequence_reader.GetNextFrame();

		INFO_STREAM("Starting tracking");


        // Try to hack the control loop: update the reference gaze point at low rate.
        int counter = 0;
        // Create rotation control quantity vector.
        float quan[3] = {0., 0., 0.};
        float *q = quan;

		while (!captured_image.empty()) // && ros::ok())
		{
            // Converting to grayscale
            cv::Mat_<uchar> grayscale_image = sequence_reader.GetGrayFrame();
            // The actual facial landmark detection / tracking
            bool detection_success = LandmarkDetector::DetectLandmarksInVideo(captured_image, face_model, det_parameters, grayscale_image);
			
            // Gaze tracking, absolute gaze direction
            //cv::Point3f gazeDirection0(0, 0, 0); cv::Point3f gazeDirection1(0, 0, 0); cv::Vec2d gazeAngle(0, 0);

            // if (detection_success && face_model.eye_model)
            // {
            // 	GazeAnalysis::EstimateGaze(face_model, gazeDirection0, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, true);
            // 	GazeAnalysis::EstimateGaze(face_model, gazeDirection1, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy, false);

            // 	gazeAngle = GazeAnalysis::GetGazeAngle(gazeDirection0, gazeDirection1);
            // }
			
            cv::Vec6d pose_WRTcamera = LandmarkDetector::GetPoseWRTCamera(face_model, sequence_reader.fx, 
                                        sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
    
            if (detection_success && face_model.eye_model){
                // Initialize rotation control vector
                if(counter >= 3) counter = 0;
                if(counter == 0){
                    // Form the rotation matrix
                    cv::Vec3f eulerAngles = cv::Vec3f((float) pose_WRTcamera[3], (float) pose_WRTcamera[4], (float) pose_WRTcamera[5]); 
                    cv::Matx33f rMtx = Utilities::Euler2RotationMatrix (eulerAngles);

                    cv::Mat_<float> RMtx = (cv::Mat_<float>(4, 3) << rMtx(0, 0), rMtx(0, 1), rMtx(0, 2),
                                                                    rMtx(1, 0), rMtx(1, 1), rMtx(1, 2),
                                                                    rMtx(2, 0), rMtx(2, 1), rMtx(2, 2),
                                                                    0.0, 0.0, 0.0);
                    // Form the translation matrix
                    cv::Mat_<float> TMtx = (cv::Mat_<float>(4, 1) << (float) pose_WRTcamera[0], (float) pose_WRTcamera[1], (float) pose_WRTcamera[2], 1.0);
                    
                    // Form the tranformation matrix
                    cv::Mat_<float> TLH;
                    cv::hconcat(RMtx, TMtx, TLH);
                    cerr<<"Transformation between face and cam: \n"<<TLH<<endl;

                    // Compute the transformation matrix T1L
                    cv::Mat_<float> T1L = getT1L();
                    //cout<<T1L<<endl;

                    cout<<"trans x: "<<pose_WRTcamera[0]<<"  trans y: "<<pose_WRTcamera[1]<<"  trans z: "<<pose_WRTcamera[2]<<endl;
                    //cout<<"euler x: "<<pose_WRTcamera[3]<<"  euler y: "<<pose_WRTcamera[4]<<"  euler z: "<<pose_WRTcamera[5]<<endl;
                    
                    // Transformate the gaze point to the camera coordinate
                    // Create gaze point (attention)
                    cv::Mat_<float> gzpoint = (cv::Mat_<float>(4, 1) << 0.0, 0.0, -150.0, 1);
                    cv::Mat_<float> T1H = T1L * TLH;
                    cv::Mat gzWRTwd1 = T1L * TLH * gzpoint;
                    cerr<<"Transformation between world and cam: \n"<<T1L<<endl;

                    cv::Rect rect(0, 0, 1, 3);
                    cv::Mat_<float> gzWRTwd;
                    gzWRTwd1(rect).copyTo(gzWRTwd);
                    cv::transpose(gzWRTwd, gzWRTwd);

                    //! Control with disparity On image plane
                    //cout<<gzWRTwd<<endl;
                    // Project the gaze point to image plane;
                    //cv::Mat_<float> gzONimg;
                    //Pjt(gzONimg, gzWRTcamera, sequence_reader.fx, 
                    //    sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
                    //cout<<gzONimg<<endl;

                    float gzPoint[3] = {gzWRTwd(0), gzWRTwd(1), gzWRTwd(2)};
                    q = Inv_Kinematic(gzPoint);

                    printf("Gaze Point: [%4.6f, %4.6f, %4.6f]\n", gzPoint[0], gzPoint[1], gzPoint[2] );
                    cerr<<"Gaze Point WRT world--x: "<<gzPoint[0]<<", y: "<<gzPoint[1]<<" , z: "<<gzPoint[2]<<endl;
                    printf("quantity of rotation: [%4.6f, %4.6f, %4.6f]\n", q[0]*180/PI, q[1]*180/PI, q[2]*180/PI );
                    cerr<<"quantity of rotations--q2: "<<q[0]*180/PI<<", q3: "<<q[1]*180/PI<<", q4: "<<q[2]*180/PI<<endl;
                }
                vergence_servo_ctrl(q);
                // Print out generated joint value
                //printf("q2 = %4.5f, q3 = %4.5f, q4 = %4.5f \n", q[0], q[1], q[2]);
            }

            // Keeping track of FPS
            fps_tracker.AddFrame();

            // Displaying the tracking visualizations
            visualizer.SetImage(captured_image, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy);
            visualizer.SetObservationLandmarks(face_model.detected_landmarks, face_model.detection_certainty, face_model.GetVisibilities());
            visualizer.SetObservationPose(pose_WRTcamera, face_model.detection_certainty);
            //visualizer.SetObservationGaze(gazeDirection0, gazeDirection1, LandmarkDetector::CalculateAllEyeLandmarks(face_model), LandmarkDetector::Calculate3DEyeLandmarks(face_model, sequence_reader.fx, sequence_reader.fy, sequence_reader.cx, sequence_reader.cy), face_model.detection_certainty);
            visualizer.SetFps(fps_tracker.GetFPS());

            // detect key presses
            char character_press = visualizer.ShowObservation();
			
            // quit processing the current sequence (useful when in Webcam mode)
            if (character_press == 'q')
            {break;}

            // Grabbing the next frame in the sequence
            captured_image = sequence_reader.GetNextFrame();
            // Set counter
            counter++;
            //ros::spinOnce();
            //loop_rate.sleep();
        }	
    	INFO_STREAM("Closing input reader");
    	sequence_reader.Close();
    	INFO_STREAM("Closed successfully");

    	// Reset the models for the next video
    	// face_analyser.Reset();
    	face_model.Reset();
    }
	pthread_join(tx_serial, NULL);
	pthread_join(rx_serial, NULL);
    return 0;
}
