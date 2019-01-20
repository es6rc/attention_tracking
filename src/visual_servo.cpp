/*
 * visual_servo.cpp
 *
 *  Created on: Jun 26, 2018
 *      Author: priori
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/objdetect/objdetect.hpp"

#include "head_neck_ctl.h"

using namespace cv;
using namespace std;



static int color_track(Mat &frame, Mat &frame_processed,
		int low_h, int high_h, int low_s, int high_s, int low_v, int high_v,
		int *u, int *v) {

	int ret = 0;

	// convert to HSV
	cvtColor(frame, frame_processed, COLOR_BGR2HSV);
	// binary threshold
	inRange(frame_processed, Scalar(low_h, low_s, low_v),
			Scalar(high_h, high_s, high_v), frame_processed);



	// morphological opening (remove small objects from the foreground)
	erode(frame_processed, frame_processed,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	dilate(frame_processed, frame_processed,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

	// morphological closing (fill small holes in the foreground)
	dilate(frame_processed, frame_processed,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	erode(frame_processed, frame_processed,
			getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


	// calculate the moments of the thresholded image
	Moments m = moments(frame_processed);

	double m01 = m.m01;
	double m10 = m.m10;
	double m00 = m.m00;

	double u1 = m10 / m00;
	double v1 = m01 / m00;


	// convert processed image back to RGB
	cvtColor(frame_processed, frame_processed, COLOR_GRAY2BGR);


	if(u1 > 10 && u1 < frame_processed.cols - 10 && v1 > 10 && v1 < frame_processed.rows - 10) {

		line(frame_processed, Point(u1, v1 -10), Point(u1, v1 + 10), Scalar(0, 0, 255), 3);
		line(frame_processed, Point(u1 - 10, v1), Point(u1 + 10, v1), Scalar(0, 0, 255), 3);

		ret = 1;

	} else {

		u1 = 0;
		v1 = 0;

		ret = 0;
	}


	*u = (int)u1;
	*v = (int)v1;

	return ret;
}

static int face_track(CascadeClassifier &classifier, Mat &frame, Mat &frame_processed,
		int *u, int *v) {

	int ret = 0;

	frame_processed = frame;

	vector<Rect> faces;
	Mat frame_gray;

	cvtColor(frame, frame_gray, CV_BGR2GRAY);
	equalizeHist(frame_gray, frame_gray);

	classifier.detectMultiScale(frame_gray, faces, 1.1, 2,
			0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

	if (faces.size() == 0) {

		*u = 0;
		*v = 0;

		return 0;

	} else {

		Rect face = faces[0];

		// choose the face with the largest bounding box
		for (size_t i = 0; i < faces.size(); i++) {

			if (faces[i].height > face.height && faces[i].width > face.width) {

				face = faces[i];
			}

		}


		rectangle(frame_processed, face, Scalar(255, 0, 255), 4);

		*u = face.x + face.width / 2;
		*v = face.y + face.height / 2;

		return 1;
	}

}


double pid_ctrl(double err, double err_sum, double err_diff, double kp, double ki, double kd) {

	return err * kp + err_sum * ki + err * kd;
}

void visual_servo_left(int enable, int u, int v) {

	// PID controller

	double kpu = 0.05;
	double kpv = 0.05;

	if (!isnan(u) && !isnan(v) && enable) {


		double err_u = u - 320;
		double err_v = v - 240;


		double out_u = pid_ctrl(err_u, 0, 0, kpu, 0, 0);
		double out_v = pid_ctrl(err_v, 0, 0, kpv, 0, 0);

		// CAUTION: Be careful on the servo direction!!!

		pt_ctrl.pan_left += out_u;
		pt_ctrl.tilt_left += out_v;

	}
}

void visual_servo_right(int enable, int u, int v) {

	// PID controller

	double kpu = 0.05;
	double kpv = 0.05;

	if (!isnan(u) && !isnan(v) && enable) {


		double err_u = u - 320;
		double err_v = v - 240;


		double out_u = pid_ctrl(err_u, 0, 0, kpu, 0, 0);
		double out_v = pid_ctrl(err_v, 0, 0, kpv, 0, 0);

		// CAUTION: Be careful on the servo direction!!!

		pt_ctrl.pan_right += out_u;
		pt_ctrl.tilt_right += out_v;

	}
}

void vergence_servo_ctrl(int enable1, int enable2, int u1, int v1, int u2, int v2){
	
	// PID controller

	double kpneck = 0.005;
	double kptilt = 0.02;
	double kpvg = 0.02;

	if (!isnan(u1) && !isnan(v1) && enable1 && !isnan(u2) && !isnan(v2) && enable2) {


		double xl = u1 - 320;
		double yl = v1 - 240;
		double xr = u2 - 320;
		double yr = v2 - 240;

		double delta = xl;
		double out_neck = pid_ctrl(xl + xr, 0, 0, kpneck, 0, 0);
		double out_tilt = pid_ctrl(yl + yr, 0, 0, kptilt, 0, 0);
		double out_vg = pid_ctrl(delta, 0, 0, kpvg, 0, 0);

		// CAUTION: Be careful on the servo direction!!!

		pt_ctrl.pan_neck -= out_neck;
		if (abs(yl) > 15 && abs(yr) > 15){
			pt_ctrl.tilt_right += out_tilt;
			pt_ctrl.tilt_left += out_tilt;
		}
		if (abs(xl) > 15 && abs(xr) > 15){
			pt_ctrl.pan_left += out_vg;
			pt_ctrl.pan_right -= out_vg;
		}
	}
}

void single_cam_visual_servo_loop_color_track(int cam_id) {

	VideoCapture cam(cam_id);

	if (!cam.isOpened()) {
		cerr << "Error opening camera" << endl;
	}

	//cout << "Connected to camera =" << picam.getId() << endl;

	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	namedWindow("original", CV_WINDOW_AUTOSIZE);
	namedWindow("threshold", CV_WINDOW_AUTOSIZE);

	int low_h = 0;
	int high_h = 20;

	int low_s = 180;
	int high_s = 255;

	int low_v = 150;
	int high_v = 255;

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &low_h, 180); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &high_h, 180);

	createTrackbar("LowS", "Control", &low_s, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &high_s, 255);

	createTrackbar("LowV", "Control", &low_v, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &high_v, 255);

	while (1) {

		Mat frame;
		Mat frame_processed;

		cam.grab();
		cam.retrieve(frame);


		int u = 0;
		int v = 0;

		int ret = color_track(frame, frame_processed, low_h, high_h, low_s, high_s, low_v, high_v, &u, &v);

		visual_servo_left(ret, u, v);




		// show the ROI and the image



		imshow("original", frame); //show the original image
		imshow("threshold", frame_processed);

		cout << "center of mass: (" << u << ", " << v << ")" << endl;

		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break;
		}

	}

	cam.release();


}

void single_cam_visual_servo_loop_face_track(int cam_id) {

	String face_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
	CascadeClassifier face_cascade;


	if (!face_cascade.load(face_cascade_name)) {

		printf("unable to load face detector\n");
		return;
	}


	VideoCapture cam(cam_id);

	if (!cam.isOpened()) {
		cerr << "Error opening camera" << endl;
	}

	//cout << "Connected to camera =" << picam.getId() << endl;

	namedWindow("frame", CV_WINDOW_AUTOSIZE);


	while (1) {

		Mat frame;
		Mat frame_processed;

		cam.grab();
		cam.retrieve(frame);


		int u = 0;
		int v = 0;

		int ret = face_track(face_cascade, frame, frame_processed, &u, &v);

		visual_servo_left(ret, u, v);

		imshow("frame", frame_processed);

		cout << "center of mass: (" << u << ", " << v << ")" << endl;

		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break;
		}

	}

	cam.release();


}

void dual_cam_visual_servo_loop_color_track(int cam_left_id, int cam_right_id) {

	VideoCapture cam_left(cam_left_id);
	VideoCapture cam_right(cam_right_id);

	if (!cam_left.isOpened() || !cam_right.isOpened()) {
		cerr << "Error opening camera" << endl;
	}

	//cout << "Connected to camera =" << picam.getId() << endl;

	namedWindow("Control", CV_WINDOW_AUTOSIZE);
	namedWindow("original_left", CV_WINDOW_AUTOSIZE);
	namedWindow("threshold_left", CV_WINDOW_AUTOSIZE);
	namedWindow("original_right", CV_WINDOW_AUTOSIZE);
	namedWindow("threshold_right", CV_WINDOW_AUTOSIZE);

	int low_h = 0;
	int high_h = 17;

	int low_s = 180;
	int high_s = 255;

	int low_v = 150;
	int high_v = 255;

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &low_h, 180); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &high_h, 180);

	createTrackbar("LowS", "Control", &low_s, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &high_s, 255);

	createTrackbar("LowV", "Control", &low_v, 255); //Value (0 - 255)
	createTrackbar("HighV", "Control", &high_v, 255);

	while (1) {

		Mat frame_left;
		Mat frame_right;
		Mat frame_left_processed;
		Mat frame_right_processed;


		cam_left.grab();
		cam_right.grab();
		cam_left.retrieve(frame_left);
		cam_right.retrieve(frame_right);


		int u_left = 0;
		int v_left = 0;
		int u_right = 0;
		int v_right = 0;

		int ret_left = color_track(frame_left, frame_left_processed,
				low_h, high_h, low_s, high_s, low_v, high_v, &u_left, &v_left);
		int ret_right = color_track(frame_right, frame_right_processed,
				low_h, high_h, low_s, high_s, low_v, high_v, &u_right, &v_right);

		visual_servo_left(ret_left, u_left, v_left);
		visual_servo_right(ret_right, u_right, v_right);



		imshow("original_left", frame_left);
		imshow("threshold_left", frame_left_processed);
		imshow("original_right", frame_right);
		imshow("threshold_right", frame_right_processed);


		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break;
		}

	}

	cam_left.release();
	cam_right.release();


}

void dual_cam_visual_servo_loop_face_track(int cam_left_id, int cam_right_id) {

	String face_cascade_name = "/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml";
	CascadeClassifier face_cascade;


	if (!face_cascade.load(face_cascade_name)) {

		printf("unable to load face detector\n");
		return;
	}

	VideoCapture cam_left(cam_left_id);
	VideoCapture cam_right(cam_right_id);

	if (!cam_left.isOpened() || !cam_right.isOpened()) {
		cerr << "Error opening camera" << endl;
	}


	while (1) {

		Mat frame_left;
		Mat frame_right;
		Mat frame_left_processed;
		Mat frame_right_processed;


		cam_left.grab();
		cam_right.grab();
		cam_left.retrieve(frame_left);
		cam_right.retrieve(frame_right);


		int u_left = 0;
		int v_left = 0;
		int u_right = 0;
		int v_right = 0;

		int ret_left = face_track(face_cascade, frame_left, frame_left_processed,
				&u_left, &v_left);
		int ret_right = face_track(face_cascade, frame_right, frame_right_processed,
				&u_right, &v_right);

		// visual_servo_left(ret_left, u_left, v_left);
		// visual_servo_right(ret_right, u_right, v_right);
		vergence_servo_ctrl(ret_left, ret_right, u_left, v_left, u_right, v_right);


		imshow("left", frame_left_processed);
		imshow("right", frame_right_processed);


		if (waitKey(30) == 27) {
			cout << "esc key is pressed by user" << endl;
			break;
		}

	}

	cam_left.release();
	cam_right.release();


}


