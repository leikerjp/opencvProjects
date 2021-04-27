//
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/calib3d.hpp>
//#include <iostream>
//#include <thread>
//#include "MultiThreadQueue.h"
//
//using namespace cv;
//using namespace std;
//
//#define M_PI 3.14159265358979323846
//
///******************** Functions ********************/
//void videoCapture0(void);
//void videoCapture1(void);
//void videoProcess(void);
//void videoDisplay(void);
//
//
///******************** Constants ********************/
//#define FPS 30//40
//#define WIDTH 640
//#define HEIGHT 480
//
//
///******************** Global Variables ********************/
//MultiThreadQueue<Mat> qCap0(10), qCap1(10), qProc(10);
//bool isRunning;
//
//
//int main()
//{
//	isRunning = true;
//
//	/******************** Thread setup / Launch threads ********************/
//	thread thread_vidCap0, thread_vidCap1, thread_vidProc, thread_vidDisp;
//	thread_vidCap0 = thread(videoCapture0);
//	thread_vidCap1 = thread(videoCapture1);
//	thread_vidProc = thread(videoProcess);
//	thread_vidDisp = thread(videoDisplay);
//
//
//	/******************** Main Thread Idle ********************/
//	cout << "All Threads Launched!" << endl;
//	while (isRunning);
//
//
//	/******************** Clean up ********************/
//	thread_vidCap0.join();
//	thread_vidCap1.join();
//	thread_vidProc.join();
//	thread_vidDisp.join();
//
//	return 1;
//}
//
//
//void videoCapture0(void)
//{
//	Mat frame; 
//	VideoCapture vidCam("udpsrc port=5001 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink", CAP_GSTREAMER);
//	if (!vidCam.isOpened())
//	{
//		cerr << "Application Failure: Video stream 0 failed. Exiting now..." << endl;
//		isRunning = false;
//	}
//	else
//	{
//		cout << "Cam 0 launched successfully!" << endl;
//	}
//
//	while (isRunning)
//	{
//		vidCam.read(frame);
//		// Convert frame to grayscale (for disparity calc). This was moved to ingress thread for efficiency purposes
//		cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
//		// Keep trying to write to buffer until we can push
//		while (!frame.empty() && !qCap0.push(frame) && isRunning);
//	}
//}
//
//
//void videoCapture1(void)
//{
//	Mat frame;
//	VideoCapture vidCam("udpsrc port=5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink", CAP_GSTREAMER);
//	if (!vidCam.isOpened())
//	{
//		cerr << "Application Failure: Video stream 1 failed. Exiting now..." << endl;
//		isRunning = false;
//	}
//	else
//	{
//		cout << "Cam 1 launched successfully!" << endl;
//	}
//
//	while (isRunning)
//	{
//		vidCam.read(frame);
//		// Convert frame to grayscale (for disparity calc). This was moved to ingress thread for efficiency purposes
//		cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
//		// Keep trying to write to buffer until we can push
//		while (!frame.empty() && !qCap1.push(frame) && isRunning);
//	}
//}
//
//
//void videoProcess(void)
//{
//	Mat frameL, frameR, disparityFrame, measureFrame;
//
//	double fLen = 3.04; // {mm}
//	double hFov = 62.2;
//	double vFov = 48.8;
//
//	double xFocalPix = ((double)WIDTH / 2.0) / tan((hFov * M_PI / 180.0) / 2.0);
//	double xBase = 25.4; // 25.4 mm == 1 inch
//	double xBf = (xBase * xFocalPix);
//
//	double yFocalPix = ((double)HEIGHT / 2.0) / tan((vFov * M_PI / 180.0) / 2.0);	
//	double yBase = 0; // TODO
//	double yBf = (yBase * yFocalPix);
//
//	double disparity;
//
//	while (isRunning)
//	{
//		/******************** Frames In ********************/
//		// Keep trying to read from buffers until we can pop
//		while (!qCap0.pop(frameL) && isRunning);
//		while (!qCap1.pop(frameR) && isRunning);
//
//
//		/******************** Process ********************/
//		// Calculate image disparity (note: frameL and frameR need to be grayscale)
//		Ptr<StereoBM> stereo = StereoBM::create(16, 15);
//		stereo->compute(frameL, frameR, disparityFrame);
//		//// Output of stereo is a SIXTEEN BIT (16 bit) image, but imshow needs 8 bit images to display
//		//disparityFrame.convertTo(disparityFrame, CV_8U);
//
//		//// Calc distance from disparity
//		disparity = (double)disparityFrame.at<unsigned short>((int)HEIGHT / 2, (int)WIDTH / 2);
//		double dist = xBf / disparity;
//		cout << "bf " << xBf << "\t" << "dist " << dist << "\t" << "disparity " << disparity << endl;
//	
//		// Add the distance to the frame
//		cv::putText(frameL,
//					std::to_string(dist) + " mm",
//					cv::Point((double)WIDTH / 2, (double)HEIGHT / 2), // Coordinates
//					cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
//					1.0, // Scale. 2.0 = 2x bigger
//					cv::Scalar(255, 255, 255), // BGR Color
//					1, // Line Thickness (Optional)
//					cv::LINE_AA); // Anti-alias (Optional, see version note)
//
//
//		/******************** Frame Out ********************/
//		// Keep trying to write to buffer until we can push
//		while (!qProc.push(frameL) && isRunning);
//	}
//}
//
//
//void videoDisplay(void)
//{
//	Mat frame;
//	char k;
//
//	while (isRunning)
//	{
//		// Keep trying to read from buffer until we can pop
//		while (!qProc.pop(frame) && isRunning);
//
//		imshow("Cam", frame);
//		k = (char)waitKey(1);
//		if (k == 27 || k == 'q' || k == 'Q') isRunning = false;
//	}
//}
//
