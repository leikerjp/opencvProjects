//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <iostream>
//#include <thread>
//#include "MultiThreadQueue.h"
//
//using namespace cv;
//using namespace std;
//
//
//void videoCapture(void);
//void videoProcess(void);
//void videoDisplay(void);
//
//
//
//#define FPS 30//40
//#define WIDTH 640
//#define HEIGHT 480
//
//
//
//MultiThreadQueue<Mat> qCap(10), qProc(10);
//VideoCapture* vidCam;
//bool isRunning;
//
//
//
//int main()
//{
//	vidCam = new VideoCapture;
//	vidCam->open(0);
//	if (!vidCam->isOpened())
//	{
//		cerr << "Application Failure: Video stream failed. Exiting now..." << endl;
//		return 0;
//	}
//	isRunning = true;
//
//	thread thread_vidCap, thread_vidProc, thread_vidDisp;
//	thread_vidCap = thread(videoCapture);
//	thread_vidProc = thread(videoProcess);
//	thread_vidDisp = thread(videoDisplay);
//	
//	while (isRunning);
//
//	thread_vidCap.join();
//	thread_vidProc.join();
//	thread_vidDisp.join();
//	delete vidCam;
//	return 1;
//}
//
//
//void videoCapture(void)
//{
//	Mat frame;
//
//	while (isRunning)
//	{
//		// Keep trying to write to buffer until we can push
//		vidCam->read(frame);
//		while (!frame.empty() && !qCap.push(frame) && isRunning);
//	}
//}
//
//
//void videoProcess(void)
//{
//	Mat frameIn, frameOut;
//
//	while (isRunning)
//	{
//		// Keep trying to write to buffer until we can push
//		while (!qCap.pop(frameIn) && isRunning);
//
//		// Process
//		frameOut = frameIn;
//
//		// Keep trying to write to buffer until we can push
//		while (!qProc.push(frameOut) && isRunning);
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
