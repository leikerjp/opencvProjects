
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include "MultiThreadQueue.h"

using namespace cv;
using namespace std;

#define M_PI 3.14159265358979323846

/******************** Functions ********************/
void videoCapture0(void);
void videoCapture1(void);
void videoProcess(void);
void videoDisplay(void);


/******************** Constants ********************/
#define FPS 30//40
#define WIDTH 640
#define HEIGHT 480


/******************** Global Variables ********************/
MultiThreadQueue<Mat> qCap0(10), qCap1(10), qProc(10), qTest(10);
bool isRunning;
// ranges picked based on https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
unsigned char lowB = 0, lowG = 0, lowR = 125;
unsigned char highB = 100, highG = 100, highR = 255;

int main()
{
	isRunning = true;

	/******************** Thread setup / Launch threads ********************/
	thread thread_vidCap0, thread_vidCap1, thread_vidProc, thread_vidDisp;
	thread_vidCap0 = thread(videoCapture0);
	thread_vidCap1 = thread(videoCapture1);
	thread_vidProc = thread(videoProcess);
	thread_vidDisp = thread(videoDisplay);


	/******************** Main Thread Idle ********************/
	cout << "All Threads Launched!" << endl;
	while (isRunning);


	/******************** Clean up ********************/
	thread_vidCap0.join();
	thread_vidCap1.join();
	thread_vidProc.join();
	thread_vidDisp.join();

	return 1;
}


void videoCapture0(void)
{
	Mat frame; 
	VideoCapture vidCam("udpsrc port=5001 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink", CAP_GSTREAMER);
	if (!vidCam.isOpened())
	{
		cerr << "Application Failure: Video stream 0 failed. Exiting now..." << endl;
		isRunning = false;
	}
	else
	{
		cout << "Cam 0 launched successfully!" << endl;
	}

	while (isRunning)
	{
		vidCam.read(frame);
		// Convert frame to grayscale (for disparity calc). This was moved to ingress thread for efficiency purposes
		//cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		// Keep trying to write to buffer until we can push
		while (!frame.empty() && !qCap0.push(frame) && isRunning);
	}
}


void videoCapture1(void)
{
	Mat frame;
	VideoCapture vidCam("udpsrc port=5000 caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! decodebin ! videoconvert ! appsink", CAP_GSTREAMER);
	if (!vidCam.isOpened())
	{
		cerr << "Application Failure: Video stream 1 failed. Exiting now..." << endl;
		isRunning = false;
	}
	else
	{
		cout << "Cam 1 launched successfully!" << endl;
	}

	while (isRunning)
	{
		vidCam.read(frame);
		// Convert frame to grayscale (for disparity calc). This was moved to ingress thread for efficiency purposes
		//cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		// Keep trying to write to buffer until we can push
		while (!frame.empty() && !qCap1.push(frame) && isRunning);
	}
}


void videoProcess(void)
{
	Mat frameL, frameR, frameOutL, frameOutR, measureFrame;
	std::vector<KeyPoint> centroidsL, centroidsR;

	double fLen = 3.04; // {mm}
	double hFov = 62.2;
	double vFov = 48.8;

	double xFocalPix = ((double)WIDTH / 2.0) / tan((hFov * M_PI / 180.0) / 2.0);
	double xBase = 25.4; // 25.4 mm == 1 inch
	double xBf = (xBase * xFocalPix);

	double yFocalPix = ((double)HEIGHT / 2.0) / tan((vFov * M_PI / 180.0) / 2.0);	
	double yBase = 0; // TODO
	double yBf = (yBase * yFocalPix);

	double disparity;

	// Blob detector
	SimpleBlobDetector::Params blobParams;
	blobParams.minThreshold = 0;
	blobParams.maxThreshold = 254;
	blobParams.thresholdStep = 253;
	blobParams.minDistBetweenBlobs = 128;
	blobParams.filterByArea = true;
	blobParams.minArea = 400;
	blobParams.maxArea = (HEIGHT * WIDTH) / 10; // allow blobs as large as 1/10th the screen to be detected
	blobParams.filterByColor = false;
	blobParams.filterByCircularity = false;
	blobParams.filterByConvexity = false;
	blobParams.filterByInertia = false;
	Ptr<SimpleBlobDetector> pBlobDetector = SimpleBlobDetector::create(blobParams);

	while (isRunning)
	{
		/******************** Frames In ********************/
		// Keep trying to read from buffers until we can pop
		while (!qCap0.pop(frameL) && isRunning);
		while (!qCap1.pop(frameR) && isRunning);


		/******************** Process ********************/
		// Calculate image disparity (note: frameL and frameR need to be grayscale)
		//Ptr<StereoBM> stereo = StereoBM::create(16, 15);
		//stereo->compute(frameL, frameR, disparityFrame);
		//// Output of stereo is a SIXTEEN BIT (16 bit) image, but imshow needs 8 bit images to display
		//disparityFrame.convertTo(disparityFrame, CV_8U);

		//// Calc distance from disparity
		//disparity = (double)disparityFrame.at<unsigned short>((int)HEIGHT / 2, (int)WIDTH / 2);

		// Threshold by color (temporarily using this as object detector method. Set to detect RED and using 
		// a red ball for measurements). Then detect blobs and use centroids to calculate disparity of detected objects
		cv::inRange(frameL, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), frameOutL);
		cv::inRange(frameR, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), frameOutR);
		pBlobDetector->detect(frameOutL, centroidsL);
		pBlobDetector->detect(frameOutR, centroidsR);

		if (centroidsL.size() > 0 && centroidsR.size() > 0)
		{
			disparity = xBf / (double(centroidsL[0].pt.y) - double(centroidsR[0].pt.y));
			disparity /= 25.4; // convert millimeter to inches
			drawKeypoints(frameL, centroidsL, frameL, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			drawKeypoints(frameR, centroidsR, frameR, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			cv::putText(frameL,
				std::to_string(disparity) + " in",
				cv::Point(centroidsL[0].pt.x, centroidsL[0].pt.y), // Coordinates
				cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
				1.0, // Scale. 2.0 = 2x bigger
				cv::Scalar(255, 255, 255), // BGR Color
				1, // Line Thickness (Optional)
				cv::LINE_AA); // Anti-alias (Optional, see version note)
		}
		else
		{
			cv::putText(frameL,
				"No objects detected",
				cv::Point(50, 100), // Coordinates
				cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
				1.0, // Scale. 2.0 = 2x bigger
				cv::Scalar(255, 255, 255), // BGR Color
				1, // Line Thickness (Optional)
				cv::LINE_AA); // Anti-alias (Optional, see version note)
		}
	



		/******************** Frame Out ********************/
		// Keep trying to write to buffer until we can push
		while (!qProc.push(frameL) && isRunning);
		while (!qProc.push(frameR) && isRunning);
	}
}


void videoDisplay(void)
{
	Mat frame, frameTest;
	char k;

	while (isRunning)
	{
		// Keep trying to read from buffer until we can pop
		while (!qProc.pop(frame) && isRunning);
		while (!qProc.pop(frameTest) && isRunning);

		imshow("Cam", frame);
		imshow("CamTest", frameTest);
		k = (char)waitKey(1);
		if (k == 27 || k == 'q' || k == 'Q') isRunning = false;
	}
}

