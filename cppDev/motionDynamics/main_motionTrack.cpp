#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <thread>
#include "MultiThreadQueue.h"
#include "MotionTracker.h"

using namespace cv;
using namespace std;


/******************** Functions ********************/
void videoCapture(void);
void videoProcess(void);
void videoDisplay(void);


/******************** Constants ********************/
#define FPS 30//40
#define WIDTH 640
#define HEIGHT 480


/******************** Global Variables ********************/
MultiThreadQueue<Mat> qCap(10), qProc(10);
VideoCapture* vidCam;
bool isRunning;
MotionTracker* mTracker;


int main()
{

	/******************** Camera Initialization ********************/
	vidCam = new VideoCapture;
	vidCam->open(0);
	//vidCam->open("tcpclientsrc host = 192.168.0.111 port = 7001 ! decodebin ! videoconvert ! appsink", CAP_GSTREAMER);
	if (!vidCam->isOpened())
	{
		cerr << "Application Failure: Video stream failed. Exiting now..." << endl;
		return 0;
	}

	//if (!vidCam->set(cv::CAP_PROP_FPS, FPS) ||
	//	!vidCam->set(cv::CAP_PROP_FRAME_WIDTH, WIDTH) ||
	//	!vidCam->set(cv::CAP_PROP_FRAME_HEIGHT, HEIGHT))
	//{
	//	cerr << "Cannot Set FPS" << endl;
	//	return 0;
	//}


	/******************** Motion Tracker Setup ********************/
	// Motion tracking object pointer
	Ptr<BackgroundSubtractorMOG2> pBackSub = createBackgroundSubtractorMOG2();
	pBackSub->setBackgroundRatio(0.7);	// set to match Matlab
	pBackSub->setNMixtures(3); // set to match Matlab

	SimpleBlobDetector::Params blobParams;
	blobParams.minThreshold = 0;
	blobParams.maxThreshold = 254;
	blobParams.thresholdStep = 253;
	blobParams.minDistBetweenBlobs = 50;
	blobParams.filterByArea = true;
	blobParams.minArea = 400;
	blobParams.maxArea = (HEIGHT * WIDTH) / 10; // allow blobs as large as 1/10th the screen to be detected
	blobParams.filterByColor = false;
	blobParams.filterByCircularity = false;
	blobParams.filterByConvexity = false;
	blobParams.filterByInertia = false;
	Ptr<SimpleBlobDetector> pBlobDetector = SimpleBlobDetector::create(blobParams);

	Mat openStrel = getStructuringElement(cv::MORPH_RECT, Size(10, 10));
	Mat closeStrel = getStructuringElement(cv::MORPH_RECT, Size(20, 20));

	mTracker = new MotionTracker(pBackSub, pBlobDetector, openStrel, closeStrel, FPS);
	Mat mask, detectFrame;
	vector<KeyPoint> detectedCentroids, trackedCentroids;



	/******************** Thread setup / Launch threads ********************/
	isRunning = true;

	thread thread_vidCap, thread_vidProc, thread_vidDisp;
	thread_vidCap = thread(videoCapture);
	thread_vidProc = thread(videoProcess);
	thread_vidDisp = thread(videoDisplay);


	/******************** Main Thread Idle ********************/
	while (isRunning);


	/******************** Clean up ********************/
	thread_vidCap.join();
	thread_vidProc.join();
	thread_vidDisp.join();
	delete vidCam;

	return 1;
}


void videoCapture(void)
{
	Mat frame;

	while (isRunning)
	{
		// Keep trying to write to buffer until we can push
		vidCam->read(frame);
		while (!frame.empty() && !qCap.push(frame) && isRunning);
	}
}


void videoProcess(void)
{
	Mat frame;
	Mat mask, detectFrame;
	vector<KeyPoint> detectedCentroids, trackedCentroids;

	while (isRunning)
	{
		/******************** Frame In ********************/
		// Keep trying to write to buffer until we can push
		while (!qCap.pop(frame) && isRunning);

		/******************** Process ********************/
		// Motion detection, tracking, and path management
		mTracker->detect(frame, mask, detectedCentroids);
		mTracker->predictNewLocationsOfTracks();
		mTracker->getCentroids(trackedCentroids);
		mTracker->assignDetectionsToTracks(detectedCentroids, 200.0);
		mTracker->deleteLostTracks();

		// Draw detected circles around detected motion
		drawKeypoints(frame, trackedCentroids, detectFrame, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		drawKeypoints(detectFrame, detectedCentroids, detectFrame, Scalar(0, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//drawKeypoints(mask, trackedCentroids, detectFrame, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//drawKeypoints(detectFrame, detectedCentroids, detectFrame, Scalar(0, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

		/******************** Frame Out ********************/
		// Keep trying to write to buffer until we can push
		while (!qProc.push(detectFrame) && isRunning);
	}
}


void videoDisplay(void)
{
	Mat frame;
	char k;

	while (isRunning)
	{
		// Keep trying to read from buffer until we can pop
		while (!qProc.pop(frame) && isRunning);

		imshow("Cam", frame);
		k = (char)waitKey(1);
		if (k == 27 || k == 'q' || k == 'Q') isRunning = false;
	}
}

