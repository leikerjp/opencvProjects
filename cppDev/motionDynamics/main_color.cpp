#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <thread>
#include <vector>
#include "MultiThreadQueue.h"

using namespace cv;
using namespace std;


void videoCapture(void);
void videoProcess(void);
void videoDisplay(void);



#define FPS 30//40
#define WIDTH 640
#define HEIGHT 480


MultiThreadQueue<Mat> qCap(10), qProc(10);
bool isRunning;
// ranges picked based on https://docs.opencv.org/3.4/da/d97/tutorial_threshold_inRange.html
unsigned char lowB = 0, lowG = 0, lowR = 150;
unsigned char highB = 100, highG = 100, highR = 255;



int main()
{
	isRunning = true;

	thread thread_vidCap, thread_vidProc, thread_vidDisp;
	thread_vidCap = thread(videoCapture);
	thread_vidProc = thread(videoProcess);
	thread_vidDisp = thread(videoDisplay);
	
	while (isRunning);

	thread_vidCap.join();
	thread_vidProc.join();
	thread_vidDisp.join();

	return 1;
}


void videoCapture(void)
{
	VideoCapture vidCam(0);
	Mat frame;

	if (!vidCam.isOpened())
	{
		cerr << "Application Failure: Video stream failed. Exiting now..." << endl;
		isRunning = false;
	}

	while (isRunning)
	{
		// Keep trying to write to buffer until we can push
		vidCam.read(frame);
		while (!frame.empty() && !qCap.push(frame) && isRunning);
	}
}


void videoProcess(void)
{
	Mat frameIn, frameOut;// frameOut(480, 640, CV_8UC1);
	std::vector<KeyPoint> centroids;

	// Background subtractor
	Ptr<BackgroundSubtractorMOG2> pBackSub = createBackgroundSubtractorMOG2();
	pBackSub->setBackgroundRatio(0.7);	// set to match Matlab
	pBackSub->setNMixtures(3); // set to match Matlab

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

	// Morphological elements
	Mat openStrel = getStructuringElement(cv::MORPH_RECT, Size(10, 10));
	Mat closeStrel = getStructuringElement(cv::MORPH_RECT, Size(20, 20));

	while (isRunning)
	{
		//------------------------------------------------------
		// Frame In	
		//------------------------------------------------------
		// Keep trying to write to buffer until we can push
		while (!qCap.pop(frameIn) && isRunning);

		//------------------------------------------------------
		// Process
		//------------------------------------------------------

		// First segment by color
		cv::inRange(frameIn, cv::Scalar(lowB, lowG, lowR), cv::Scalar(highB, highG, highR), frameOut);

		//// Segment the foreground from background
		//pBackSub->apply(frameOut, frameOut);

		//// Morpological open/close to remove noise
		//morphologyEx(frameOut, frameOut, MORPH_OPEN, openStrel);
		//morphologyEx(frameOut, frameOut, MORPH_CLOSE, closeStrel);

		//// Binary threshold with inversion. The background detector outputs a mask that has the background as 
		//// black, objects as white and shadows as gray. So turn objects + shadows to white and keep the background black.
		//// Then finally (using the binary_inv mode) invert everything for the blob detector (which finds black blobs on white)
		//threshold(frameOut, frameOut, 1, 255, THRESH_BINARY_INV);

		// Detect blobs / groups of related pixels and return their centroid
		pBlobDetector->detect(frameOut, centroids);

		drawKeypoints(frameIn, centroids, frameIn, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		drawKeypoints(frameOut, centroids, frameOut, Scalar(0, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		imshow("Cam0", frameIn);
		imshow("Cam1", frameOut);
		char k = (char)waitKey(1);
		if (k == 27 || k == 'q' || k == 'Q') isRunning = false;

		//------------------------------------------------------
		// Frame Out
		//------------------------------------------------------
		// Keep trying to write to buffer until we can push
		//while (!qProc.push(frameOut) && isRunning);
	}
}


void videoDisplay(void)
{
	Mat frame;
	char k;

	while (isRunning)
	{
		// Keep trying to read from buffer until we can pop
		//while (!qProc.pop(frame) && isRunning);

		//imshow("Cam", frame);
		//k = (char)waitKey(1);
		//if (k == 27 || k == 'q' || k == 'Q') isRunning = false;
	}
}

