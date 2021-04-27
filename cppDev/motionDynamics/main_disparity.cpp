//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/calib3d.hpp>
//#include <iostream>
//
//#define M_PI 3.14159265358979323846
//
//using namespace cv;
//using namespace std;
//
//int main()
//{
//	Mat disparity;
//	Mat imgL = imread("C:\\Users\\jplei\\Workspace\\opencvProjects\\tsukuba\\tsukuba_left.png", 0);
//	Mat imgR = imread("C:\\Users\\jplei\\Workspace\\opencvProjects\\tsukuba\\tsukuba_right.png", 0);
//
//	//imshow("left", imgL);
//	//imshow("right", imgR);
//	//waitKey(0);
//
//	Ptr<StereoBM> stereo = StereoBM::create(16, 15);
//	stereo->compute(imgL, imgR, disparity);
//
//
//	double fLen = 3.04;
//	double hFov = 62.2;
//	double vFov = 48.8;
//	double fPix = (imgL.rows / 2.0) / tan((hFov * M_PI / 180.0) / 2.0);
//	double baseline = 101.6; // 4inches / 101.6 mm (guess)
//
//	double bf = (baseline * fPix);
//
//	//Point objL(180, 195);
//	//Point objR(170, 195);
//	double disp;
//	disp = double(disparity.at<unsigned short>(175, 195));
//	double dist = bf / disp;
//	disp = double(disparity.at<unsigned short>(195, 175));
//	double dist2 = bf / disp;
//
//	std::cout << "dist " << dist << "\t" << "dist2 " << dist2 << std::endl;
//
//	// Output of stereo is a SIXTEEN BIT (16 bit) image, but imshow needs 8 bit images to display
//	disparity.convertTo(disparity, CV_8U);
//	imshow("Disparity", disparity);
//	waitKey(0);
//
//	//Mat bfd(imgL.rows, imgL.cols, CV_32F);
//
//	//double minVal, maxVal;
//	//Point minLoc, maxLoc;
//
//	//disparity.convertTo(disparity, CV_32F);
//	//minMaxLoc(disparity, &minVal, &maxVal, &minLoc, &maxLoc);
//	//bfd = bf * (1 / disparity);
//
//	//minMaxLoc(bfd, &minVal, &maxVal, &minLoc, &maxLoc);
//	//bfd = 255 * (bfd / maxVal);
//	//minMaxLoc(bfd, &minVal, &maxVal, &minLoc, &maxLoc);
//	//bfd.convertTo(bfd, CV_8U);
//
//	//imshow("bfd", bfd);
//	//waitKey(0);
//
//	return 0;
//}
//
////int main()
////{
////	cv::KeyPoint centL, centR;
////	Mat imgL = imread("C:\\Users\\jplei\\Workspace\\opencvProjects\\tsukuba\\scene1.row3.col1.png", 0);
////	Mat imgR = imread("C:\\Users\\jplei\\Workspace\\opencvProjects\\tsukuba\\scene1.row3.col2.png", 0);
////	
////	//Mat subImgL = imgL(cv::Range(140, 270), cv::Range(140, 220));
////	//Mat subImgR = imgR(cv::Range(140, 270), cv::Range(125, 215));
////	//imshow("left", subImgL);
////	//imshow("right", subImgR);
////	//waitKey(0);
////
////	//Mat imgL2 = cv::Mat::zeros(100, 100, CV_8UC1);
////	//Mat imgR2 = cv::Mat::zeros(100, 100, CV_8UC1);
////
////	//for (int i = 10; i < 30; i++) {
////	//	for (int j = 80; j < 98; j++) {
////	//		imgL2.at<unsigned char>(i, j) = 255;
////	//	}
////	//}
////	//imshow("left", imgL2);
////	//imshow("right", imgR2);
////	//waitKey(0);
////
////
////	double fLen = 3.04;
////	double hFov = 62.2;
////	double vFov = 48.8;
////
////
////	Point objL(180, 195);
////	Point objR(170, 195);
////
////
////	return 1;
////}