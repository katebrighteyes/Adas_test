#include <iostream>	
#include <opencv2/opencv.hpp>

#include "stdafx.h"
#include "utils.h"
#include <math.h>
using namespace cv;
using namespace std;
#define PI 3.14

//Image size 960 720
int main()
{

	//Rotation 벡터, 매트릭스
	Mat Rvec, R_M, T_M;
	//Translation 벡터
	Mat Tvec;
	//내부,외부 파라미터
	Mat intrinsic, distCoeffs, extrinsic, projection, homography;
	Mat inverseHomography, inverseRotation;
	vector<Point3d> object_points;
	vector<Point2d> image_points, image_points_2, object_points_2;
	double Fx = 1.2651e+03;
	double Fy = 1.2752e+03;
	double Cx = 329.4638;
	double Cy = 239.7041;


	//내부파라미터
	FileStorage fs("Camera_info.xml", FileStorage::READ);
	fs["camera_matrix"] >> intrinsic;
	fs["distCoeffs"] >> distCoeffs;

	cout << "Intrinsic" << endl << intrinsic << endl;
	cout << "distCoeffs" << endl << distCoeffs << endl;

	//image plan 에서의 좌표
	//가운데
	image_points.push_back(Point2d(328, 355)); object_points.push_back(Point3d(15, 0, 0));
	image_points.push_back(Point2d(328, 363)); object_points.push_back(Point3d(14, 0, 0));
	image_points.push_back(Point2d(328, 372)); object_points.push_back(Point3d(13, 0, 0));
	image_points.push_back(Point2d(328, 384)); object_points.push_back(Point3d(12, 0, 0));
	image_points.push_back(Point2d(328, 397)); object_points.push_back(Point3d(11, 0, 0));
	image_points.push_back(Point2d(328, 413)); object_points.push_back(Point3d(10, 0, 0));
	image_points.push_back(Point2d(328, 431)); object_points.push_back(Point3d(9, 0, 0));
	image_points.push_back(Point2d(328, 455)); object_points.push_back(Point3d(8, 0, 0));
	image_points.push_back(Point2d(328, 480)); object_points.push_back(Point3d(7, 0, 0));
	//오른쪽
	image_points.push_back(Point2d(409, 355)); object_points.push_back(Point3d(15, 1, 0));
	image_points.push_back(Point2d(415, 363)); object_points.push_back(Point3d(14, 1, 0));
	image_points.push_back(Point2d(422, 372)); object_points.push_back(Point3d(13, 1, 0));
	image_points.push_back(Point2d(429, 384)); object_points.push_back(Point3d(12, 1, 0));
	image_points.push_back(Point2d(438, 397)); object_points.push_back(Point3d(11, 1, 0));
	image_points.push_back(Point2d(449, 413)); object_points.push_back(Point3d(10, 1, 0));
	image_points.push_back(Point2d(463, 431)); object_points.push_back(Point3d(9, 1, 0));
	image_points.push_back(Point2d(479, 455)); object_points.push_back(Point3d(8, 1, 0));
	image_points.push_back(Point2d(497, 480)); object_points.push_back(Point3d(7, 1, 0));
	//왼쪽
	image_points.push_back(Point2d(247, 355)); object_points.push_back(Point3d(15, -1, 0));
	image_points.push_back(Point2d(241, 363)); object_points.push_back(Point3d(14, -1, 0));
	image_points.push_back(Point2d(235, 372)); object_points.push_back(Point3d(13, -1, 0));
	image_points.push_back(Point2d(227, 384)); object_points.push_back(Point3d(12, -1, 0));
	image_points.push_back(Point2d(219, 397)); object_points.push_back(Point3d(11, -1, 0));
	image_points.push_back(Point2d(206, 413)); object_points.push_back(Point3d(10, -1, 0));
	image_points.push_back(Point2d(193, 431)); object_points.push_back(Point3d(9, -1, 0));
	image_points.push_back(Point2d(176, 455)); object_points.push_back(Point3d(8, -1, 0));
	image_points.push_back(Point2d(160, 480)); object_points.push_back(Point3d(7, -1, 0));


	//rotation , translation vector 생성
	solvePnP(object_points, image_points, intrinsic, Mat(), Rvec, Tvec);
	//cout << "Rvec"<<endl << Rvec << endl;
	cout << "Tvec" << endl << Tvec << endl << endl;

	//rotation vector -> rotation matrix 3x3
	Rodrigues(Rvec, R_M);
	cout << "Rotation Matrix" << endl << R_M << endl << endl;

	//inverseRotation 
	inverseRotation = R_M.inv();
	//cout << "inverseRotation" << endl << inverseRotation << endl;

	//extrinsic matrix 3x4
	hconcat(R_M, Tvec, extrinsic);
	cout << "Extrinsic Matrix" << endl << extrinsic << endl;

	//projection matrix 3x4
	projection = intrinsic * extrinsic;
	//cout << "Projection Matrix"<<endl << projection << endl;



	//homograpy 생성
	double p11 = projection.at<double>(0, 0),
		p12 = projection.at<double>(0, 1),
		p14 = projection.at<double>(0, 3),
		p21 = projection.at<double>(1, 0),
		p22 = projection.at<double>(1, 1),
		p24 = projection.at<double>(1, 3),
		p31 = projection.at<double>(2, 0),
		p32 = projection.at<double>(2, 1),
		p34 = projection.at<double>(2, 3);

	homography = (Mat_<double>(3, 3) << p11, p12, p14, p21, p22, p24, p31, p32, p34);
	cout << "Homography Matrix" << endl << homography << endl;

	//homography 역행렬 3x3
	inverseHomography = homography.inv();
	cout << "Inverse Homography" << endl << inverseHomography << endl;


	Mat point2D = (Mat_<double>(3, 1) << 328, 480, 1);
	cout << "point2D" << endl << point2D << endl;
	// 방법 1 기하학적 접근
	double pixel_x = 328;
	double pixel_y = 480;

	double u = (pixel_x - Cx) / Fx;
	double v = (pixel_y - Cy) / Fy;
	cout << "atan_22" << endl << atan(1) * 180 / PI << endl;
	double Camera_H = 1.4;
	cout << "atan" << endl << atan(v) * 180 / PI << endl;
	double Z = Camera_H * tan(PI / 2 - (atan(v)));
	cout << "tan" << endl << tan(0.78) << endl;
	cout << "Z" << endl << Z << endl;

	double D = sqrt(pow(Camera_H, 2) + pow(Z, 2));
	double J = sqrt(2 + pow(v, 2));
	double PP = u * D / J;
	double dist = sqrt(pow(Z, 2) + pow(PP, 2));
	double theta = atan2(PP, Z);
	double WX = dist * sin(theta);
	double WY = dist * cos(theta);

	cout << "WX" << endl << WX << endl;
	cout << "WY" << endl << WY << endl;

	//방법 2 호모그래피 이용

	printf("####################Solution 2######################\n ");
	Mat Homo;
	//가운데
	image_points_2.push_back(Point2d(328, 355)); object_points_2.push_back(Point2d(15, 0));
	image_points_2.push_back(Point2d(328, 363)); object_points_2.push_back(Point2d(14, 0));
	image_points_2.push_back(Point2d(328, 372)); object_points_2.push_back(Point2d(13, 0));
	image_points_2.push_back(Point2d(328, 384)); object_points_2.push_back(Point2d(12, 0));
	image_points_2.push_back(Point2d(328, 397)); object_points_2.push_back(Point2d(11, 0));
	image_points_2.push_back(Point2d(328, 413)); object_points_2.push_back(Point2d(10, 0));
	image_points_2.push_back(Point2d(328, 431)); object_points_2.push_back(Point2d(9, 0));
	image_points_2.push_back(Point2d(328, 455)); object_points_2.push_back(Point2d(8, 0));
	image_points_2.push_back(Point2d(328, 480)); object_points_2.push_back(Point2d(7, 0));
	//오른쪽
	image_points_2.push_back(Point2d(409, 355)); object_points_2.push_back(Point2d(15, 1));
	image_points_2.push_back(Point2d(415, 363)); object_points_2.push_back(Point2d(14, 1));
	image_points_2.push_back(Point2d(422, 372)); object_points_2.push_back(Point2d(13, 1));
	image_points_2.push_back(Point2d(429, 384)); object_points_2.push_back(Point2d(12, 1));
	image_points_2.push_back(Point2d(438, 397)); object_points_2.push_back(Point2d(11, 1));
	image_points_2.push_back(Point2d(449, 413)); object_points_2.push_back(Point2d(10, 1));
	image_points_2.push_back(Point2d(463, 431)); object_points_2.push_back(Point2d(9, 1));
	image_points_2.push_back(Point2d(479, 455)); object_points_2.push_back(Point2d(8, 1));
	image_points_2.push_back(Point2d(497, 480)); object_points_2.push_back(Point2d(7, 1));
	//왼쪽
	image_points_2.push_back(Point2d(247, 355)); object_points_2.push_back(Point2d(15, -1));
	image_points_2.push_back(Point2d(241, 363)); object_points_2.push_back(Point2d(14, -1));
	image_points_2.push_back(Point2d(235, 372)); object_points_2.push_back(Point2d(13, -1));
	image_points_2.push_back(Point2d(227, 384)); object_points_2.push_back(Point2d(12, -1));
	image_points_2.push_back(Point2d(219, 397)); object_points_2.push_back(Point2d(11, -1));
	image_points_2.push_back(Point2d(206, 413)); object_points_2.push_back(Point2d(10, -1));
	image_points_2.push_back(Point2d(193, 431)); object_points_2.push_back(Point2d(9, -1));
	image_points_2.push_back(Point2d(176, 455)); object_points_2.push_back(Point2d(8, -1));
	image_points_2.push_back(Point2d(160, 480)); object_points_2.push_back(Point2d(7, -1));

	Homo = findHomography(image_points_2, object_points_2, CV_RANSAC);

	cout << "homography_2" << endl << Homo << endl;


	Mat point2Dxy = (Mat_<double>(3, 1) << pixel_x, pixel_y, 1);
	cout << "point2Dxy" << endl << point2D << endl;



	Mat abc = Homo * point2Dxy;
	cout << "abc " << endl << abc << endl;
	double a = 0;
	a = abc.at<double>(0, 0);
	printf("a = %f \n", a);



	double b = abc.at<double>(1, 0);
	printf("b = %f\n", b);
	double ss = abc.at<double>(2, 0);
	printf("ss = %f \n", ss);

	double XW = a / ss;
	double YW = b / ss;
	printf("XW = %f   YW = %f\n", XW, YW);

	printf("####################Solution3##################\n");


	Mat Cc = (Mat_<double>(3, 1) << 0, 0, 0);
	Mat Pc = (Mat_<double>(3, 1) << u, v, 1);

	Mat Pw = inverseRotation * (Pc - Tvec);
	cout << "Pw" << endl << Pw << endl;
	Mat Cw = inverseRotation * (-Tvec);
	cout << "Cw" << endl << Cw << endl;

	cout << "Cw(2,0) =" << Cw.at<double>(2, 0) << endl;
	cout << "Pw(2,0) = " << Pw.at<double>(2, 0) << endl;

	double K = 0;
	K = Cw.at<double>(2, 0) / (Pw.at<double>(2, 0) - Cw.at<double>(2, 0));
	cout << "K = " << K << endl;

	Mat re = Pw - Cw;
	cout << "Pw - Cw" << endl << re << endl;
	Mat result = Cw + K * (Pw - Cw);
	cout << "Result " << endl << result << endl;

	Mat asdf = (Mat_<double>(3, 1) << 1.23123123, 1.1241291239, 1.12312491239);
	cout << "asdg" << asdf << endl;
	cout << "asdg = " << asdf.at<double>(2, 0) << endl;

	Mat point3D = inverseHomography * point2D;

	cout << " 3D " << endl << point3D << endl;

	double w = point3D.at<double>(2, 0);
	cout << "w" << endl << w << endl;

	Mat matPoint3D;
	divide(w, point3D, matPoint3D);

	cout << "coordinate" << endl << matPoint3D << endl;







	Mat tempMat, tempMat2;
	//scale factor , z 값
	double s, zConst = 0;
	tempMat = inverseRotation * intrinsic.inv() * point2D;
	cout << "Intrinsic_Inv" << intrinsic.inv() << endl;
	tempMat2 = inverseRotation *Tvec;

	s = zConst + tempMat2.at<double>(2, 0);
	s /= tempMat.at<double>(2, 0);

	Mat wcPoint = inverseRotation * (s*intrinsic.inv() * point2D - Tvec);

	Point3f realPoint(wcPoint.at<double>(0, 0), wcPoint.at<double>(1, 0), wcPoint.at<double>(2, 0));

	cout << " realPoint" << endl << realPoint << endl;



	//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@검출
	cv::VideoCapture cap;
	cap.open("CameraSensor_3.avi");

	CascadeClassifier cascade_fcw;
	cascade_fcw.load("cascade_black.xml");

	Mat frame;
	Mat frameGray;

	Rect roi(200, 200, 800, 400);

	while (true)
	{
		cap >> frame;
		if (frame.empty()) break;

		resize(frame, frame, Size(1280, 720));

		cvtColor(frame, frameGray, COLOR_BGR2GRAY);

		Size minSize(10, 10);
		Size maxSize(roi.width, roi.height);

		vector<Rect> locations;

		cascade_fcw.detectMultiScale(frameGray(roi), locations, 1.1, 5, 0 | CV_HAAR_SCALE_IMAGE);
		cout << "detections= " << locations.size() << std::endl;


		//printf("Location %f\n", locations);
		for (size_t i = 0; i < locations.size(); ++i) {
			locations[i].x += roi.x;
			locations[i].y += roi.y;
		}
		
		double image_x = (2 * locations[0].x + locations[0].width) / 2;
		double image_y = locations[0].y + locations[0].height;
		circle(frame, Point(image_x, image_y), 5, Scalar(255, 0, 0), 1);
		rectangle(frame, Point(locations[0].x, locations[0].y), Point(locations[0].x + locations[0].width, locations[0].y + locations[0].height),Scalar(0,255,0));

		cv::rectangle(frame, roi, cv::Scalar(255, 0, 0), 2);

		//@@@@@@@@@@@@@호모그래피 이용
		Mat point2Dxy = (Mat_<double>(3, 1) << image_x, image_y, 1);
		cout << "point2Dxy" << endl << point2D << endl;



		Mat abc = Homo * point2Dxy;
		cout << "abc " << endl << abc << endl;
		double a = 0;
		a = abc.at<double>(0, 0);
		printf("a = %f \n", a);



		double b = abc.at<double>(1, 0);
		printf("b = %f\n", b);
		double ss = abc.at<double>(2, 0);
		printf("ss = %f \n", ss);

		double XW = a / ss;
		double YW = b / ss;
		printf("XW = %f   YW = %f\n", XW, YW);
		int baseline;
		string str_dist = format("Homo - X =  %0.2lf  Y = %0.2lf",XW,YW);
		Size labelSize = getTextSize(str_dist, CV_FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
		putText(frame,str_dist,Point(image_x, image_y), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());
		imshow("FCW Forward Colission Warning]", frame);

		waitKey(30);
	}

	
}