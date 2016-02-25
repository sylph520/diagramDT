#include "stdafx.h"
vector<Point2f> getPointPositions(Mat binaryImage)
{
	vector<Point2f> pointPositions;

	for (unsigned int y = 0; y < binaryImage.rows; ++y)
	{
		//unsigned char* rowPtr = binaryImage.ptr<unsigned char>(y);
		for (unsigned int x = 0; x < binaryImage.cols; ++x)
		{
			//if(rowPtr[x] > 0) pointPositions.push_back(Point2i(x,y));
			if (binaryImage.at<unsigned char>(y, x) > 0) pointPositions.push_back(Point2f(x, y));
		}
	}

	return pointPositions;
}
inline void getCircle(Point2f& p1, Point2f& p2, Point2f& p3, Point2f& center, float& radius)
{
	float x1 = p1.x;
	float x2 = p2.x;
	float x3 = p3.x;

	float y1 = p1.y;
	float y2 = p2.y;
	float y3 = p3.y;

	// PLEASE CHECK FOR TYPOS IN THE FORMULA :)
	center.x = (x1*x1 + y1*y1)*(y2 - y3) + (x2*x2 + y2*y2)*(y3 - y1) + (x3*x3 + y3*y3)*(y1 - y2);
	center.x /= (2 * (x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2));

	center.y = (x1*x1 + y1*y1)*(x3 - x2) + (x2*x2 + y2*y2)*(x1 - x3) + (x3*x3 + y3*y3)*(x2 - x1);
	center.y /= (2 * (x1*(y2 - y3) - y1*(x2 - x3) + x2*y3 - x3*y2));

	radius = sqrt((center.x - x1)*(center.x - x1) + (center.y - y1)*(center.y - y1));
}
float evaluateCircle(Mat dt, Point2f center, float radius)
{

	float completeDistance = 0.0f;
	int counter = 0;

	float maxDist = 1.0f;   //TODO: this might depend on the size of the circle!

	float minStep = 0.001f;
	// choose samples along the circle and count inlier percentage

	//HERE IS THE TRICK that no minimum/maximum circle is used, the number of generated points along the circle depends on the radius.
	// if this is too slow for you (e.g. too many points created for each circle), increase the step parameter, but only by factor so that it still depends on the radius

	// the parameter step depends on the circle size, otherwise small circles will create more inlier on the circle
	float step = 2 * 3.14159265359f / (6.0f * radius);
	if (step < minStep) step = minStep; // TODO: find a good value here.

	//for(float t =0; t<2*3.14159265359f; t+= 0.05f) // this one which doesnt depend on the radius, is much worse!
	for (float t = 0; t < 2 * 3.14159265359f; t += step)
	{
		float cX = radius*cos(t) + center.x;
		float cY = radius*sin(t) + center.y;

		if (cX < dt.cols)
			if (cX >= 0)
				if (cY < dt.rows)
					if (cY >= 0)
						if (dt.at<float>(cY, cX) <= maxDist)
						{
							completeDistance += dt.at<float>(cY, cX);
							counter++;
						}

	}

	return counter;
}

void detect_circle(Mat& color, Mat& mask, vector<Vec3f>& circles)
{
	unsigned int numberOfCirclesToDetect = 1;   // TODO: if unknown, you'll have to find some nice criteria to stop finding more (semi-) circles

	for (unsigned int j = 0; j < numberOfCirclesToDetect; ++j)
	{
		vector<Point2f> edgePositions;
		edgePositions = getPointPositions(mask);

		cout << "number of edge positions: " << edgePositions.size() << endl;

		// create distance transform to efficiently evaluate distance to nearest edge
		Mat dt;
		distanceTransform(255 - mask, dt, CV_DIST_L1, 3);

		unsigned int nIterations = 0;

		Point2f bestCircleCenter;
		float bestCircleRadius;
		//float bestCVal = FLT_MAX;
		float bestCVal = -1;

		//float minCircleRadius = 20.0f; // TODO: if you have some knowledge about your image you might be able to adjust the minimum circle radius parameter.
		float minCircleRadius = 0.0f;

		//TODO: implement some more intelligent ransac without fixed number of iterations
		for (unsigned int i = 0; i < 2000; ++i)
		{
			//RANSAC: randomly choose 3 point and create a circle:
			//TODO: choose randomly but more intelligent,
			//so that it is more likely to choose three points of a circle.
			//For example if there are many small circles, it is unlikely to randomly choose 3 points of the same circle.
			unsigned int idx1 = rand() % edgePositions.size();
			unsigned int idx2 = rand() % edgePositions.size();
			unsigned int idx3 = rand() % edgePositions.size();

			// we need 3 different samples:
			if (idx1 == idx2) continue;
			if (idx1 == idx3) continue;
			if (idx3 == idx2) continue;

			// create circle from 3 points:
			Point2f center; float radius;
			getCircle(edgePositions[idx1], edgePositions[idx2], edgePositions[idx3], center, radius);

			if (radius < minCircleRadius)continue;


			//verify or falsify the circle by inlier counting:
			//float cPerc = verifyCircle(dt,center,radius, inlierSet);
			float cVal = evaluateCircle(dt, center, radius);

			if (cVal > bestCVal)
			{
				bestCVal = cVal;
				bestCircleRadius = radius;
				bestCircleCenter = center;
			}

			++nIterations;
		}
		cout << "current best circle: " << bestCircleCenter << " with radius: " << bestCircleRadius << " and nInlier " << bestCVal << endl;
		Vec3f circle = { bestCircleCenter.x, bestCircleCenter.y, bestCircleRadius };
		circles.push_back(circle);
		cv::circle(color, bestCircleCenter, bestCircleRadius, Scalar(0, 0, 255));

		//TODO: hold and save the detected circle.

		//TODO: instead of overwriting the mask with a drawn circle it might be better to hold and ignore detected circles and dont count new circles which are too close to the old one.
		// in this current version the chosen radius to overwrite the mask is fixed and might remove parts of other circles too!

		// update mask: remove the detected circle!
		cv::circle(mask, bestCircleCenter, bestCircleRadius, 0, 5); // here the radius is fixed which isnt so nice.
	}

	namedWindow("edges"); imshow("edges", mask);
	namedWindow("color"); imshow("color", color);

	//imwrite("detectedCircles_c.png", color);
}

void detect_cirlce2_remove(Mat& colorgeo, Mat& bw, vector<Vec3f>& circles, Mat& nonC_geo)
{
	Mat gray;
	GaussianBlur(colorgeo, gray, Size(7, 7), 2, 2);
	//apply the hough transfrom  to find the circles
	HoughCircles(gray, circles, CV_HOUGH_GRADIENT, 1, colorgeo.rows / 8, 100, 50, 0, 0);
	cout << circles.size() << endl;
	// Draw the circles detected
	for (size_t i = 0; i < circles.size(); i++)
	{
		Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		// circle outline
		circle(gray, center, radius, Scalar(0, 0, 0), 3, 8, 0);
		circle(bw, center, radius, Scalar(0, 0, 0), 7, 8, 0);
	}
	namedWindow("Hough circle Transform", CV_WINDOW_AUTOSIZE);
	imshow("Hough circle Transform", gray);
	namedWindow("after", CV_WINDOW_AUTOSIZE);
	imshow("after", bw);


}
void parse_primitive(Mat &color, Mat &colorgeo, Mat& bw,Mat& nonC_geo, vector<Vec3f>& circles, vector<Vec4i>& lines)
{
	/*the function meant to parse the primitives in the geometry graph, here we adopt the strategy to detect from up to bottom*/
	Mat gray; cvtColor(color, gray, CV_RGB2GRAY);
	Mat mask = gray > 0;
	//detect_circle(color, bw, circles);
	detect_cirlce2_remove(colorgeo, bw, circles, nonC_geo);
	



}