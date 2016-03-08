// diagramDT.cpp : Defines the entry point for the console application.
//
/*
This project is meant for the geometry element detectation task with a input diagram picture
*/
#include "stdafx.h"
#include "diagram_importing.h"
#include "diagram_processing.h"
#include "diagram_parsing.h"

int _tmain(int argc, _TCHAR* argv[])
{
	/*** Start stage ***/
	
	// get the input diagram picture to be the original image
	
	char diagram_file[200];
	diagram_file_importing(diagram_file);
	
	///*test for the image importing*/(pass)
	//cout << diagram_file << endl;

	/*** Image Processing and geo-primitives detection stage ***/

	/** first start image processing part **/
	// read the original image
	Mat diagram_image = imread(diagram_file, 0);
	///*test origianl image loading*/
	//if (diagram_image.empty())
	//{
	//	std::cout << "image load error!" << endl;
	//	return -1;
	//}
	//namedWindow("original diagram show for test");
	//imshow("original diagram show for test", diagram_image);
	

	/* image bianarizing for next processing operation */
	Mat binarized_image;
	image_binarizing(diagram_image, binarized_image);
	//imshow("binarized image", binarized_image); // display the binarized image
	/* image labelling with binarized image */
	//labeled_image is a color image
	int nlabels; Mat label_matrix, labeled_image, log_background, background;
	vector<Mat> segmented_slice, segmented_blob;
	labeled_image = image_labelling(binarized_image, nlabels, label_matrix, segmented_slice);
	//imshow("colored segmented image", labeled_image);
	/* image segmenting, find the geometry diagram region and associating labelling letter region */
	// get the blob information of each labeled region
	//segmenting_blob(segmented_slice, segmented_blob);//image segmenting
	//Here we assume the main geometry graph is of the most number of non-zero pixel
	
	//sort(segmented_blob.begin(), segmented_blob.end(), nonzero_pixel_number_comp);
	sort(segmented_slice.begin(), segmented_slice.end(), nonzero_pixel_number_comp);
	//imshow("slice 0", segmented_slice[0]);
	//imshow("test2", diagram_image.mul(segmented_slice[0] / 255));
	//gray geo should be small gray number from ori plus the background
	//background = diagram_image;//initialize
	//for (size_t i = 1; i < segmented_slice.size(); i++)
	//{
	//	background = background - diagram_image.mul(segmented_slice[i]);
	//}
	//imshow("test",255*diagram_image.mul(segmented_slice[2]));
	//cout << segmented_slice[1];
	//cout << diagram_image.mul(segmented_slice[1]);
	//imshow("background", background);
	Mat all_white = 255 * Mat::ones(diagram_image.rows,diagram_image.cols,diagram_image.type());
	Mat graygeo = diagram_image.mul(segmented_slice[0] / 255) + all_white.mul(255 - segmented_slice[0]);
	//cout << countNonZero(diagram_image.mul(segmented_slice[0]));
	//Mat colorgeo_blob; cvtColor(graygeo_blob,colorgeo_blob, CV_GRAY2BGR);
	//Mat graygeo_blob = graygeo(boundingRect(segmented_slice[0]));
	
	//imshow("gray geo", graygeo);
	//cout << graygeo << endl;
	//imshow("graygeo blob", graygeo_blob);
	/*xml save tamplates*/
	//FileStorage fs(".\\graygeo.xml", FileStorage::WRITE);
	//fs << "graygeo" << graygeo;
	//fs.release();
	//namedWindow("ts", 1);
	//imshow("ts", graygeo);

	Mat geometry_graph_bw = segmented_slice[0];
	//imshow("geometry graph", geometry_graph_bw);
	
	/** now begin primitive detection part **/
	vector<Vec4i> lines; vector<Vec3f> circles; vector<spe_point> points;
	Mat colorgeo; cvtColor(graygeo, colorgeo, CV_GRAY2BGR);
	//imshow("original color blob ",colorgeo_blob);

	// head down to bottom detect sequence 
	parse_primitive(colorgeo, graygeo, geometry_graph_bw, circles, lines, points);

	/*imwrite tamplates*/
	//vector<int> quality;
	//quality.push_back(CV_IMWRITE_JPEG_QUALITY);
	//quality.push_back(100);
	//imwrite("geograph.jpg", geometry_graph_bw,quality );
	
	/*xml save templates*/
	//FileStorage fs(".\\geograph.xml", FileStorage::WRITE);
	//fs << "geograph" << geograph;
	//fs.release();
	/*xml read templates*/
	//FileStorage fs(".\\geograph.xml", FileStorage::READ);
	//Mat geograph; fs["geograph"] >> geograph;
	//fs.release();

	waitKey(0);
	return 0;
}

