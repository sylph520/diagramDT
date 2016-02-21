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
	//	cout << "image load error!" << endl;
	//	return -1;
	//}
	//namedWindow("original diagram show for test");
	//imshow("original diagram show for test", diagram_image);
	

	/* image bianarizing for next processing operation */
	Mat binarized_image;
	image_binarizing(diagram_image, binarized_image);
	//imshow("binarized image", binarized_image); // display the binarized image

	/* image labelling with binarized image */
	int nlabels; Mat label_matrix, labeled_image;
	vector<Mat> segmented_slice, segmented_blob;
	labeled_image = image_labelling(binarized_image, nlabels, label_matrix, segmented_slice);
	//imshow("colored segmented image", labeled_image);

	/* image segmenting, find the geometry diagram region and associating labelling letter region */
	// get the blob information of each labeled region
	segmenting_blob(segmented_slice, segmented_blob);//image segmenting
	
	//Here we assume the main geometry graph is of the most number of non-zero pixel
	sort(segmented_blob.begin(), segmented_blob.end(), nonzero_pixel_number_comp);
	Mat geometry_graph = segmented_blob[0];
	//imshow("geometry graph", geometry_graph);
	
	/** now begin primitive detection part **/
	vector<Vec4i> lines; vector<Vec3f> circles; vector<Vec2i> l_endpoints;
	vector<Vec2i> crosses; vector<Vec2i> ll_crosses; vector<Vec2i> lc_crosses;
	
	Mat color; cvtColor(geometry_graph, color, CV_GRAY2BGR);

	waitKey(0);
	return 0;
}

