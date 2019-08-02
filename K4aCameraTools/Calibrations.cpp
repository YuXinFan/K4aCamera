#include "../K4aCamera/K4aCamera.h"

#include <cstdio>
#include <iostream>
#include <vector>
#include <filesystem>

#include <conio.h>
#include <direct.h>
#include "opencv2/opencv.hpp"
#include "../K4aCamera/Type.h"

using namespace std;
namespace fs = std::filesystem;

/* Helper */

int ShowManyImages(string title, const vector<cv::Mat>& images, const int fct = 1)
{
	int size;
	// w - Maximum number of images in a row
	// h - Maximum number of images in a column
	int w, h;
	int nI = static_cast<int>(images.size());
	// If the number of images is lesser than 0 or greater than 12
	// return without displaying
	if (images.empty()) {
		cout << "nothing to display" << endl;
		return 0;
	}
	else if (nI > 12) {
		cout << "Number of images too large, can only handle maximally 12 images at a time ...\n" << endl;
		return 0;
	}
	// Determine the size of the image, and the number of rows/cols
	// from number of arguments
	else if (nI == 1) {
		w = h = 1;
		size = 1600;
	}
	else if (nI == 2) {
		w = 2; h = 1;
		size = 800;
	}
	else if (nI == 3 || nI == 4) {
		w = 2; h = 2;
		size = 800;
	}
	else if (nI == 5 || nI == 6) {
		w = 3; h = 2;
		size = 600;
	}
	else if (nI == 7 || nI == 8) {
		w = 4; h = 2;
		size = 600;
	}
	else {
		w = 4; h = 3;
		size = 450;
	}
	// Create a new 3 channel image
	cv::Mat DispImage = cv::Mat::zeros(cv::Size(100 + size * w, 60 + size * h), images[0].type());
	// Loop for nArgs number of arguments
	for (int i = 0, m = 20, n = 20; i < nI; i++, m += (20 + size)) {
		// Get the Pointer to the IplImage
		cv::Mat img = images[i];
		// Check whether it is NULL or not
		// If it is NULL, release the image, and return
		if (img.empty()) {
			cout << "Empty image";
			return 0;
		}
		// Find the width and height of the image
		int x = img.cols;
		int y = img.rows;
		// Find whether height or width is greater in order to resize the image
		int max = (x > y) ? x : y;
		// Find the scaling factor to resize the image
		float scale = (float)((float)max / size);
		// Used to Align the images
		if (i % w == 0 && m != 20) {
			m = 20;
			n += 20 + size;
		}
		// Set the image ROI to display the current image
		// Resize the input image and copy the it to the Single Big Image
		cv::Rect ROI(m, n, (int)(x / scale), (int)(y / scale));
		cv::Mat temp; resize(img, temp, cv::Size(ROI.width, ROI.height));
		temp *= fct;
		temp.copyTo(DispImage(ROI));
	}
	// Create a new window, and show the Single Big Image
	cv::imshow(title, DispImage);
	return cv::waitKey(1);
}


void CreateFolderSubFolders(const string& folder_path, size_t num) {
	fs::remove_all(folder_path);
	fs::create_directory(folder_path);
	for (size_t i = 0; i < num; i++) {
		stringstream folder_tag;
		folder_tag << (i + 1);
		string sub_folder = folder_path + folder_tag.str();
		fs::create_directory(sub_folder);
	}
}
enum {
	IMAGE_ONLY,
	TRINSIC_ONLY,
	IMAGE_TRINSIC
};

int main(int argc, char* argv[]) {
	uint32_t mode = 0;
	uint32_t image_num = 0;
	string image_folder_name{}, trinsic_folder_name{};
	bool auto_save = true;
	if ( argc == 4 ) {
		mode = IMAGE_ONLY;
		image_num = atoi(argv[2]);
		image_folder_name = string(argv[3]);
		if (strcmp(argv[1], "-AUTO")) {

		}
		else if (strcmp(argv[1], "-MANUAL")) {

		}
	}
	else if ( argc == 2 ) {
		mode = TRINSIC_ONLY;
		trinsic_folder_name = string(argv[1]);
	}
	else if (argc == 5 ) {
		mode = IMAGE_TRINSIC;
		image_num = atoi(argv[2]);
		image_folder_name = string(argv[3]);
		trinsic_folder_name = string(argv[4]);
	}
	else {
		cout << "Usage: # Record Image Only   # : exe -AUTO MAX_IMAGE_NUMBER IMAGE_FOLDER \n";
		cout << "       # Record Image Only   # : exe -MANUAL MAX_IMAGE_NUMBER IMAGE_FOLDER \n";
		cout << "       # Record Trinsic Only # : exe TRINSIC_FOLDER\n";
		cout << "       # Record Both         # : exe -AUTO MAX_IMAGE_NUMBER IMAGE_FOLDER TRINSIC_FOLDER\n";
		exit(0);
	}

	/* Check camera amount */
	size_t camera_amount = K4aCamera::DetectCameras();
	size_t master_idx;

	/* Create folder */
	if (mode == IMAGE_ONLY) {
		CreateFolderSubFolders(image_folder_name, camera_amount);
	}
	else if (mode == TRINSIC_ONLY) {
		CreateFolderSubFolders(trinsic_folder_name, camera_amount);
	}
	else {
		CreateFolderSubFolders(image_folder_name, camera_amount);
		CreateFolderSubFolders(trinsic_folder_name, camera_amount);
	}

	if (camera_amount == 0) {
		cout << "No camera found!" << endl;
		exit(-1);
	}
	else if (camera_amount == 1) {
		master_idx = 0;
	}
	else {
		master_idx = camera_amount - 1;
	}

	/* Init device */
	vector<unique_ptr<K4aCamera>> k4a_cameras;
	for (size_t i = 0; i < camera_amount; i++) {
		if (i == master_idx) {
			k4a_cameras.push_back(make_unique<K4aCamera>(i, true));
		}
		else {
			k4a_cameras.push_back(make_unique<K4aCamera>(i, false));
		}
	}
	/* Ensure that the master is started after all sub started*/
	/* Defeult 30fps, BGRA32, NFOV_UNBINNED, 1080P*/
	for (size_t i = 0; i < camera_amount; i++) {
		k4a_cameras[i]->StartCamera();
	}

	/* Get the intrinsic of color */
	/* Get the intrinsic of depth */
	/* Get the extrinsic of color to depth */
	std::string file_path_base;
	for (size_t i = 0; i < camera_amount; i++) {
		stringstream folder_tag;
		folder_tag << (i + 1);
		shared_ptr<Intrinsic> depth_in = k4a_cameras[i]->GetDepthIntrinsic();
		cv::Mat depth_in_mat(3, 3, CV_32F, depth_in->intrinsic);
		string depth_in_path_name = trinsic_folder_name + "\\" + folder_tag.str() + "depth_intrinsic.xml";
		cv::FileStorage depth_xml(depth_in_path_name, cv::FileStorage::WRITE);
		depth_xml << depth_in_mat;
		depth_xml.release();

		shared_ptr<Intrinsic> color_in = k4a_cameras[i]->GetColorIntrinsic();
		cv::Mat color_in_mat(3, 3, CV_32F, color_in->intrinsic);
		string color_in_path_name = trinsic_folder_name + "\\" + folder_tag.str() + "color_intrinsic.xml";
		cv::FileStorage color_xml(color_in_path_name, cv::FileStorage::WRITE);
		color_xml << color_in_mat;
		color_xml.release();

		shared_ptr<Extrinsic> color_to_depth_ex = k4a_cameras[i]->GetColor2DepthExtrinsic();
		cv::Mat color_to_depth_ex_mat(3, 3, CV_32F, color_to_depth_ex->extrinsic);
		string color_to_depth_ex_path_name = trinsic_folder_name + "\\" + folder_tag.str() + "color2depth.xml";
		cv::FileStorage color_to_depth_xml(color_to_depth_ex_path_name, cv::FileStorage::WRITE);
		color_to_depth_xml << color_to_depth_ex_mat;
		color_to_depth_xml.release();
	}

	if (mode == TRINSIC_ONLY) {
		return 0;
	}

	/* Record color and depth images */
	vector<uint8_t> color_buf = vector<uint8_t>(k4a_cameras[0]->ColorHeight() * k4a_cameras[0]->ColorWidth() * 4);
	vector<uint16_t> depth_buf = vector<uint16_t>(k4a_cameras[0]->DepthHeight() * k4a_cameras[0]->DepthWidth());
	size_t in_size_color = k4a_cameras[0]->ColorHeight() * k4a_cameras[0]->ColorWidth() * 4 * sizeof(uint8_t);
	size_t in_size_depth = k4a_cameras[0]->DepthHeight() * k4a_cameras[0]->DepthWidth() * sizeof(uint16_t);
	size_t out_size_color = 0;
	size_t out_size_depth = 0;

	vector<vector<cv::Mat>> color_images;
	vector<vector<cv::Mat>> depth_images;

	size_t numpics = image_num;
	for (size_t k = 0; k < camera_amount; k++) {
		color_images.push_back(vector<cv::Mat>());
		depth_images.push_back(vector<cv::Mat>());
		for (size_t i = 0; i < numpics; i++) {
			color_images[k].push_back(cv::Mat(k4a_cameras[0]->ColorHeight(), k4a_cameras[0]->ColorWidth(), CV_8UC4));
			depth_images[k].push_back(cv::Mat(k4a_cameras[0]->DepthHeight(), k4a_cameras[0]->DepthWidth(), CV_16UC1));
		}
	}

	size_t image_selected = 0;
	while (image_selected < numpics) {
		
		/* Add interact for manual ESC end */
		if (_kbhit()) {
			char ch = _getch();
			if (ch == 27) { break; }
		}

		/* Read image from camera */
		for (size_t i = 0; i < camera_amount; i++) {
			k4a_cameras[i]->FetchFrame((uint8_t*)color_images[i][image_selected].data, in_size_color, &out_size_color,
				(uint16_t*)depth_images[i][image_selected].data, in_size_depth, &out_size_depth);
		}

		/* Write color depth image into file*/
		if ( auto_save || (_kbhit() && (!auto_save)) ) {
			char cnt[10];
			sprintf_s(cnt, "%zd", image_selected);
			for (size_t i = 0; i < camera_amount; i++) {
				std::stringstream folder_tag;
				folder_tag << (i + 1);
				string color_path = image_folder_name + "color\\" + folder_tag.str() + "\\";
				string color_name = "image.cam0" + folder_tag.str() + "_" + string(cnt) + ".jpg";

				string depth_path = image_folder_name + "depth\\" + folder_tag.str() + "\\";
				string depth_name = "image.cam0" + folder_tag.str() + "_" + string(cnt) + ".png";
				cv::imwrite(color_path + color_name, color_images[i][image_selected]);
				cv::imwrite(depth_path + depth_name, depth_images[i][image_selected]);
			}
			image_selected++;
		}
		size_t frame_size = k4a_cameras[0]->GetSyncQueueSize();
		for (size_t i = 0; frame_size > 2 && i < camera_amount; i++) {
			k4a_cameras[i]->PopFrontSyncQueue(frame_size - 1);
		}
		//ShowManyImages("Images", color_images);
	}



	return 0;
}