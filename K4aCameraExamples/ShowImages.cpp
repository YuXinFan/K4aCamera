#include "../K4aCamera/K4aCamera.h"
#include "../K4aCamera/K4aConfig.h"
#include "../K4aCamera/Type.h"

#include <thread>
#include <chrono>
#include <vector>

#include <conio.h>
#include <opencv2/opencv.hpp>

using namespace std;

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

cv::Mat Uint8ToColorMat(uint8_t* buffer, int width, int height) {

	cv::Mat mat = cv::Mat(height, width, CV_8UC4, buffer, cv::Mat::AUTO_STEP);
	return mat;
}

cv::Mat Uint16ToDepthMat(uint16_t* buffer, int width, int height) {

	return cv::Mat(height, width, CV_16UC1, buffer, cv::Mat::AUTO_STEP);
}


int example_main() {

	size_t device_amount = K4aCamera::DetectCameras();
	if (device_amount == static_cast<size_t>(0)) {
		cout << "[ERROR] No device founded !" << endl;
		exit(-1);
	}
	vector<unique_ptr<K4aCamera>> cameras;
	size_t master_idx = device_amount - 1;
	for (size_t i = 0; i < device_amount; i++) {
		if (i == master_idx) {
			cameras.push_back(unique_ptr<K4aCamera>(new K4aCamera(i, true)));
			continue;
		}
		cameras.push_back(unique_ptr<K4aCamera>(new K4aCamera(i, false)));
	}
	K4aConfig config;
	config.image_format = IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = COLOR_RESOLUTION_1080P;
	config.depth_mode = DEPTH_MODE_NFOV_UNBINNED;
	config.fps = FRAMES_PER_SECOND_30;
	config.capture_sync_only = true;
	config.depth_delay_off_color_usec = 0;
	config.wired_sync_mode = WIRED_SYNC_MODE_SUBORDINATE;

	/* Start camera, sub device must start before master  */
	for (size_t i = 0; i < device_amount; i++) {
		if (i == master_idx) {
			K4aConfig config_master = config;
			config_master.wired_sync_mode = WIRED_SYNC_MODE_MASTER;
			cameras[master_idx]->StartCamera(config_master);
			goto GETINTRINSIC;
		}
		cameras[i]->StartCamera(config);

	GETINTRINSIC:
		shared_ptr<Intrinsic> colorIntr = cameras[i]->GetColorIntrinsic();
		cv::Mat colorIntrMat(3, 3, CV_32F, colorIntr->intrinsic);
		shared_ptr<Intrinsic> depthIntr = cameras[i]->GetDepthIntrinsic();
		cv::Mat depthIntrMat(3, 3, CV_32F, depthIntr->intrinsic);
		shared_ptr<Extrinsic> Extrin = cameras[i]->GetColor2DepthExtrinsic();
		cv::Mat color2depthMat(4, 4, CV_32F, Extrin->extrinsic);
		std::stringstream ss;
		ss << (i + 1);
		std::string fileNameColorIntr = "C:\\Users\\zhuyu\\Desktop\\calibration\\ex-in-trinsics\\" + ss.str() + "\\" + "color_intrinsic.xml";
		std::string fileNameDepthIntr = "C:\\Users\\zhuyu\\Desktop\\calibration\\ex-in-trinsics\\" + ss.str() + "\\" + "depth_intrinsic.xml";
		std::string fileNameColor2Depth = "C:\\Users\\zhuyu\\Desktop\\calibration\\ex-in-trinsics\\" + ss.str() + "\\" + "color2depth.xml";

		cv::FileStorage fs1(fileNameColorIntr, cv::FileStorage::WRITE);
		fs1 << "M" << colorIntrMat;
		fs1.release();

		cv::FileStorage fs2(fileNameDepthIntr, cv::FileStorage::WRITE);
		fs2 << "M" << depthIntrMat;
		fs2.release();

		cv::FileStorage fs3(fileNameColor2Depth, cv::FileStorage::WRITE);
		fs3 << "M" << color2depthMat;
		fs3.release();
	}
	cout << "Camera open succeed!" << endl;


	/* set device exposure */
	for (size_t i = 0; i < device_amount; i++) {
		//cameras[i]->SetWhiteBalance(2500);
		//cameras[i]->SetExposure(4000);
	}
	/* start thread to get capture*/
	for (size_t i = 0; i < device_amount; i++) {
		//cameras[i]->StartCapture();
	}
	cout << "Start capturing" << endl;


	int frame_cout = 0;

	vector<uint8_t> color_buf = vector<uint8_t>(cameras[0]->ColorHeight() * cameras[0]->ColorWidth() * 4);
	vector<uint16_t> depth_buf = vector<uint16_t>(cameras[0]->DepthHeight() * cameras[0]->DepthWidth());
	size_t in_size_color = cameras[0]->ColorHeight() * cameras[0]->ColorWidth() * 4 * sizeof(uint8_t);
	size_t in_size_depth = cameras[0]->DepthHeight() * cameras[0]->DepthWidth() * sizeof(uint16_t);
	size_t out_size_color = 0;
	size_t out_size_depth = 0;


	std::chrono::duration<double> diff_time;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	vector<cv::Mat> color_images;
	vector<cv::Mat> depth_images;
	vector<cv::Mat> depth_images_toshow;
	for (size_t i = 0; i < device_amount; i++) {
		color_images.push_back(cv::Mat(cameras[0]->ColorHeight(), cameras[0]->ColorWidth(), CV_8UC4));
		depth_images.push_back(cv::Mat(cameras[0]->DepthHeight(), cameras[0]->DepthWidth(), CV_16UC1));
		depth_images_toshow.push_back(cv::Mat(cameras[0]->DepthWidth(), cameras[0]->DepthHeight(), CV_16UC1));
	}

	std::string baseColorPath = "C:\\Users\\zhuyu\\Desktop\\calibration\\human-pose\\color";
	std::string baseDepthPath = "C:\\Users\\zhuyu\\Desktop\\calibration\\human-pose\\depth";
	while (true) {
		if (_kbhit()) {
			char ch = _getch();
			if (ch == 27) { break; }
		}
		cout << "In round " << frame_cout << endl;
		std::chrono::high_resolution_clock::time_point last_time = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < device_amount; i++) {

			cameras[i]->FetchFrame((uint8_t*)color_images[i].data, in_size_color, &out_size_color,
				(uint16_t*)depth_images[i].data, in_size_depth, &out_size_depth);
			//transpose(depth_images[i], depth_images_toshow[i]);
			//flip(depth_images[i], depth_images[i], 1);
		}
		uint32_t frame_size = cameras[0]->GetSyncQueueSize();
		for (size_t i = 0; frame_size > 2 && i < device_amount; i++) {
			cameras[i]->PopFrontSyncQueue(frame_size - 1);
		}
		//if (_kbhit()) {
		//	char ch = _getch();
		//	if (ch) {
		//		
		//	}
		//}
		for (size_t i = 0; i < device_amount; i++) {
			std::stringstream id;
			id << (i + 1);
			char ss[10];
			sprintf_s(ss, "%04d", frame_cout);
			std::string fileName = "image.cam0" + id.str() + "_" + string(ss) + ".png";
			std::string fileColorPath = baseColorPath + "\\" + id.str() + "\\" + fileName;
			std::string fileDepthPath = baseDepthPath + "\\" + id.str() + "\\" + fileName;
			cv::imwrite(fileColorPath, color_images[i]);
			cv::imwrite(fileDepthPath, depth_images[i]);
		}
		frame_cout++;

		std::chrono::high_resolution_clock::time_point this_time = std::chrono::high_resolution_clock::now();
		diff_time = this_time - last_time;
		cout << "time diff is " << diff_time.count() << endl;
		//ShowManyImages("depths", depth_images);
		//ShowManyImages("Images", color_images);
	}
	cout << "End showing image" << endl;

	return 0;
}