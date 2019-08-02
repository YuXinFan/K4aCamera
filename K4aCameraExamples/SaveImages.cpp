#include "../K4aCamera/K4aCamera.h"
#include "../K4aCamera/K4aConfig.h"
#include "../K4aCamera/Type.h"

#include <thread>
#include <chrono>
#include <vector>

#include <conio.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main() {

	size_t device_amount = K4aCamera::DetectCameras();
	if (device_amount == static_cast<size_t>(0)) {
		std::cout << "[ERROR] No device founded !" << endl;
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
	std::cout << "Camera open succeed!" << endl;


	/* set device exposure */
	for (size_t i = 0; i < device_amount; i++) {
		//cameras[i]->SetWhiteBalance(2500);
		//cameras[i]->SetExposure(4000);
	}
	/* start thread to get capture*/
	for (size_t i = 0; i < device_amount; i++) {
		//cameras[i]->StartCapture();
	}
	std::cout << "Start capturing" << endl;


	int frame_cout = 0;

	vector<uint8_t> color_buf = vector<uint8_t>(cameras[0]->ColorHeight() * cameras[0]->ColorWidth() * 4);
	vector<uint16_t> depth_buf = vector<uint16_t>(cameras[0]->DepthHeight() * cameras[0]->DepthWidth());
	size_t in_size_color = cameras[0]->ColorHeight() * cameras[0]->ColorWidth() * 4 * sizeof(uint8_t);
	size_t in_size_depth = cameras[0]->DepthHeight() * cameras[0]->DepthWidth() * sizeof(uint16_t);
	size_t out_size_color = 0;
	size_t out_size_depth = 0;


	std::chrono::duration<double> diff_time;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	vector<vector<cv::Mat>> color_images;
	vector<vector<cv::Mat>> depth_images;

	size_t numpics = 300;
	for (size_t k = 0; k < device_amount; k++) {
		color_images.push_back(vector<cv::Mat>());
		depth_images.push_back(vector<cv::Mat>());
		for (size_t i = 0; i < numpics; i++) {
			color_images[k].push_back(cv::Mat(cameras[0]->ColorHeight(), cameras[0]->ColorWidth(), CV_8UC4));
			depth_images[k].push_back(cv::Mat(cameras[0]->DepthHeight(), cameras[0]->DepthWidth(), CV_16UC1));
		}
	}




	size_t frame_size = 0;
	while (frame_size < numpics) {
		if (_kbhit()) {
			char ch = _getch();
			if (ch == 27) { break; }
		}
		std::cout << "In round " << frame_cout << std::endl;
		std::chrono::high_resolution_clock::time_point last_time = std::chrono::high_resolution_clock::now();
		for (size_t i = 0; i < device_amount; i++) {
			cameras[i]->FetchFrame((uint8_t*)color_images[i][frame_size].data, in_size_color, &out_size_color,
				(uint16_t*)depth_images[i][frame_size].data, in_size_depth, &out_size_depth);
		}
		uint32_t frame_size = cameras[0]->GetSyncQueueSize();
		for (size_t i = 0; frame_size > 2 && i < device_amount; i++) {
			cameras[i]->PopFrontSyncQueue(frame_size - 1);
		}

		frame_size++;

		std::chrono::high_resolution_clock::time_point this_time = std::chrono::high_resolution_clock::now();
		diff_time = this_time - last_time;
		std::cout << "time diff is " << diff_time.count() << std::endl;
		//ShowManyImages("depths", depth_images);
		//ShowManyImages("Images", color_images);
	}

	std::string baseColorPath = "C:\\Users\\zhuyu\\Desktop\\calibration\\human-pose\\color";
	std::string baseDepthPath = "C:\\Users\\zhuyu\\Desktop\\calibration\\human-pose\\depth";
	for (size_t i = 0; i < device_amount; i++) {
		std::stringstream id;
		id << (i + 1);
		for (size_t j = 0; j < numpics; j++) {
			char ss[10];
			sprintf_s(ss, "%04d", j);
			std::string fileName = "image.cam0" + id.str() + "_" + string(ss) + ".png";
			std::string fileColorPath = baseColorPath + "\\" + id.str() + "\\" + fileName;
			std::string fileDepthPath = baseDepthPath + "\\" + id.str() + "\\" + fileName;
			cv::imwrite(fileColorPath, color_images[i][j]);
			cv::imwrite(fileDepthPath, depth_images[i][j]);
		}



	}

	std::cout << "End capture image" << endl;


	return 0;
}