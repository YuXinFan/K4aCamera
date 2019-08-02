#include "pch.h"

#include <vector>
#include <numeric>

#include "../K4aCamera/K4aCamera.h"
#include "../K4aCamera/Type.h"
#include "../K4aCamera/K4aConfig.h"
#include <opencv2/opencv.hpp>

class K4aCameraTest : public ::testing::Test {
protected:
	void SetUp() override {
		device_count = K4aCamera::DetectCameras();
		master = device_count - 1;
		for (size_t i = 0; i < device_count - 1; i++) {
			cameras.push_back(shared_ptr<K4aCamera>(new K4aCamera(i, false)));
		}
		cameras.push_back(shared_ptr<K4aCamera>(new K4aCamera(master, true)));
		K4aConfig config;
		config.image_format = IMAGE_FORMAT_COLOR_BGRA32;
		config.color_resolution = COLOR_RESOLUTION_720P;
		config.depth_mode = DEPTH_MODE_NFOV_2X2BINNED;
		config.capture_sync_only = true;
		config.fps = FRAMES_PER_SECOND_30;
		config.wired_sync_mode = WIRED_SYNC_MODE_SUBORDINATE;
		for (size_t i = 0; i < device_count - 1; i++) {
			cameras[i]->StartCamera(config);
		}
		K4aConfig config_master = config;
		config_master.wired_sync_mode = WIRED_SYNC_MODE_MASTER;
		cameras[master]->StartCamera(config_master);

	}
	void TearDown()  override {
		cameras.clear();
	}
	size_t device_count;
	std::vector<shared_ptr<K4aCamera>> cameras;
	size_t master;
};

TEST_F(K4aCameraTest, DetectCamera) {
	try {
		ASSERT_GE(device_count, 1);
	}
	catch (std::runtime_error& e) {
		ASSERT_STREQ("exception error", e.what());
	}
}

TEST_F(K4aCameraTest, NormalSetup) {
	try {
		for (size_t i = 0; i < device_count; i++) {
			/* color/depth enabled */
			ASSERT_TRUE(cameras[i]->IsOpened());
			ASSERT_TRUE(cameras[i]->IsDepthEnabled());
			ASSERT_TRUE(cameras[i]->IsColorEnabled());

			/* color/depth image size */
			ASSERT_EQ(cameras[i]->DepthWidth(), 320);
			ASSERT_EQ(cameras[i]->DepthHeight(), 288);
			ASSERT_EQ(cameras[i]->ColorWidth(), 1280);
			ASSERT_EQ(cameras[i]->ColorHeight(), 720);

			/* intrinsic */
#define INTRINSIC(i) (i==0?depthIn:colorIn)
			shared_ptr<Intrinsic> depthIn = cameras[i]->GetDepthIntrinsic();
			shared_ptr<Intrinsic> colorIn = cameras[i]->GetColorIntrinsic();

			ASSERT_EQ(depthIn->width, 320);
			ASSERT_EQ(depthIn->height, 288);
			ASSERT_EQ(colorIn->width, 1280);
			ASSERT_EQ(colorIn->height, 720);

			for (int j = 0; j < 2; ++j) {
				ASSERT_GE(INTRINSIC(j)->intrinsic[0], 100.f);
				ASSERT_FLOAT_EQ(INTRINSIC(j)->intrinsic[1], 0.f);
				ASSERT_GE(INTRINSIC(j)->intrinsic[2], 100.f);

				ASSERT_FLOAT_EQ(INTRINSIC(j)->intrinsic[3], 0.f);
				ASSERT_GE(INTRINSIC(j)->intrinsic[4], 100.f);
				ASSERT_GE(INTRINSIC(j)->intrinsic[5], 100.f);

				ASSERT_FLOAT_EQ(INTRINSIC(j)->intrinsic[6], 0.f);
				ASSERT_FLOAT_EQ(INTRINSIC(j)->intrinsic[7], 0.f);
				ASSERT_FLOAT_EQ(INTRINSIC(j)->intrinsic[8], 1.f);
			}

			/* extrinsic */
#define EXTRINSIC(i) (i==0?depthIn:colorIn)
			shared_ptr<Extrinsic> color2Depth = cameras[i]->GetColor2DepthExtrinsic();

			ASSERT_FLOAT_EQ(color2Depth->extrinsic[15], 1.f);
		}
	}
	catch (std::runtime_error& e) {
		ASSERT_STREQ("exception error", e.what());
	}
}

TEST_F(K4aCameraTest, FetchFrame) {
	size_t N = 100;
	size_t count = 0;
	std::vector<size_t> sync(device_count);
	size_t szExpectDepth = cameras[0]->DepthWidth() * cameras[0]->DepthHeight() * sizeof(uint16_t);
	size_t szExpectColor = cameras[0]->ColorWidth() * cameras[0]->ColorHeight() * sizeof(uint8_t) * 4;
	uint16_t* depthBuf = new uint16_t[szExpectDepth]();
	uint8_t* colorBuf = new uint8_t[szExpectColor]();
	size_t szDepth = 0;
	size_t szColor = 0;

	try {
		while (count++ < N) {
			/* fetch frames */
			for (size_t i = 0; i < device_count; ++i) {
				ASSERT_TRUE(cameras[i]->FetchFrame(colorBuf, szExpectColor, &szColor,
					depthBuf, szExpectDepth, &szDepth, &sync[i]));
				ASSERT_EQ(szColor, szExpectColor);
				ASSERT_EQ(szDepth, szExpectDepth);

				uint64_t sum = std::accumulate<uint16_t*, uint64_t>(depthBuf, depthBuf + szDepth, 0);
				ASSERT_GT(sum, 10000000);
				sum = std::accumulate<uint8_t*, uint64_t>(colorBuf, colorBuf + szColor, 0);
				ASSERT_GT(sum, 10000000);
			}

			/* Adjust frame queue size */
			size_t queueSz = cameras[0]->GetSyncQueueSize();
			for (int i = 0; queueSz > 2 && i < device_count; ++i) {
				cameras[i]->PopFrontSyncQueue(queueSz - 1);
			}

			ASSERT_EQ(sync, std::vector<size_t>(device_count, sync[0]));
		}
	}
	catch (std::runtime_error& e) {
		ASSERT_STREQ("exception error", e.what());
	}


}


TEST_F(K4aCameraTest, CVFetchFrame) {


	std::vector<std::uint8_t> color_buf = std::vector<uint8_t>(cameras[0]->ColorHeight() * cameras[0]->ColorWidth() * 4);
	std::vector<uint16_t> depth_buf = std::vector<uint16_t>(cameras[0]->DepthHeight() * cameras[0]->DepthWidth());
	size_t in_size_color = cameras[0]->ColorHeight() * cameras[0]->ColorWidth() * 4 * sizeof(uint8_t);
	size_t in_size_depth = cameras[0]->DepthHeight() * cameras[0]->DepthWidth() * sizeof(uint16_t);
	size_t out_size_color = 0;
	size_t out_size_depth = 0;

	std::vector<cv::Mat> color_images;
	std::vector<cv::Mat> depth_images;
	for (size_t i = 0; i < device_count; i++) {
		color_images.push_back(cv::Mat(cameras[0]->ColorHeight(), cameras[0]->ColorWidth(), CV_8UC4));
		depth_images.push_back(cv::Mat(cameras[0]->DepthHeight(), cameras[0]->DepthWidth(), CV_16UC1));
	}
	int frame_cout = 100;
	try {
		while (frame_cout-- > 0) {
			for (size_t i = 0; i < device_count; i++) {
				ASSERT_TRUE(cameras[i]->FetchFrame((uint8_t*)color_images[i].data, in_size_color, &out_size_color,
					(uint16_t*)depth_images[i].data, in_size_depth, &out_size_depth));

			}
			size_t queueSz = cameras[0]->GetSyncQueueSize();
			for (int i = 0; queueSz > 2 && i < device_count; ++i) {
				cameras[i]->PopFrontSyncQueue(queueSz - 1);
			}

		}
	}
	catch (std::runtime_error& e) {
		ASSERT_STREQ("exception error", e.what());
	}
}
int main(int argc, char** argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}