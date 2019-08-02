#include "K4aCameraImpl.h"

#include <algorithm>			/* min and max */
#include <iostream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include "Type.h"

static std::mutex ioMtx;

std::vector<k4a_device_t> K4aCameraImpl::_selected;


shared_ptr<Intrinsic> GetIntrinsic(const k4a_calibration_camera_t& calib) {
	shared_ptr<Intrinsic> intrin = std::make_shared<Intrinsic>();

	intrin->width = calib.resolution_width;
	intrin->height = calib.resolution_height;

	/* add intrinsic */
	k4a_calibration_intrinsic_parameters_t param = calib.intrinsics.parameters;
	intrin->intrinsic[0] = param.param.fx;
	intrin->intrinsic[1] = 0;
	intrin->intrinsic[2] = param.param.cx;
	intrin->intrinsic[3] = 0;
	intrin->intrinsic[4] = param.param.fy;
	intrin->intrinsic[5] = param.param.cy;
	intrin->intrinsic[6] = 0;
	intrin->intrinsic[7] = 0;
	intrin->intrinsic[8] = 1;

	/* add distortion */
	intrin->distortion[0] = param.param.k1;
	intrin->distortion[1] = param.param.k2;
	intrin->distortion[2] = param.param.p1;
	intrin->distortion[3] = param.param.p2;
	intrin->distortion[4] = param.param.k3;
	intrin->distortion[5] = param.param.k4;
	intrin->distortion[6] = param.param.k5;
	intrin->distortion[7] = param.param.k6;
	intrin->distortion[8] = 0;
	intrin->distortion[9] = 0;
	intrin->distortion[10] = 0;
	intrin->distortion[11] = 0;
	return intrin;
}


void InitUndistortMap(shared_ptr<cv::Mat>& map1, shared_ptr<cv::Mat>& map2, const k4a_calibration_camera_t& calib) {
	shared_ptr<Intrinsic> Intr = GetIntrinsic(calib);
	cv::Mat cameraMatrix(3, 3, CV_32F, Intr->intrinsic);
	cv::Mat distcoeff(8, 1, CV_32F, &Intr->distortion[0]);
	cv::Size imgSize(calib.resolution_width, calib.resolution_height);
	cv::initUndistortRectifyMap(cameraMatrix, distcoeff, cv::Mat(), cameraMatrix,
		imgSize, CV_16SC2, *(map1.get()), *(map2.get()));
}

void K4aCameraImpl::StartCamera() {
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.camera_fps = K4A_FRAMES_PER_SECOND_30;

	config.wired_sync_mode = K4A_WIRED_SYNC_MODE_SUBORDINATE;
	if (_master) {
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_MASTER;
	}

	config.synchronized_images_only = true;
	config.depth_delay_off_color_usec = 0;
	config.subordinate_delay_off_master_usec = 0;
	config.disable_streaming_indicator = false;

	if (!K4A_RESULT_SUCCEEDED == k4a_device_start_cameras(_hDev, &config)) {
		std::cout << "Fait to start camera of device " << _deviceIdx << " !" << std::endl;
		exit(-1);
	}
	/* set depth and color enable */
	_colorEnabled = true;
	_depthEnabled = true;

	/* depth calibration interinsic exterinsic */
	k4a_device_get_calibration(_hDev, config.depth_mode, config.color_resolution, &_deviceCalib);
	_depthCalib = _deviceCalib.depth_camera_calibration;

	/* color calibration interinsic exterinsic */
	_colorCalib = _deviceCalib.color_camera_calibration;

	/* Initialize undistortion maps */
	try {
		InitUndistortMap(_map1Color, _map2Color, _colorCalib);
		InitUndistortMap(_map1Depth, _map2Depth, _depthCalib);
	}
	catch (std::runtime_error& e) {
		std::cerr << "[ERROR] StartCamera: " << e.what() << std::endl;
		exit(-1);
	}
	_isOpened = true;
	_capture = true;
	_captureTrd = std::thread(&K4aCameraImpl::CaptureThread, this, std::ref(_frames), std::ref(_capture), std::ref(_idx), std::ref(_frameMtx), std::ref(_frameCV));
	return;
}

void K4aCameraImpl::StartCamera(const K4aConfig& cam_config) {
	/* configure components, try to enable depth and color camera */
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	config.color_format = static_cast<k4a_image_format_t>(cam_config.image_format);
	config.color_resolution = static_cast<k4a_color_resolution_t>(cam_config.color_resolution);
	config.depth_mode = static_cast<k4a_depth_mode_t>(cam_config.depth_mode);
	config.camera_fps = static_cast<k4a_fps_t>(cam_config.fps);
	config.wired_sync_mode = static_cast<k4a_wired_sync_mode_t>(cam_config.wired_sync_mode);

	config.synchronized_images_only = true;
	config.depth_delay_off_color_usec = 0;
	config.subordinate_delay_off_master_usec = 0;
	config.disable_streaming_indicator = false;

	if (!K4A_RESULT_SUCCEEDED == k4a_device_start_cameras(_hDev, &config)) {
		std::cout << "Fail to start camera of device " << _deviceIdx << " !" << std::endl;
		exit(-1);
	}
	/* set depth and color enable */
	_colorEnabled = false;
	if (cam_config.color_resolution != COLOR_RESOLUTION_OFF) {
		_colorEnabled = true;
	}
	_depthEnabled = false;
	if (cam_config.depth_mode != DEPTH_MODE_OFF) {
		_depthEnabled = true;
	}
	/* depth calibration interinsic exterinsic */
	if (!K4A_RESULT_SUCCEEDED == k4a_device_get_calibration(_hDev, config.depth_mode, config.color_resolution, &_deviceCalib)) {
		std::cout << "Fail to get camera calibration of of device " << _deviceIdx << " !" << std::endl;
		exit(-1);
	}
	_depthCalib = _deviceCalib.depth_camera_calibration;

	/* color calibration interinsic exterinsic */
	_colorCalib = _deviceCalib.color_camera_calibration;

	/* Initialize undistortion maps */
	try {
		InitUndistortMap(_map1Color, _map2Color, _colorCalib);
		InitUndistortMap(_map1Depth, _map2Depth, _depthCalib);
	}
	catch (std::runtime_error& e) {
		std::cerr << "[ERROR] StartCamera: " << e.what() << std::endl;
		exit(-1);
	}

	_isOpened = true;
	_capture = true;
	_captureTrd = std::thread(&K4aCameraImpl::CaptureThread, this, std::ref(_frames), std::ref(_capture), std::ref(_idx), std::ref(_frameMtx), std::ref(_frameCV));
	return;
}

void K4aCameraImpl::StopCamera() {
	StopCapture();
	k4a_device_stop_cameras(_hDev);
}

size_t K4aCameraImpl::DetectCameras() {
	size_t count = k4a_device_get_installed_count();
	_selected.resize(count);
	return count;
}

bool K4aCameraImpl::IsMaster() {
	if (_master) {
		return true;
	}
	bool sync_in;
	bool sync_out;
	k4a_device_get_sync_jack(_hDev, &sync_in, &sync_out);
	bool is_master = (sync_out && (!sync_in));
	_master = is_master;
	return is_master;
}

K4aCameraImpl::K4aCameraImpl() :
	_idx(0),
	_isOpened(false),
	_sn(""),
	_hDev(nullptr),
	_scaleUnit(),
	_depthCalib(),
	_colorCalib(),
	_imageBufColor(nullptr),
	_map1Color(new cv::Mat()),
	_map2Color(new cv::Mat()),
	_imageBufDepth(nullptr),
	_map1Depth(new cv::Mat()),
	_map2Depth(new cv::Mat()),
	_master(false),
	_deviceIdx(-1),
	_capture(false),
	_captureTrd(),
	_frames(),
	_frameMtx(),
	_frameCV(),
	_rGain(1.f),
	_gGain(1.f),
	_bGain(1.f) {
}

/*
open device
*/
K4aCameraImpl::K4aCameraImpl(const size_t id, const bool master) :
	_idx(0),
	_isOpened(false),
	_scaleUnit(),
	_depthCalib(),
	_colorCalib(),
	_imageBufColor(nullptr),
	_map1Color(new cv::Mat()),
	_map2Color(new cv::Mat()),
	_imageBufDepth(nullptr),
	_map1Depth(new cv::Mat()),
	_map2Depth(new cv::Mat()),
	_deviceIdx(-1),
	_capture(false),
	_captureTrd(),
	_frames(),
	_frameMtx(),
	_frameCV(),
	_rGain(1.f),
	_gGain(1.f),
	_bGain(1.f) {
	/* open device of current id */
	if (!K4A_RESULT_SUCCEEDED == k4a_device_open(id, &_hDev)) {
		std::cout << "Fail to open device of index " << id << " !" << std::endl;
		exit(-1);
	}

	_deviceIdx = id;
	_master = master;

	/* set serial number */
	char* serial_number = nullptr;
	size_t sn_size = 0;
	if (!K4A_BUFFER_RESULT_SUCCEEDED == k4a_device_get_serialnum(_hDev, nullptr, &sn_size)) {
		std::cout << "Fail to get serial number of device " << id << " !" << std::endl;
		exit(-1);
	}
	if (sn_size > 32) {
		std::cout << "serial number too long to store" << std::endl;
	}
	else {
		k4a_device_get_serialnum(_hDev, &_sn[0], &sn_size);
		std::cout << "Serial number of device " << id << " is " << _sn << std::endl;
	}
}
/*
close device
*/
K4aCameraImpl::~K4aCameraImpl() {
	StopCapture();
	k4a_device_stop_cameras(_hDev);
	k4a_device_close(_hDev);
}

void K4aCameraImpl::StopCapture() {
	if (_isOpened) {
		_capture = false;
		if (_captureTrd.joinable()) {
			_captureTrd.join();
		}
	}
}

int K4aCameraImpl::DepthHeight() const {
	if (!_isOpened) {
		return 0;
	}
	return _depthCalib.resolution_height;
}

int K4aCameraImpl::DepthWidth() const {
	if (!_isOpened) {
		return 0;
	}
	return _depthCalib.resolution_width;
}

int K4aCameraImpl::ColorHeight() const {
	if (!_isOpened) {
		return 0;
	}
	return _colorCalib.resolution_height;
}

int K4aCameraImpl::ColorWidth() const {
	if (!_isOpened) {
		return 0;
	}
	return _colorCalib.resolution_width;
}

shared_ptr<Intrinsic> K4aCameraImpl::GetDepthIntrinsic() const {
	if (!_isOpened) {
		return nullptr;
	}
	return GetIntrinsic(_depthCalib);
}

shared_ptr<Intrinsic> K4aCameraImpl::GetColorIntrinsic() const {
	if (!_isOpened) {
		return nullptr;
	}
	return GetIntrinsic(_colorCalib);
}

size_t K4aCameraImpl::GetSyncQueueSize() {
	std::lock_guard<std::mutex> lk(_frameMtx);
	return _frames.size();
}

void K4aCameraImpl::PopFrontSyncQueue(const size_t sz) {
	std::lock_guard<std::mutex> lk(_frameMtx);
	_frames.erase(_frames.begin(), _frames.begin() + sz);
	return;
}

void K4aCameraImpl::SetWhiteBalance(const int32_t& degreesKelvin) {
	uint32_t mod = degreesKelvin - (degreesKelvin % 10);
	k4a_device_set_color_control(_hDev,
		K4A_COLOR_CONTROL_WHITEBALANCE,
		K4A_COLOR_CONTROL_MODE_MANUAL,
		mod);
}

void K4aCameraImpl::SetExposure(const uint32_t& microsecond) {
	k4a_device_set_color_control(_hDev,
		K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE,
		K4A_COLOR_CONTROL_MODE_MANUAL,
		microsecond);
}

void K4aCameraImpl::SetGain(const int32_t& g) {
	k4a_device_set_color_control(_hDev,
		K4A_COLOR_CONTROL_GAIN,
		K4A_COLOR_CONTROL_MODE_MANUAL,
		g);
}
shared_ptr<Extrinsic> K4aCameraImpl::GetColor2DepthExtrinsic() const {
	if (!_isOpened) {
		return nullptr;
	}
	k4a_calibration_extrinsics_t ex_param = _colorCalib.extrinsics;

	shared_ptr<Extrinsic> extrin(new Extrinsic);
	extrin->extrinsic[0] = ex_param.rotation[0];
	extrin->extrinsic[1] = ex_param.rotation[1];
	extrin->extrinsic[2] = ex_param.rotation[2];
	extrin->extrinsic[3] = ex_param.translation[0];
	extrin->extrinsic[4] = ex_param.rotation[3];
	extrin->extrinsic[5] = ex_param.rotation[4];
	extrin->extrinsic[6] = ex_param.rotation[5];
	extrin->extrinsic[7] = ex_param.translation[1];
	extrin->extrinsic[8] = ex_param.rotation[6];
	extrin->extrinsic[9] = ex_param.rotation[7];
	extrin->extrinsic[10] = ex_param.rotation[8];
	extrin->extrinsic[11] = ex_param.translation[2];
	extrin->extrinsic[12] = 0;
	extrin->extrinsic[13] = 0;
	extrin->extrinsic[14] = 0;
	extrin->extrinsic[15] = 1;
	return extrin;
}

static std::mutex sCommandMtx;
static std::mutex sPlayerMtx;
std::mutex sSizeMtx;
static std::condition_variable sCommandCV;
static std::condition_variable sPlayerCV;
std::condition_variable sSizeCV;
static int sPlayers = 0;
static std::vector<bool> sRun(8, false);

void K4aCameraImpl::CaptureThread(std::deque<Frame>& frames,
	std::atomic_bool& capture,
	size_t& idx,
	std::mutex& frameMtx,
	std::condition_variable& frameCV) {
	std::this_thread::sleep_for(std::chrono::seconds(2));
	while (capture) {
		if (_master) {
			// The commander will wait until all players are ready
			std::unique_lock<std::mutex> lk(sCommandMtx);
			if (!sCommandCV.wait_for(lk, std::chrono::seconds(3), []() { return sPlayers == _selected.size() - 1; })) {
				std::cerr << "[ERROR] FetchThread: Commander wait_for timeout" << std::endl;
				break;
			}
			sPlayers = 0;
			lk.unlock();

			// and give the running signal
			{
				std::lock_guard<std::mutex> lk(sPlayerMtx);
				for (auto&& r : sRun) r = true;
			}
			sPlayerCV.notify_all();
		}
		else {
			// Players will notify the commander he is ready
			{
				std::lock_guard<std::mutex> lk(sCommandMtx);
				sPlayers++;
			}
			sCommandCV.notify_one();

			// and wait for the running signal
			std::unique_lock<std::mutex> lk(sPlayerMtx);
			if (!sPlayerCV.wait_for(lk, std::chrono::seconds(3), [this]() { return sRun[_deviceIdx]; })) {
				std::cerr << "[ERROR] FetchThread: Player wait_for timeout" << std::endl;
				break;
			}
			sRun[_deviceIdx] = false;
			lk.unlock();
		}

		k4a_capture_t cpt;
		try {
			if (!K4A_WAIT_RESULT_SUCCEEDED == k4a_device_get_capture(_hDev, &cpt, K4A_WAIT_INFINITE)) {
				std::cerr << "[ERROR] CaptureFrame: Fail to get capture of device " << _deviceIdx << " !" << std::endl;
				exit(-1);
			}
		}
		catch (std::runtime_error& e) {
			std::cerr << "[ERROR] CaptureFrame: " << e.what() << std::endl;
			break;
		}
		idx++;

		cv::Mat depth;
		cv::Mat color;

		/* Get depth image */
		k4a_image_t depth_image = k4a_capture_get_depth_image(cpt);
		uint8_t* depth_buf = k4a_image_get_buffer(depth_image);
		depth = cv::Mat(k4a_image_get_height_pixels(depth_image),
			k4a_image_get_width_pixels(depth_image),
			CV_16UC1, depth_buf, cv::Mat::AUTO_STEP);

		/* Get color image */
		size_t color_height = ColorHeight();
		size_t color_width = ColorWidth();
		k4a_image_t color_image = k4a_capture_get_color_image(cpt);
		uint8_t* color_buf = k4a_image_get_buffer(color_image);
		color = cv::Mat(color_height, color_width, CV_8UC4, color_buf, cv::Mat::AUTO_STEP);

		/* Push a frame to queue, critical section */
		{
			std::lock_guard<std::mutex> lk(frameMtx);
			if (_frames.size() < 100) {
				frames.push_back(Frame());
				frames.back().idx = _idx;
				frames.back().depth.reset(new cv::Mat());
				depth.copyTo(*(frames.back().depth));
				frames.back().color.reset(new cv::Mat());
				//remap(color, *(frames.back().color), *_map1Color, *_map2Color, cv::INTER_LINEAR);
				color.copyTo(*(frames.back().color));
				frames.back().timestamp = k4a_image_get_timestamp_usec(depth_image);
			}
		}
		k4a_image_release(depth_image);
		k4a_image_release(color_image);
		k4a_capture_release(cpt);

		/* Notify main thread a frame is ready */
		frameCV.notify_one();
	}
}

bool K4aCameraImpl::FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
	uint16_t* depth, size_t inSzDepth, size_t* outSzDepth) {
	return FetchFrame(color, inSzColor, outSzColor, depth, inSzDepth, outSzDepth, nullptr);
}


bool K4aCameraImpl::FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
	uint16_t* depth, size_t inSzDepth, size_t* outSzDepth, size_t* idx) {

	/* Get first frame from queue */
	Frame frm;
	std::unique_lock<std::mutex> lk(_frameMtx);
	_frameCV.wait(lk, [this]() { return !_frames.empty(); }); // wait for notify and _frames not empty

	frm = std::move(_frames.front());

	/* Reduce queue size */
	_frames.pop_front();
	lk.unlock();

	/* Process depth image With undistortion */
	if (depth) {
		size_t rows = DepthHeight();
		size_t cols = DepthWidth();
		size_t sz_depth = 0;
		sz_depth = rows * cols * sizeof(uint16_t);
		cv::Mat corrected(rows, cols, CV_16UC1, depth);
		cv::remap(*(frm.depth), corrected, *_map1Depth, *_map2Depth, cv::INTER_LINEAR);
		//std::memcpy(depth, frm.depth->ptr<uint16_t>(), std::min(inSzDepth, sz_depth));
		if (outSzDepth) {
			*outSzDepth = sz_depth;
		}
	}

	/* Process color image With undistortion */
	if (color) {
		size_t rows = ColorHeight();
		size_t cols = ColorWidth();
		size_t sz_color = 0;
		sz_color = rows * cols * sizeof(uint8_t) * 4;
		cv::Mat corrected(rows, cols, CV_8UC4, color);
		cv::remap(*(frm.color), corrected, *_map1Color, *_map2Color, cv::INTER_LINEAR);
		//std::memcpy(color, frm.color->ptr<uint8_t>(), std::min(inSzColor, sz_color));
		if (outSzColor) {
			*outSzColor = sz_color;
		}
	}
	std::cout << "Timestamp of device " << _deviceIdx << " of frame idx " << frm.idx << "is" << frm.timestamp << std::endl;
	if (idx) {
		*idx = frm.idx;
	}
	return true;
}

bool K4aCameraImpl::FetchFrameInTime(uint8_t* color, size_t inSzColor, size_t* outSzColor,
	uint16_t* depth, size_t inSzDepth, size_t* outSzDepth) {
	k4a_capture_t cpt;
	k4a_device_get_capture(_hDev, &cpt, K4A_WAIT_INFINITE);

	/* Process color image */
	k4a_image_t color_image = k4a_capture_get_color_image(cpt);
	size_t sz_color = ColorHeight() * ColorWidth() * sizeof(uint8_t) * 4;
	uint8_t* color_buf = k4a_image_get_buffer(color_image);
	memcpy(color, color_buf, sz_color);

	/* Process depth image */
	k4a_image_t depth_image = k4a_capture_get_depth_image(cpt);
	size_t sz_depth = DepthHeight() * DepthWidth() * sizeof(uint16_t);
	uint8_t* depth_buf = k4a_image_get_buffer(depth_image);
	memcpy(depth, depth_buf, sz_depth);
	k4a_image_release(color_image);
	k4a_image_release(depth_image);
	k4a_capture_release(cpt);
	return true;
}