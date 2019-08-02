#pragma once

#include <cstdint>

#include <memory>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <mutex>
#include <deque>
#include <vector>

#include "K4aConfig.h"
#include "k4a/k4a.h"

using std::shared_ptr;

struct Intrinsic;
struct Extrinsic;


namespace cv {
	class Mat;
}

struct Frame
{
	shared_ptr<cv::Mat> depth;
	shared_ptr<cv::Mat> color;
	size_t idx;
	uint64_t timestamp;
};

class K4aCameraImpl {
public:
	static size_t DetectCameras();

private:
	static std::vector<k4a_device_t> _selected;

public:
	K4aCameraImpl();
	K4aCameraImpl(const size_t id, const bool master = false);
	~K4aCameraImpl();

	bool IsMaster();

	void StartCamera();
	void StartCamera(const K4aConfig& config);
	void StopCamera();

	bool IsOpened() const { return _isOpened; }
	bool IsDepthEnabled() const { return _depthEnabled; }
	bool IsColorEnabled() const { return _colorEnabled; }

	float GetScaleUnit() const { return _scaleUnit; }

	std::string GetSerialNumber() const { return _sn; }

	int DepthWidth() const;
	int DepthHeight() const;
	int ColorWidth() const;
	int ColorHeight() const;

	shared_ptr<Intrinsic> GetDepthIntrinsic() const;
	shared_ptr<Intrinsic> GetColorIntrinsic() const;
	shared_ptr<Extrinsic> GetColor2DepthExtrinsic() const;

	size_t GetSyncQueueSize();
	void PopFrontSyncQueue(const size_t sz);

	void SetWhiteBalance(const int32_t& degreesKelvin);
	void SetExposure(const uint32_t& microsecond);
	void SetGain(const int32_t& g);

	bool FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
		uint16_t* depth, size_t inSzDepth, size_t* outSzDepth);
	bool FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
		uint16_t* depth, size_t inSzDepth, size_t* outSzDepth, size_t* idx);
	bool FetchFrameInTime(uint8_t* color, size_t inSzColor, size_t* outSzColor,
		uint16_t* depth, size_t inSzDepth, size_t* outSzDepth);

private:
	void StopCapture();
	void CaptureThread(std::deque<Frame>& frames, std::atomic_bool& capture, size_t& idx, std::mutex& frameMtx, std::condition_variable& frameCV);

private:
	bool _isOpened; /* camera is opened */

	char _sn[32]; /* serial number */
	k4a_device_t _hDev; /* device handler*/
	float _scaleUnit; // TODO

	k4a_calibration_t _deviceCalib; /* device calibration */
	k4a_calibration_camera_t _depthCalib;/* depth cam calibration */
	k4a_calibration_camera_t _colorCalib; /* color cam calibration */

	bool _depthEnabled; /* depth cam enabled */
	bool _colorEnabled; /* color cam enabled */

	shared_ptr<cv::Mat> _imageBufColor;
	shared_ptr<cv::Mat> _map1Color;
	shared_ptr<cv::Mat> _map2Color;

	shared_ptr<cv::Mat> _imageBufDepth;
	shared_ptr<cv::Mat> _map1Depth;
	shared_ptr<cv::Mat> _map2Depth;

	size_t _idx; /* index of frame */
	bool _master; /* who is master */
	size_t _deviceIdx; /* index in device list */
	std::atomic_bool _capture; /* capturing flag */
	std::thread		_captureTrd; /* capturing thread */

	std::deque<Frame> _frames;  /* frame queue */
	std::mutex _frameMtx; /* protect frame queue */
	std::condition_variable _frameCV; /* notify frame ready */

	float _rGain, _gGain, _bGain; /* channels gain */
	float _sRGain, _sGGain, _sBGain; /* suggessted channels gain */


};