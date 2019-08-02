#pragma once
#include <memory>
#include <string>

using std::shared_ptr;

class K4aCameraImpl;

struct Intrinsic;
struct Extrinsic;
struct K4aConfig;

#define EXPORT __declspec(dllexport)

class K4aCamera
{
public:
	EXPORT static size_t DetectCameras();

public:
	EXPORT K4aCamera();

	/* The id of a camera ranges from 0 to DetectedCameras() - 1. A master
	 * camera is configured by master configuration. Slave cameras are
	 * configured to receive signals from master camera by sub configuration.
	 */
	EXPORT K4aCamera(const size_t id, const bool master = false);

	EXPORT ~K4aCamera();

	EXPORT bool IsMaster();
	EXPORT void StartCamera();
	EXPORT void StartCamera(const K4aConfig& config);
	EXPORT void StopCamera();

	EXPORT bool IsOpened() const;
	EXPORT bool IsDepthEnabled() const;
	EXPORT bool IsColorEnabled() const;

	EXPORT float GetScaleUnit() const;

	EXPORT std::string GetSerialNumber() const;

	EXPORT int DepthWidth() const;
	EXPORT int DepthHeight() const;
	EXPORT int ColorWidth() const;
	EXPORT int ColorHeight() const;

	EXPORT shared_ptr<Intrinsic> GetDepthIntrinsic() const;
	EXPORT shared_ptr<Intrinsic> GetColorIntrinsic() const;
	EXPORT shared_ptr<Extrinsic> GetColor2DepthExtrinsic() const;

	/* Depth image pixel format is uint16_t. It's size should be W*H*2 bytes.
	 * Color image pixel format is BGRA. It's size should be W*H*4 bytes.
	 */
	EXPORT bool FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
		uint16_t* depth, size_t inSzDepth, size_t* outSzDepth);
	EXPORT bool FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
		uint16_t* depth, size_t inSzDepth, size_t* outSzDepth, size_t* idx);

	/* Synchronization queue operation */
	EXPORT size_t GetSyncQueueSize();
	EXPORT void PopFrontSyncQueue(const size_t sz);

	/* White balance */
	EXPORT void SetWhiteBalance(const int32_t& degreesKelvin);

	/* Exposure times of 10, about 1000~ to 5000~ */
	EXPORT void SetExposure(const uint32_t& microsecond);

	/* Gain*/
	EXPORT void SetGain(const int32_t& g);
private:
	shared_ptr<K4aCameraImpl> _pImpl;
};