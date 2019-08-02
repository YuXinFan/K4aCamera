#include "K4aCamera.h"
#include "K4aCameraImpl.h"

using namespace std;

size_t K4aCamera::DetectCameras() { return K4aCameraImpl::DetectCameras(); }

K4aCamera::K4aCamera()
	: _pImpl(new K4aCameraImpl())
{
}

K4aCamera::K4aCamera(const size_t i, const bool master)
	: _pImpl(new K4aCameraImpl(i, master))
{}

K4aCamera::~K4aCamera() { _pImpl = nullptr; }

bool K4aCamera::IsMaster() { return _pImpl->IsMaster(); }
void K4aCamera::StartCamera() { _pImpl->StartCamera(); }

void K4aCamera::StartCamera(const K4aConfig& config) { _pImpl->StartCamera(config); }

void K4aCamera::StopCamera() { _pImpl->StopCamera(); }

bool K4aCamera::IsOpened() const { return _pImpl->IsOpened(); }

bool K4aCamera::IsDepthEnabled() const { return _pImpl->IsDepthEnabled(); }

bool K4aCamera::IsColorEnabled() const { return _pImpl->IsColorEnabled(); }

float K4aCamera::GetScaleUnit() const { return _pImpl->GetScaleUnit(); }

std::string K4aCamera::GetSerialNumber() const { return _pImpl->GetSerialNumber(); }

int K4aCamera::DepthWidth() const { return _pImpl->DepthWidth(); }

int K4aCamera::DepthHeight() const { return _pImpl->DepthHeight(); }

int K4aCamera::ColorWidth() const { return _pImpl->ColorWidth(); }

int K4aCamera::ColorHeight() const { return _pImpl->ColorHeight(); }

shared_ptr<Intrinsic> K4aCamera::GetDepthIntrinsic() const { return _pImpl->GetDepthIntrinsic(); }

shared_ptr<Intrinsic> K4aCamera::GetColorIntrinsic() const { return _pImpl->GetColorIntrinsic(); }

shared_ptr<Extrinsic> K4aCamera::GetColor2DepthExtrinsic() const { return _pImpl->GetColor2DepthExtrinsic(); }

bool K4aCamera::FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
	uint16_t* depth, size_t inSzDepth, size_t* outSzDepth)
{
	return _pImpl->FetchFrame(color, inSzColor, outSzColor, depth, inSzDepth, outSzDepth);
}

bool K4aCamera::FetchFrame(uint8_t* color, size_t inSzColor, size_t* outSzColor,
	uint16_t* depth, size_t inSzDepth, size_t* outSzDepth, size_t* idx)
{
	return _pImpl->FetchFrame(color, inSzColor, outSzColor, depth, inSzDepth, outSzDepth, idx);
}

size_t K4aCamera::GetSyncQueueSize()
{
	return _pImpl->GetSyncQueueSize();
}

void K4aCamera::PopFrontSyncQueue(const size_t sz)
{
	_pImpl->PopFrontSyncQueue(sz);
}

void K4aCamera::SetWhiteBalance(const int32_t& degreesKelvin)
{
	_pImpl->SetWhiteBalance(degreesKelvin);
}

void K4aCamera::SetGain(const int32_t& g)
{
	_pImpl->SetGain(g);
}

void K4aCamera::SetExposure(const uint32_t& microsecond)
{
	_pImpl->SetExposure(microsecond);
}

