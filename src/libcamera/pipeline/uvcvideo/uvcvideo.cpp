/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Pipeline handler for uvcvideo devices
 */

#include <algorithm>
#include <bitset>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/mutex.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/sysfs.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(UVC)

class UVCCameraData : public Camera::Private
{
public:
	UVCCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	int init(MediaDevice *media);
	void addControl(uint32_t cid, const ControlInfo &v4l2info,
			ControlInfoMap::Map *ctrls);
	void imageBufferReady(FrameBuffer *buffer);

	const std::string &id() const { return id_; }

	Mutex openLock_;
	std::unique_ptr<V4L2VideoDevice> video_;
	Stream stream_;
	std::map<PixelFormat, std::vector<SizeRange>> formats_;

	std::optional<v4l2_exposure_auto_type> autoExposureMode_;
	std::optional<v4l2_exposure_auto_type> manualExposureMode_;

private:
	bool generateId();

	std::string id_;
};

class UVCCameraConfiguration : public CameraConfiguration
{
public:
	UVCCameraConfiguration(UVCCameraData *data);

	Status validate() override;

private:
	UVCCameraData *data_;
};

class PipelineHandlerUVC : public PipelineHandler
{
public:
	PipelineHandlerUVC(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	int processControl(const UVCCameraData *data, ControlList *controls,
			   unsigned int id, const ControlValue &value);
	int processControls(UVCCameraData *data, const ControlList &reqControls);

	bool acquireDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	UVCCameraData *cameraData(Camera *camera)
	{
		return static_cast<UVCCameraData *>(camera->_d());
	}
};

namespace {

std::optional<controls::ExposureTimeModeEnum> v4l2ToExposureMode(int32_t x)
{
	using namespace controls;

	switch (x) {
	case V4L2_EXPOSURE_AUTO:
	case V4L2_EXPOSURE_APERTURE_PRIORITY:
		return ExposureTimeModeAuto;
	case V4L2_EXPOSURE_MANUAL:
	case V4L2_EXPOSURE_SHUTTER_PRIORITY:
		return ExposureTimeModeManual;
	default:
		return {};
	}
}

} /* namespace */

UVCCameraConfiguration::UVCCameraConfiguration(UVCCameraData *data)
	: CameraConfiguration(), data_(data)
{
}

CameraConfiguration::Status UVCCameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	if (orientation != Orientation::Rotate0) {
		orientation = Orientation::Rotate0;
		status = Adjusted;
	}

	/* Cap the number of entries to the available streams. */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];
	const StreamFormats &formats = cfg.formats();
	const PixelFormat pixelFormat = cfg.pixelFormat;
	const Size size = cfg.size;

	const std::vector<PixelFormat> pixelFormats = formats.pixelformats();
	auto iter = std::find(pixelFormats.begin(), pixelFormats.end(), pixelFormat);
	if (iter == pixelFormats.end()) {
		cfg.pixelFormat = pixelFormats.front();
		LOG(UVC, Debug)
			<< "Adjusting pixel format from " << pixelFormat
			<< " to " << cfg.pixelFormat;
		status = Adjusted;
	}

	const std::vector<Size> &formatSizes = formats.sizes(cfg.pixelFormat);
	cfg.size = formatSizes.front();
	for (const Size &formatsSize : formatSizes) {
		if (formatsSize > size)
			break;

		cfg.size = formatsSize;
	}

	if (cfg.size != size) {
		LOG(UVC, Debug)
			<< "Adjusting size from " << size << " to " << cfg.size;
		status = Adjusted;
	}

	cfg.bufferCount = 4;

	V4L2DeviceFormat format;
	format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	/*
	 * For power-consumption reasons video_ is closed when the camera is not
	 * acquired. Open it here if necessary.
	 */
	{
		bool opened = false;

		MutexLocker locker(data_->openLock_);

		if (!data_->video_->isOpen()) {
			int ret = data_->video_->open();
			if (ret)
				return Invalid;

			opened = true;
		}

		int ret = data_->video_->tryFormat(&format);
		if (opened)
			data_->video_->close();
		if (ret)
			return Invalid;
	}

	cfg.stride = format.planes[0].bpl;
	cfg.frameSize = format.planes[0].size;

	if (cfg.colorSpace != format.colorSpace) {
		cfg.colorSpace = format.colorSpace;
		status = Adjusted;
	}

	return status;
}

PipelineHandlerUVC::PipelineHandlerUVC(CameraManager *manager)
	: PipelineHandler(manager)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerUVC::generateConfiguration(Camera *camera,
					  Span<const StreamRole> roles)
{
	UVCCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<UVCCameraConfiguration>(data);

	if (roles.empty())
		return config;

	StreamFormats formats(data->formats_);
	StreamConfiguration cfg(formats);

	cfg.pixelFormat = formats.pixelformats().front();
	cfg.size = formats.sizes(cfg.pixelFormat).back();
	cfg.bufferCount = 4;

	config->addConfiguration(cfg);

	config->validate();

	return config;
}

int PipelineHandlerUVC::configure(Camera *camera, CameraConfiguration *config)
{
	UVCCameraData *data = cameraData(camera);
	StreamConfiguration &cfg = config->at(0);
	int ret;

	V4L2DeviceFormat format;
	format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
	format.size = cfg.size;

	ret = data->video_->setFormat(&format);
	if (ret)
		return ret;

	if (format.size != cfg.size ||
	    format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat))
		return -EINVAL;

	cfg.setStream(&data->stream_);

	return 0;
}

int PipelineHandlerUVC::exportFrameBuffers(Camera *camera, Stream *stream,
					   std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	return data->video_->exportBuffers(count, buffers);
}

int PipelineHandlerUVC::start(Camera *camera, const ControlList *controls)
{
	UVCCameraData *data = cameraData(camera);
	unsigned int count = data->stream_.configuration().bufferCount;

	int ret = data->video_->importBuffers(count);
	if (ret < 0)
		return ret;

	if (controls) {
		ret = processControls(data, *controls);
		if (ret < 0)
			goto err_release_buffers;
	}

	ret = data->video_->streamOn();
	if (ret < 0)
		goto err_release_buffers;

	return 0;

err_release_buffers:
	data->video_->releaseBuffers();

	return ret;
}

void PipelineHandlerUVC::stopDevice(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);
	data->video_->streamOff();
	data->video_->releaseBuffers();
}

int PipelineHandlerUVC::processControl(const UVCCameraData *data, ControlList *controls,
				       unsigned int id, const ControlValue &value)
{
	uint32_t cid;

	if (id == controls::Brightness)
		cid = V4L2_CID_BRIGHTNESS;
	else if (id == controls::Contrast)
		cid = V4L2_CID_CONTRAST;
	else if (id == controls::Saturation)
		cid = V4L2_CID_SATURATION;
	else if (id == controls::ExposureTimeMode)
		cid = V4L2_CID_EXPOSURE_AUTO;
	else if (id == controls::ExposureTime)
		cid = V4L2_CID_EXPOSURE_ABSOLUTE;
	else if (id == controls::AnalogueGain)
		cid = V4L2_CID_GAIN;
	else if (id == controls::Gamma)
		cid = V4L2_CID_GAMMA;
	else if (id == controls::AeEnable)
		return 0; /* Handled in `Camera::queueRequest()`. */
	else
		return -EINVAL;

	const ControlInfo &v4l2Info = controls->infoMap()->at(cid);
	int32_t min = v4l2Info.min().get<int32_t>();
	int32_t def = v4l2Info.def().get<int32_t>();
	int32_t max = v4l2Info.max().get<int32_t>();

	/*
	 * See UVCCameraData::addControl() for explanations of the different
	 * value mappings.
	 */
	switch (cid) {
	case V4L2_CID_BRIGHTNESS: {
		float scale = std::max(max - def, def - min);
		float fvalue = value.get<float>() * scale + def;
		controls->set(cid, static_cast<int32_t>(std::lround(fvalue)));
		break;
	}

	case V4L2_CID_SATURATION: {
		float scale = def - min;
		float fvalue = value.get<float>() * scale + min;
		controls->set(cid, static_cast<int32_t>(std::lround(fvalue)));
		break;
	}

	case V4L2_CID_EXPOSURE_AUTO: {
		std::optional<v4l2_exposure_auto_type> mode;

		switch (value.get<int32_t>()) {
		case controls::ExposureTimeModeAuto:
			mode = data->autoExposureMode_;
			break;
		case controls::ExposureTimeModeManual:
			mode = data->manualExposureMode_;
			break;
		}

		if (!mode)
			return -EINVAL;

		controls->set(V4L2_CID_EXPOSURE_AUTO, static_cast<int32_t>(*mode));
		break;
	}

	case V4L2_CID_EXPOSURE_ABSOLUTE:
		controls->set(cid, value.get<int32_t>() / 100);
		break;

	case V4L2_CID_CONTRAST:
	case V4L2_CID_GAIN: {
		float m = (4.0f - 1.0f) / (max - def);
		float p = 1.0f - m * def;

		if (m * min + p < 0.5f) {
			m = (1.0f - 0.5f) / (def - min);
			p = 1.0f - m * def;
		}

		float fvalue = (value.get<float>() - p) / m;
		controls->set(cid, static_cast<int32_t>(std::lround(fvalue)));
		break;
	}

	case V4L2_CID_GAMMA:
		controls->set(cid, static_cast<int32_t>(std::lround(value.get<float>() * 100)));
		break;

	default: {
		int32_t ivalue = value.get<int32_t>();
		controls->set(cid, ivalue);
		break;
	}
	}

	return 0;
}

int PipelineHandlerUVC::processControls(UVCCameraData *data, const ControlList &reqControls)
{
	ControlList controls(data->video_->controls());

	for (const auto &[id, value] : reqControls)
		processControl(data, &controls, id, value);

	for (const auto &ctrl : controls)
		LOG(UVC, Debug)
			<< "Setting control " << utils::hex(ctrl.first)
			<< " to " << ctrl.second.toString();

	int ret = data->video_->setControls(&controls);
	if (ret) {
		LOG(UVC, Error) << "Failed to set controls: " << ret;
		return ret < 0 ? ret : -EINVAL;
	}

	return ret;
}

int PipelineHandlerUVC::queueRequestDevice(Camera *camera, Request *request)
{
	UVCCameraData *data = cameraData(camera);
	FrameBuffer *buffer = request->findBuffer(&data->stream_);
	if (!buffer) {
		LOG(UVC, Error)
			<< "Attempt to queue request with invalid stream";

		return -ENOENT;
	}

	int ret = processControls(data, request->controls());
	if (ret < 0)
		return ret;

	ret = data->video_->queueBuffer(buffer);
	if (ret < 0)
		return ret;

	return 0;
}

bool PipelineHandlerUVC::match(DeviceEnumerator *enumerator)
{
	MediaDevice *media;
	DeviceMatch dm("uvcvideo");

	media = acquireMediaDevice(enumerator, dm);
	if (!media)
		return false;

	std::unique_ptr<UVCCameraData> data = std::make_unique<UVCCameraData>(this);

	if (data->init(media))
		return false;

	/* Create and register the camera. */
	std::string id = data->id();
	std::set<Stream *> streams{ &data->stream_ };
	std::shared_ptr<Camera> camera =
		Camera::create(std::move(data), id, streams);
	registerCamera(std::move(camera));

	/* Enable hot-unplug notifications. */
	hotplugMediaDevice(media);

	return true;
}

bool PipelineHandlerUVC::acquireDevice(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);

	MutexLocker locker(data->openLock_);

	return data->video_->open() == 0;
}

void PipelineHandlerUVC::releaseDevice(Camera *camera)
{
	UVCCameraData *data = cameraData(camera);

	MutexLocker locker(data->openLock_);
	data->video_->close();
}

int UVCCameraData::init(MediaDevice *media)
{
	int ret;

	/* Locate and initialise the camera data with the default video node. */
	const std::vector<MediaEntity *> &entities = media->entities();
	auto entity = std::find_if(entities.begin(), entities.end(),
				   [](MediaEntity *e) {
					   return e->flags() & MEDIA_ENT_FL_DEFAULT;
				   });
	if (entity == entities.end()) {
		LOG(UVC, Error) << "Could not find a default video device";
		return -ENODEV;
	}

	/* Create and open the video device. */
	video_ = std::make_unique<V4L2VideoDevice>(*entity);
	ret = video_->open();
	if (ret)
		return ret;

	video_->bufferReady.connect(this, &UVCCameraData::imageBufferReady);

	/* Generate the camera ID. */
	if (!generateId()) {
		LOG(UVC, Error) << "Failed to generate camera ID";
		return -EINVAL;
	}

	/*
	 * Populate the map of supported formats, and infer the camera sensor
	 * resolution from the largest size it advertises.
	 */
	Size resolution;
	for (const auto &format : video_->formats()) {
		PixelFormat pixelFormat = format.first.toPixelFormat();
		if (!pixelFormat.isValid())
			continue;

		formats_[pixelFormat] = format.second;

		const std::vector<SizeRange> &sizeRanges = format.second;
		for (const SizeRange &sizeRange : sizeRanges) {
			if (sizeRange.max > resolution)
				resolution = sizeRange.max;
		}
	}

	if (formats_.empty()) {
		LOG(UVC, Error)
			<< "Camera " << id_ << " (" << media->model()
			<< ") doesn't expose any supported format";
		return -EINVAL;
	}

	/* Populate the camera properties. */
	properties_.set(properties::Model, utils::toAscii(media->model()));

	/*
	 * Derive the location from the device removable attribute in sysfs.
	 * Non-removable devices are assumed to be front as we lack detailed
	 * location information, and removable device are considered external.
	 *
	 * The sysfs removable attribute is derived from the ACPI _UPC attribute
	 * if available, or from the USB hub descriptors otherwise. ACPI data
	 * may not be very reliable, and the USB hub descriptors may not be
	 * accurate on DT-based platforms. A heuristic may need to be
	 * implemented later if too many devices end up being miscategorized.
	 *
	 * \todo Find a way to tell front and back devices apart. This could
	 * come from the ACPI _PLD, but that may be even more unreliable than
	 * the _UPC.
	 */
	properties::LocationEnum location = properties::CameraLocationExternal;
	std::ifstream file(video_->devicePath() + "/../removable");
	if (file.is_open()) {
		std::string value;
		std::getline(file, value);
		file.close();

		if (value == "fixed")
			location = properties::CameraLocationFront;
	}

	properties_.set(properties::Location, location);

	properties_.set(properties::PixelArraySize, resolution);
	properties_.set(properties::PixelArrayActiveAreas, { Rectangle(resolution) });

	/* Initialise the supported controls. */
	ControlInfoMap::Map ctrls;

	for (const auto &ctrl : video_->controls()) {
		uint32_t cid = ctrl.first->id();
		const ControlInfo &info = ctrl.second;

		addControl(cid, info, &ctrls);
	}

	if (autoExposureMode_ && manualExposureMode_) {
		/* \todo Move this to the Camera class */
		ctrls[&controls::AeEnable] = ControlInfo(false, true, true);
	}

	controlInfo_ = ControlInfoMap(std::move(ctrls), controls::controls);

	/*
	 * Close to allow camera to go into runtime-suspend, video_ will be
	 * re-opened from acquireDevice() and validate().
	 */
	video_->close();

	return 0;
}

bool UVCCameraData::generateId()
{
	const std::string path = video_->devicePath();

	/* Create a controller ID from first device described in firmware. */
	std::string controllerId;
	std::string searchPath = path;
	while (true) {
		std::string::size_type pos = searchPath.rfind('/');
		if (pos <= 1) {
			LOG(UVC, Error) << "Can not find controller ID";
			return false;
		}

		searchPath = searchPath.substr(0, pos);

		controllerId = sysfs::firmwareNodePath(searchPath);
		if (!controllerId.empty())
			break;
	}

	/*
	 * Create a USB ID from the device path which has the known format:
	 *
	 *	path = bus, "-", ports, ":", config, ".", interface ;
	 *	bus = number ;
	 *	ports = port, [ ".", ports ] ;
	 *	port = number ;
	 *	config = number ;
	 *	interface = number ;
	 *
	 * Example: 3-2.4:1.0
	 *
	 * The bus is not guaranteed to be stable and needs to be stripped from
	 * the USB ID. The final USB ID is built up of the ports, config and
	 * interface properties.
	 *
	 * Example 2.4:1.0.
	 */
	std::string usbId = utils::basename(path.c_str());
	usbId = usbId.substr(usbId.find('-') + 1);

	/* Creata a device ID from the USB devices vendor and product ID. */
	std::string deviceId;
	for (const char *name : { "idVendor", "idProduct" }) {
		std::ifstream file(path + "/../" + name);

		if (!file.is_open())
			return false;

		std::string value;
		std::getline(file, value);
		file.close();

		if (!deviceId.empty())
			deviceId += ":";

		deviceId += value;
	}

	id_ = controllerId + "-" + usbId + "-" + deviceId;
	return true;
}

void UVCCameraData::addControl(uint32_t cid, const ControlInfo &v4l2Info,
			       ControlInfoMap::Map *ctrls)
{
	const ControlId *id;
	ControlInfo info;

	/* Map the control ID. */
	switch (cid) {
	case V4L2_CID_BRIGHTNESS:
		id = &controls::Brightness;
		break;
	case V4L2_CID_CONTRAST:
		id = &controls::Contrast;
		break;
	case V4L2_CID_SATURATION:
		id = &controls::Saturation;
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		id = &controls::ExposureTimeMode;
		break;
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		id = &controls::ExposureTime;
		break;
	case V4L2_CID_GAIN:
		id = &controls::AnalogueGain;
		break;
	case V4L2_CID_GAMMA:
		id = &controls::Gamma;
		break;
	default:
		return;
	}

	/* Map the control info. */
	const std::vector<ControlValue> &v4l2Values = v4l2Info.values();
	int32_t min = v4l2Info.min().get<int32_t>();
	int32_t max = v4l2Info.max().get<int32_t>();
	int32_t def = v4l2Info.def().get<int32_t>();

	switch (cid) {
	case V4L2_CID_BRIGHTNESS: {
		/*
		 * The Brightness control is a float, with 0.0 mapped to the
		 * default value. The control range is [-1.0, 1.0], but the V4L2
		 * default may not be in the middle of the V4L2 range.
		 * Accommodate this by restricting the range of the libcamera
		 * control, but always within the maximum limits.
		 */
		float scale = std::max(max - def, def - min);

		info = ControlInfo{
			{ static_cast<float>(min - def) / scale },
			{ static_cast<float>(max - def) / scale },
			{ 0.0f }
		};
		break;
	}

	case V4L2_CID_SATURATION:
		/*
		 * The Saturation control is a float, with 0.0 mapped to the
		 * minimum value (corresponding to a fully desaturated image)
		 * and 1.0 mapped to the default value. Calculate the maximum
		 * value accordingly.
		 */
		info = ControlInfo{
			{ 0.0f },
			{ static_cast<float>(max - min) / (def - min) },
			{ 1.0f }
		};
		break;

	case V4L2_CID_EXPOSURE_AUTO: {
		/*
		 * From the V4L2_CID_EXPOSURE_AUTO documentation:
		 *
		 * ------------------------------------------------------------
		 * V4L2_EXPOSURE_AUTO:
		 * Automatic exposure time, automatic iris aperture.
		 *
		 * V4L2_EXPOSURE_MANUAL:
		 * Manual exposure time, manual iris.
		 *
		 * V4L2_EXPOSURE_SHUTTER_PRIORITY:
		 * Manual exposure time, auto iris.
		 *
		 * V4L2_EXPOSURE_APERTURE_PRIORITY:
		 * Auto exposure time, manual iris.
		 *-------------------------------------------------------------
		 *
		 * ExposureTimeModeAuto = { V4L2_EXPOSURE_AUTO,
		 * 			    V4L2_EXPOSURE_APERTURE_PRIORITY }
		 *
		 *
		 * ExposureTimeModeManual = { V4L2_EXPOSURE_MANUAL,
		 *			      V4L2_EXPOSURE_SHUTTER_PRIORITY }
		 */

		std::bitset<
			std::max(V4L2_EXPOSURE_AUTO,
			std::max(V4L2_EXPOSURE_APERTURE_PRIORITY,
			std::max(V4L2_EXPOSURE_MANUAL,
				 V4L2_EXPOSURE_SHUTTER_PRIORITY))) + 1
		> exposureModes;
		std::optional<controls::ExposureTimeModeEnum> lcDef;

		for (const ControlValue &value : v4l2Values) {
			const auto x = value.get<int32_t>();

			if (0 <= x && static_cast<std::size_t>(x) < exposureModes.size()) {
				exposureModes[x] = true;

				if (x == def)
					lcDef = v4l2ToExposureMode(x);
			}
		}

		if (exposureModes[V4L2_EXPOSURE_AUTO])
			autoExposureMode_ = V4L2_EXPOSURE_AUTO;
		else if (exposureModes[V4L2_EXPOSURE_APERTURE_PRIORITY])
			autoExposureMode_ = V4L2_EXPOSURE_APERTURE_PRIORITY;

		if (exposureModes[V4L2_EXPOSURE_SHUTTER_PRIORITY])
			manualExposureMode_ = V4L2_EXPOSURE_SHUTTER_PRIORITY;
		else if (exposureModes[V4L2_EXPOSURE_MANUAL])
			manualExposureMode_ = V4L2_EXPOSURE_MANUAL;

		std::array<ControlValue, 2> values;
		std::size_t count = 0;

		if (autoExposureMode_)
			values[count++] = controls::ExposureTimeModeAuto;

		if (manualExposureMode_)
			values[count++] = controls::ExposureTimeModeManual;

		if (count == 0)
			return;

		info = ControlInfo{
			Span<const ControlValue>{ values.data(), count },
			!lcDef ? values.front() : *lcDef,
		};
		break;
	}
	case V4L2_CID_EXPOSURE_ABSOLUTE:
		/*
		 * ExposureTime is in units of 1 µs, and UVC expects
		 * V4L2_CID_EXPOSURE_ABSOLUTE in units of 100 µs.
		 */
		info = ControlInfo{
			{ min * 100 },
			{ max * 100 },
			{ def * 100 }
		};
		break;

	case V4L2_CID_CONTRAST:
	case V4L2_CID_GAIN: {
		/*
		 * The Contrast and AnalogueGain controls are floats, with 1.0
		 * mapped to the default value. UVC doesn't specify units, and
		 * cameras have been seen to expose very different ranges for
		 * the controls. Arbitrarily assume that the minimum and
		 * maximum values are respectively no lower than 0.5 and no
		 * higher than 4.0.
		 */
		float m = (4.0f - 1.0f) / (max - def);
		float p = 1.0f - m * def;

		if (m * min + p < 0.5f) {
			m = (1.0f - 0.5f) / (def - min);
			p = 1.0f - m * def;
		}

		info = ControlInfo{
			{ m * min + p },
			{ m * max + p },
			{ 1.0f }
		};
		break;
	}

	case V4L2_CID_GAMMA:
		/* UVC gamma is in units of 1/100 gamma. */
		info = ControlInfo{
			{ min / 100.0f },
			{ max / 100.0f },
			{ def / 100.0f }
		};
		break;

	default:
		info = v4l2Info;
		break;
	}

	ctrls->emplace(id, info);
}

void UVCCameraData::imageBufferReady(FrameBuffer *buffer)
{
	Request *request = buffer->request();

	/* \todo Use the UVC metadata to calculate a more precise timestamp */
	request->metadata().set(controls::SensorTimestamp,
				buffer->metadata().timestamp);

	pipe()->completeBuffer(request, buffer);
	pipe()->completeRequest(request);
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerUVC, "uvcvideo")

} /* namespace libcamera */
