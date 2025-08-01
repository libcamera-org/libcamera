/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Camera capture session
 */

#include "camera_session.h"

#include <iomanip>
#include <iostream>
#include <limits.h>
#include <optional>
#include <sstream>

#include <libcamera/control_ids.h>
#include <libcamera/property_ids.h>

#include "../common/event_loop.h"
#include "../common/stream_options.h"

#include "capture_script.h"
#include "file_sink.h"
#ifdef HAVE_KMS
#include "kms_sink.h"
#endif
#include "main.h"
#ifdef HAVE_SDL
#include "sdl_sink.h"
#endif

using namespace libcamera;

CameraSession::CameraSession(CameraManager *cm,
			     const std::string &cameraId,
			     unsigned int cameraIndex,
			     const OptionsParser::Options &options)
	: options_(options), cameraIndex_(cameraIndex), last_(0),
	  queueCount_(0), captureCount_(0), captureLimit_(0),
	  printMetadata_(false)
{
	char *endptr;
	unsigned long index = strtoul(cameraId.c_str(), &endptr, 10);

	if (*endptr == '\0' && index > 0) {
		auto cameras = cm->cameras();
		if (index <= cameras.size())
			camera_ = cameras[index - 1];
	}

	if (!camera_)
		camera_ = cm->get(cameraId);

	if (!camera_) {
		std::cerr << "Camera " << cameraId << " not found" << std::endl;
		return;
	}

	if (camera_->acquire()) {
		std::cerr << "Failed to acquire camera " << cameraId
			  << std::endl;
		return;
	}

	std::vector<StreamRole> roles =
		StreamKeyValueParser::roles(options_[OptStream]);
	std::vector<std::vector<StreamRole>> tryRoles;
	if (!roles.empty()) {
		/*
		 * If the roles are explicitly specified then there's no need
		 * to try other roles
		 */
		tryRoles.push_back(roles);
	} else {
		tryRoles.push_back({ StreamRole::Viewfinder });
		tryRoles.push_back({ StreamRole::Raw });
	}

	std::unique_ptr<CameraConfiguration> config;
	bool valid = false;
	for (std::vector<StreamRole> &rolesIt : tryRoles) {
		config = camera_->generateConfiguration(rolesIt);
		if (config && config->size() == rolesIt.size()) {
			roles = rolesIt;
			valid = true;
			break;
		}
	}

	if (!valid) {
		std::cerr << "Failed to get default stream configuration"
			  << std::endl;
		return;
	}

	if (options_.isSet(OptOrientation)) {
		std::string orientOpt = options_[OptOrientation].toString();
		static const std::map<std::string, libcamera::Orientation> orientations{
			{ "rot0", libcamera::Orientation::Rotate0 },
			{ "rot180", libcamera::Orientation::Rotate180 },
			{ "mirror", libcamera::Orientation::Rotate0Mirror },
			{ "flip", libcamera::Orientation::Rotate180Mirror },
		};

		auto orientation = orientations.find(orientOpt);
		if (orientation == orientations.end()) {
			std::cerr << "Invalid orientation " << orientOpt << std::endl;
			return;
		}

		config->orientation = orientation->second;
	}

	/* Apply configuration if explicitly requested. */
	if (StreamKeyValueParser::updateConfiguration(config.get(),
						      options_[OptStream])) {
		std::cerr << "Failed to update configuration" << std::endl;
		return;
	}

	bool strictFormats = options_.isSet(OptStrictFormats);

#ifdef HAVE_KMS
	if (options_.isSet(OptDisplay)) {
		if (options_.isSet(OptFile)) {
			std::cerr << "--display and --file options are mutually exclusive"
				  << std::endl;
			return;
		}

		if (roles.size() != 1) {
			std::cerr << "Display doesn't support multiple streams"
				  << std::endl;
			return;
		}

		if (roles[0] != StreamRole::Viewfinder) {
			std::cerr << "Display requires a viewfinder stream"
				  << std::endl;
			return;
		}
	}
#endif

	if (options_.isSet(OptCaptureScript)) {
		std::string scriptName = options_[OptCaptureScript].toString();
		script_ = std::make_unique<CaptureScript>(camera_, scriptName);
		if (!script_->valid()) {
			std::cerr << "Invalid capture script '" << scriptName
				  << "'" << std::endl;
			return;
		}
	}

	switch (config->validate()) {
	case CameraConfiguration::Valid:
		break;

	case CameraConfiguration::Adjusted:
		if (strictFormats) {
			std::cout << "Adjusting camera configuration disallowed by --strict-formats argument"
				  << std::endl;
			return;
		}
		std::cout << "Camera configuration adjusted" << std::endl;
		break;

	case CameraConfiguration::Invalid:
		std::cout << "Camera configuration invalid" << std::endl;
		return;
	}

	config_ = std::move(config);
}

CameraSession::~CameraSession()
{
	if (camera_)
		camera_->release();
}

void CameraSession::listControls() const
{
	for (const auto &[id, info] : camera_->controls()) {
		std::stringstream io;
		io << "["
		   << (id->isInput() ? "in" : "  ")
		   << (id->isOutput() ? "out" : "   ")
		   << "] ";

		if (info.values().empty()) {
			std::cout << "Control: " << io.str()
				  << id->vendor() << "::" << id->name() << ": "
				  << info.toString() << std::endl;
		} else {
			std::cout << "Control: " << io.str()
				  << id->vendor() << "::" << id->name() << ":"
				  << std::endl;

			std::optional<int32_t> def;
			if (!info.def().isNone())
				def = info.def().get<int32_t>();

			for (const auto &value : info.values()) {
				int32_t val = value.get<int32_t>();
				const auto &it = id->enumerators().find(val);

				std::cout << "  - ";
				if (it == id->enumerators().end())
					std::cout << "UNKNOWN";
				else
					std::cout << it->second;

				std::cout << " (" << val << ")"
					  << (val == def ? " [default]" : "")
					  << std::endl;
			}
		}

		if (id->isArray()) {
			std::size_t size = id->size();

			std::cout << "   Size: ";
			if (size == std::numeric_limits<std::size_t>::max())
				std::cout << "n";
			else
				std::cout << std::to_string(size);
			std::cout << std::endl;
		}
	}
}

void CameraSession::listProperties() const
{
	for (const auto &[key, value] : camera_->properties()) {
		const ControlId *id = properties::properties.at(key);

		std::cout << "Property: " << id->name() << " = "
			  << value.toString();

		if (!id->enumerators().empty()) {
			int32_t val = value.get<int32_t>();
			const auto &iter = id->enumerators().find(val);

			if (iter != id->enumerators().end())
				std::cout << " (" << iter->second << ")";
		}

		std::cout << std::endl;
	}
}

void CameraSession::infoConfiguration() const
{
	unsigned int index = 0;
	for (const StreamConfiguration &cfg : *config_) {
		std::cout << index << ": " << cfg.toString() << std::endl;

		const StreamFormats &formats = cfg.formats();
		for (PixelFormat pixelformat : formats.pixelformats()) {
			std::cout << " * Pixelformat: "
				  << pixelformat << " "
				  << formats.range(pixelformat).toString()
				  << std::endl;

			for (const Size &size : formats.sizes(pixelformat))
				std::cout << "  - " << size << std::endl;
		}

		index++;
	}
}

int CameraSession::start()
{
	int ret;

	queueCount_ = 0;
	captureCount_ = 0;
	captureLimit_ = options_[OptCapture].toInteger();
	printMetadata_ = options_.isSet(OptMetadata);

	ret = camera_->configure(config_.get());
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	streamNames_.clear();
	for (unsigned int index = 0; index < config_->size(); ++index) {
		StreamConfiguration &cfg = config_->at(index);
		streamNames_[cfg.stream()] = "cam" + std::to_string(cameraIndex_)
					   + "-stream" + std::to_string(index);
	}

	camera_->requestCompleted.connect(this, &CameraSession::requestComplete);

#ifdef HAVE_KMS
	if (options_.isSet(OptDisplay))
		sink_ = std::make_unique<KMSSink>(options_[OptDisplay].toString());
#endif

#ifdef HAVE_SDL
	if (options_.isSet(OptSDL))
		sink_ = std::make_unique<SDLSink>();
#endif

	if (options_.isSet(OptFile)) {
		std::unique_ptr<FileSink> sink =
			std::make_unique<FileSink>(camera_.get(), streamNames_);

		if (!options_[OptFile].toString().empty()) {
			ret = sink->setFilePattern(options_[OptFile]);
			if (ret)
				return ret;
		}

		sink_ = std::move(sink);
	}

	if (sink_) {
		ret = sink_->configure(*config_);
		if (ret < 0) {
			std::cout << "Failed to configure frame sink"
				  << std::endl;
			return ret;
		}

		sink_->requestProcessed.connect(this, &CameraSession::sinkRelease);
	}

	allocator_ = std::make_unique<FrameBufferAllocator>(camera_);

	return startCapture();
}

void CameraSession::stop()
{
	int ret = camera_->stop();
	if (ret)
		std::cout << "Failed to stop capture" << std::endl;

	if (sink_) {
		ret = sink_->stop();
		if (ret)
			std::cout << "Failed to stop frame sink" << std::endl;
	}

	sink_.reset();

	requests_.clear();

	allocator_.reset();
}

int CameraSession::startCapture()
{
	int ret;

	/* Identify the stream with the least number of buffers. */
	unsigned int nbuffers = UINT_MAX;
	for (StreamConfiguration &cfg : *config_) {
		ret = allocator_->allocate(cfg.stream());
		if (ret < 0) {
			std::cerr << "Can't allocate buffers" << std::endl;
			return -ENOMEM;
		}

		unsigned int allocated = allocator_->buffers(cfg.stream()).size();
		nbuffers = std::min(nbuffers, allocated);
	}

	/*
	 * TODO: make cam tool smarter to support still capture by for
	 * example pushing a button. For now run all streams all the time.
	 */

	for (unsigned int i = 0; i < nbuffers; i++) {
		std::unique_ptr<Request> request = camera_->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			return -ENOMEM;
		}

		for (StreamConfiguration &cfg : *config_) {
			Stream *stream = cfg.stream();
			const std::vector<std::unique_ptr<FrameBuffer>> &buffers =
				allocator_->buffers(stream);
			const std::unique_ptr<FrameBuffer> &buffer = buffers[i];

			ret = request->addBuffer(stream, buffer.get());
			if (ret < 0) {
				std::cerr << "Can't set buffer for request"
					  << std::endl;
				return ret;
			}

			if (sink_)
				sink_->mapBuffer(buffer.get());
		}

		requests_.push_back(std::move(request));
	}

	if (sink_) {
		ret = sink_->start();
		if (ret) {
			std::cout << "Failed to start frame sink" << std::endl;
			return ret;
		}
	}

	ret = camera_->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		if (sink_)
			sink_->stop();
		return ret;
	}

	for (std::unique_ptr<Request> &request : requests_) {
		ret = queueRequest(request.get());
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
			camera_->stop();
			if (sink_)
				sink_->stop();
			return ret;
		}
	}

	if (captureLimit_)
		std::cout << "cam" << cameraIndex_
			  << ": Capture " << captureLimit_ << " frames"
			  << std::endl;
	else
		std::cout << "cam" << cameraIndex_
			  << ": Capture until user interrupts by SIGINT"
			  << std::endl;

	return 0;
}

int CameraSession::queueRequest(Request *request)
{
	if (captureLimit_ && queueCount_ >= captureLimit_)
		return 0;

	if (script_)
		request->controls() = script_->frameControls(queueCount_);

	queueCount_++;

	return camera_->queueRequest(request);
}

void CameraSession::requestComplete(Request *request)
{
	if (request->status() == Request::RequestCancelled)
		return;

	/*
	 * Defer processing of the completed request to the event loop, to avoid
	 * blocking the camera manager thread.
	 */
	EventLoop::instance()->callLater([this, request]() { processRequest(request); });
}

void CameraSession::processRequest(Request *request)
{
	/*
	 * If we've reached the capture limit, we're done. This doesn't
	 * duplicate the check below that emits the captureDone signal, as this
	 * function will be called for each request still in flight after the
	 * capture limit is reached and we don't want to emit the signal every
	 * single time.
	 */
	if (captureLimit_ && captureCount_ >= captureLimit_)
		return;

	const Request::BufferMap &buffers = request->buffers();

	/*
	 * Compute the frame rate. The timestamp is arbitrarily retrieved from
	 * the first buffer, as all buffers should have matching timestamps.
	 */
	uint64_t ts = buffers.begin()->second->metadata().timestamp;
	double fps = ts - last_;
	fps = last_ != 0 && fps ? 1000000000.0 / fps : 0.0;
	last_ = ts;

	bool requeue = true;

	std::stringstream info;
	info << ts / 1000000000 << "."
	     << std::setw(6) << std::setfill('0') << ts / 1000 % 1000000
	     << " (" << std::fixed << std::setprecision(2) << fps << " fps)";

	for (const auto &[stream, buffer] : buffers) {
		const FrameMetadata &metadata = buffer->metadata();

		info << " " << streamNames_[stream]
		     << " seq: " << std::setw(6) << std::setfill('0') << metadata.sequence
		     << " bytesused: ";

		unsigned int nplane = 0;
		for (const FrameMetadata::Plane &plane : metadata.planes()) {
			info << plane.bytesused;
			if (++nplane < metadata.planes().size())
				info << "/";
		}
	}

	if (sink_) {
		if (!sink_->processRequest(request))
			requeue = false;
	}

	std::cout << info.str() << std::endl;

	if (printMetadata_) {
		const ControlList &requestMetadata = request->metadata();
		for (const auto &[key, value] : requestMetadata) {
			const ControlId *id = controls::controls.at(key);
			std::cout << "\t" << id->name() << " = "
				  << value.toString() << std::endl;
		}
	}

	/*
	 * Notify the user that capture is complete if the limit has just been
	 * reached.
	 */
	captureCount_++;
	if (captureLimit_ && captureCount_ >= captureLimit_) {
		captureDone.emit();
		return;
	}

	/*
	 * If the frame sink holds on the request, we'll requeue it later in the
	 * complete handler.
	 */
	if (!requeue)
		return;

	request->reuse(Request::ReuseBuffers);
	queueRequest(request);
}

void CameraSession::sinkRelease(Request *request)
{
	request->reuse(Request::ReuseBuffers);
	queueRequest(request);
}
