/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * SDL Sink
 */

#include "sdl_sink.h"

#include <assert.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <optional>
#include <signal.h>
#include <sstream>
#include <string.h>
#include <unistd.h>

#include <libcamera/camera.h>
#include <libcamera/formats.h>

#include "../common/event_loop.h"
#include "../common/image.h"

#include "sdl_texture_1plane.h"
#ifdef HAVE_LIBJPEG
#include "sdl_texture_mjpg.h"
#endif
#include "sdl_texture_yuv.h"

using namespace libcamera;

using namespace std::chrono_literals;

namespace {

std::optional<SDL_PixelFormatEnum> singlePlaneFormatToSDL(const libcamera::PixelFormat &f)
{
	switch (f) {
	case libcamera::formats::RGB888:
		return SDL_PIXELFORMAT_BGR24;
	case libcamera::formats::BGR888:
		return SDL_PIXELFORMAT_RGB24;
	case libcamera::formats::RGBA8888:
		return SDL_PIXELFORMAT_ABGR32;
	case libcamera::formats::ARGB8888:
		return SDL_PIXELFORMAT_BGRA32;
	case libcamera::formats::BGRA8888:
		return SDL_PIXELFORMAT_ARGB32;
	case libcamera::formats::ABGR8888:
		return SDL_PIXELFORMAT_RGBA32;
#if SDL_VERSION_ATLEAST(2, 29, 1)
	case libcamera::formats::RGBX8888:
		return SDL_PIXELFORMAT_XBGR32;
	case libcamera::formats::XRGB8888:
		return SDL_PIXELFORMAT_BGRX32;
	case libcamera::formats::BGRX8888:
		return SDL_PIXELFORMAT_XRGB32;
	case libcamera::formats::XBGR8888:
		return SDL_PIXELFORMAT_RGBX32;
#endif
	case libcamera::formats::YUYV:
		return SDL_PIXELFORMAT_YUY2;
	case libcamera::formats::UYVY:
		return SDL_PIXELFORMAT_UYVY;
	case libcamera::formats::YVYU:
		return SDL_PIXELFORMAT_YVYU;
	}

	return {};
}

} /* namespace */

SDLSink::SDLSink()
	: window_(nullptr), renderer_(nullptr), rect_({}),
	  init_(false)
{
}

SDLSink::~SDLSink()
{
	stop();
}

int SDLSink::configure(const libcamera::CameraConfiguration &config)
{
	int ret = FrameSink::configure(config);
	if (ret < 0)
		return ret;

	if (config.size() > 1) {
		std::cerr
			<< "SDL sink only supports one camera stream at present, streaming first camera stream"
			<< std::endl;
	} else if (config.empty()) {
		std::cerr << "Require at least one camera stream to process"
			  << std::endl;
		return -EINVAL;
	}

	const libcamera::StreamConfiguration &cfg = config.at(0);
	rect_.w = cfg.size.width;
	rect_.h = cfg.size.height;

	if (auto sdlFormat = singlePlaneFormatToSDL(cfg.pixelFormat))
		texture_ = std::make_unique<SDLTexture1Plane>(rect_, *sdlFormat, cfg.stride);
#ifdef HAVE_LIBJPEG
	else if (cfg.pixelFormat == libcamera::formats::MJPEG)
		texture_ = std::make_unique<SDLTextureMJPG>(rect_);
#endif
#if SDL_VERSION_ATLEAST(2, 0, 16)
	else if (cfg.pixelFormat == libcamera::formats::NV12)
		texture_ = std::make_unique<SDLTextureNV12>(rect_, cfg.stride);
#endif
	else {
		std::cerr << "Unsupported pixel format " << cfg.pixelFormat << std::endl;
		return -EINVAL;
	}

	return 0;
}

int SDLSink::start()
{
	int ret = SDL_Init(SDL_INIT_VIDEO);
	if (ret) {
		std::cerr << "Failed to initialize SDL: " << SDL_GetError()
			  << std::endl;
		return ret;
	}

	init_ = true;
	window_ = SDL_CreateWindow("", SDL_WINDOWPOS_UNDEFINED,
				   SDL_WINDOWPOS_UNDEFINED, rect_.w,
				   rect_.h,
				   SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
	if (!window_) {
		std::cerr << "Failed to create SDL window: " << SDL_GetError()
			  << std::endl;
		return -EINVAL;
	}

	renderer_ = SDL_CreateRenderer(window_, -1, 0);
	if (!renderer_) {
		std::cerr << "Failed to create SDL renderer: " << SDL_GetError()
			  << std::endl;
		return -EINVAL;
	}

	/*
	 * Set for scaling purposes, not critical, don't return in case of
	 * error.
	 */
	ret = SDL_RenderSetLogicalSize(renderer_, rect_.w, rect_.h);
	if (ret)
		std::cerr << "Failed to set SDL render logical size: "
			  << SDL_GetError() << std::endl;

	ret = texture_->create(renderer_);
	if (ret) {
		return ret;
	}

	/* \todo Make the event cancellable to support stop/start cycles. */
	EventLoop::instance()->addTimerEvent(
		10ms, std::bind(&SDLSink::processSDLEvents, this));

	return 0;
}

int SDLSink::stop()
{
	texture_.reset();

	if (renderer_) {
		SDL_DestroyRenderer(renderer_);
		renderer_ = nullptr;
	}

	if (window_) {
		SDL_DestroyWindow(window_);
		window_ = nullptr;
	}

	if (init_) {
		SDL_Quit();
		init_ = false;
	}

	return FrameSink::stop();
}

void SDLSink::mapBuffer(FrameBuffer *buffer)
{
	std::unique_ptr<Image> image =
		Image::fromFrameBuffer(buffer, Image::MapMode::ReadOnly);
	assert(image != nullptr);

	mappedBuffers_[buffer] = std::move(image);
}

bool SDLSink::processRequest(Request *request)
{
	for (auto [stream, buffer] : request->buffers()) {
		renderBuffer(buffer);
		break; /* to be expanded to launch SDL window per buffer */
	}

	return true;
}

/*
 * Process SDL events, required for things like window resize and quit button
 */
void SDLSink::processSDLEvents()
{
	for (SDL_Event e; SDL_PollEvent(&e);) {
		if (e.type == SDL_QUIT) {
			/* Click close icon then quit */
			EventLoop::instance()->exit(0);
		}
	}
}

void SDLSink::renderBuffer(FrameBuffer *buffer)
{
	Image *image = mappedBuffers_[buffer].get();

	std::vector<Span<const uint8_t>> planes;
	unsigned int i = 0;

	planes.reserve(buffer->metadata().planes().size());

	for (const FrameMetadata::Plane &meta : buffer->metadata().planes()) {
		Span<uint8_t> data = image->data(i);
		if (meta.bytesused > data.size())
			std::cerr << "payload size " << meta.bytesused
				  << " larger than plane size " << data.size()
				  << std::endl;

		planes.push_back(data);
		i++;
	}

	texture_->update(planes);

	SDL_RenderClear(renderer_);
	SDL_RenderCopy(renderer_, texture_->get(), nullptr, nullptr);
	SDL_RenderPresent(renderer_);
}
