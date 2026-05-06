/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * Common image adjustments
 */

#include "adjust.h"

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace ipa::soft::algorithms {

LOG_DEFINE_CATEGORY(IPASoftAdjust)

int Adjust::init(IPAContext &context, const ValueNode &tuningData)
{
	defaultGamma_ = tuningData["gamma"].get<float>().value_or(kDefaultGamma);
	defaultContrast_ = tuningData["contrast"].get<float>().value_or(1.0f);
	defaultSaturation_ = tuningData["saturation"].get<float>().value_or(1.0f);

	context.ctrlMap[&controls::Gamma] =
		ControlInfo(0.1f, 10.0f, defaultGamma_);
	context.ctrlMap[&controls::Contrast] =
		ControlInfo(0.0f, 2.0f, defaultContrast_);
	if (context.ccmEnabled)
		context.ctrlMap[&controls::Saturation] =
			ControlInfo(0.0f, 2.0f, defaultSaturation_);

	return 0;
}

int Adjust::configure(IPAContext &context,
		      [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	context.activeState.knobs.gamma = defaultGamma_;
	context.activeState.knobs.contrast = defaultContrast_;
	context.activeState.knobs.saturation = defaultSaturation_;

	return 0;
}

void Adjust::queueRequest(typename Module::Context &context,
			  [[maybe_unused]] const uint32_t frame,
			  [[maybe_unused]] typename Module::FrameContext &frameContext,
			  const ControlList &controls)
{
	const auto &gamma = controls.get(controls::Gamma);
	if (gamma.has_value()) {
		context.activeState.knobs.gamma = gamma.value();
		LOG(IPASoftAdjust, Debug) << "Setting gamma to " << gamma.value();
	}

	const auto &contrast = controls.get(controls::Contrast);
	if (contrast.has_value()) {
		context.activeState.knobs.contrast = contrast.value();
		LOG(IPASoftAdjust, Debug) << "Setting contrast to " << contrast.value();
	}

	const auto &saturation = controls.get(controls::Saturation);
	if (saturation.has_value()) {
		context.activeState.knobs.saturation = saturation.value();
		LOG(IPASoftAdjust, Debug) << "Setting saturation to " << saturation.value();
	}
}

void Adjust::applySaturation(Matrix<float, 3, 3> &matrix, float saturation)
{
	/* https://en.wikipedia.org/wiki/YCbCr#ITU-R_BT.601_conversion */
	const Matrix<float, 3, 3> rgb2ycbcr{
		{ 0.256788235294, 0.504129411765, 0.0979058823529,
		  -0.148223529412, -0.290992156863, 0.439215686275,
		  0.439215686275, -0.367788235294, -0.0714274509804 }
	};
	const Matrix<float, 3, 3> ycbcr2rgb{
		{ 1.16438356164, 0, 1.59602678571,
		  1.16438356164, -0.391762290094, -0.812967647235,
		  1.16438356164, 2.01723214285, 0 }
	};
	const Matrix<float, 3, 3> saturationMatrix{
		{ 1, 0, 0,
		  0, saturation, 0,
		  0, 0, saturation }
	};
	matrix =
		ycbcr2rgb * saturationMatrix * rgb2ycbcr * matrix;
}

void Adjust::prepare(IPAContext &context,
		     [[maybe_unused]] const uint32_t frame,
		     IPAFrameContext &frameContext,
		     DebayerParams *params)
{
	frameContext.gamma = context.activeState.knobs.gamma;
	frameContext.contrast = context.activeState.knobs.contrast;

	const float saturation = context.activeState.knobs.saturation;
	if (context.ccmEnabled) {
		applySaturation(context.activeState.combinedMatrix, saturation);
		frameContext.saturation = saturation;
	}

	params->gamma = 1.0 / context.activeState.knobs.gamma;
	params->contrastExp = tan(std::clamp(context.activeState.knobs.contrast * M_PI_4,
					     0.0, M_PI_2 - 0.00001));
}

void Adjust::process([[maybe_unused]] IPAContext &context,
		     [[maybe_unused]] const uint32_t frame,
		     IPAFrameContext &frameContext,
		     [[maybe_unused]] const SwIspStats *stats,
		     ControlList &metadata)
{
	metadata.set(controls::Gamma, frameContext.gamma);
	metadata.set(controls::Contrast, frameContext.contrast);
	metadata.set(controls::Saturation, frameContext.saturation);
}

REGISTER_IPA_ALGORITHM(Adjust, "Adjust")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
