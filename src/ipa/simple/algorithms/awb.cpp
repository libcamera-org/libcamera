/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2026 Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

#include "libipa/colours.h"
#include "simple/ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

int Awb::init([[maybe_unused]] IPAContext &context,
	      const ValueNode &tuningData)
{
	maxGainR_ = tuningData["maxGainR"].get<float>().value_or(4.0f);
	maxGainB_ = tuningData["maxGainB"].get<float>().value_or(4.0f);
	speed_ = tuningData["speed"].get<float>().value_or(1.0f);

	LOG(IPASoftAwb, Debug)
		<< "AWB: maxGainR " << maxGainR_
		<< ", maxGainB " << maxGainB_
		<< ", speed " << speed_;

	return 0;
}

int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &gains = context.activeState.awb.gains;
	gains = { { 1.0, 1.0, 1.0 } };

	return 0;
}

void Awb::prepare(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  DebayerParams *params)
{
	auto &gains = context.activeState.awb.gains;
	/*
	 * Store AWB gains in params for the shader to apply separately.
	 * AWB gains are NOT baked into combinedMatrix so that the CCM always
	 * receives a clamped [0,1] white-balanced signal (see shader).
	 */
	params->gains = gains;

	frameContext.gains.red = gains.r();
	frameContext.gains.blue = gains.b();
}

void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	const float mdGains[] = {
		static_cast<float>(frameContext.gains.red),
		static_cast<float>(frameContext.gains.blue)
	};
	metadata.set(controls::ColourGains, mdGains);

	if (!stats->valid)
		return;

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), uint64_t(0));
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t minValid = 1;
	/*
	 * Make sure the sums are at least minValid, while preventing unsigned
	 * integer underflow.
	 */
	const RGB<uint64_t> sum = stats->sum_.max(offset + minValid) - offset;

	/*
	 * Calculate red and blue gains for AWB. Clamp max gain to avoid
	 * division by zero and extreme color casts.
	 */
	auto &gains = context.activeState.awb.gains;
	float rawRGain = sum.r() <= sum.g() / maxGainR_ ? maxGainR_ :
				static_cast<float>(sum.g()) / sum.r();
	float rawBGain = sum.b() <= sum.g() / maxGainB_ ? maxGainB_ :
				static_cast<float>(sum.g()) / sum.b();

	/* Apply temporal smoothing to avoid rapid white balance changes. */
	float alpha = std::clamp(speed_, 0.01f, 1.0f);
	gains = { {
		gains.r() * (1.0f - alpha) + rawRGain * alpha,
		1.0f,
		gains.b() * (1.0f - alpha) + rawBGain * alpha,
	} };

	RGB<double> rgbGains{ { 1 / gains.r(), 1 / gains.g(), 1 / gains.b() } };
	context.activeState.awb.temperatureK = estimateCCT(rgbGains);
	metadata.set(controls::ColourTemperature, context.activeState.awb.temperatureK);

	LOG(IPASoftAwb, Debug)
		<< "gain R/B: " << gains << "; temperature: "
		<< context.activeState.awb.temperatureK;
}

REGISTER_IPA_ALGORITHM(Awb, "Awb")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
