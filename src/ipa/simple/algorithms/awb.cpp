/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Auto white balance
 */

#include "awb.h"

#include <numeric>
#include <stdint.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "libipa/colours.h"
#include "simple/ipa_context.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftAwb)

namespace ipa::soft::algorithms {

int Awb::configure(IPAContext &context,
		   [[maybe_unused]] const IPAConfigInfo &configInfo)
{
	auto &gains = context.activeState.awb.gains;
	gains = { { 1.0, 1.0, 1.0 } };

	return 0;
}

void Awb::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  [[maybe_unused]] IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	const SwIspStats::Histogram &histogram = stats->yHistogram;
	const uint8_t blackLevel = context.activeState.blc.level;

	/*
	 * Black level must be subtracted to get the correct AWB ratios, they
	 * would be off if they were computed from the whole brightness range
	 * rather than from the sensor range.
	 */
	const uint64_t nPixels = std::accumulate(
		histogram.begin(), histogram.end(), 0);
	const uint64_t offset = blackLevel * nPixels;
	const uint64_t sumR = stats->sumR_ - offset / 4;
	const uint64_t sumG = stats->sumG_ - offset / 2;
	const uint64_t sumB = stats->sumB_ - offset / 4;

	/*
	 * Calculate red and blue gains for AWB.
	 * Clamp max gain at 4.0, this also avoids 0 division.
	 */
	auto &gains = context.activeState.awb.gains;
	gains = { {
		sumR <= sumG / 4 ? 4.0 : static_cast<double>(sumG) / sumR,
		1.0,
		sumB <= sumG / 4 ? 4.0 : static_cast<double>(sumG) / sumB,
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
