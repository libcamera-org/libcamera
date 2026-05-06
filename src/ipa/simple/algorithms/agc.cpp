/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#include "agc.h"

#include <algorithm>
#include <cmath>
#include <stdint.h>

#include <libcamera/base/log.h>

#include "control_ids.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPASoftExposure)

namespace ipa::soft::algorithms {

/*
 * The number of bins to use for the optimal exposure calculations.
 */
static constexpr unsigned int kExposureBinsCount = 5;

/*
 * The exposure is optimal when the mean sample value of the histogram is
 * in the middle of the range. Overridable via YAML exposureTarget.
 */
static constexpr float kExposureTargetDefault = kExposureBinsCount / 2.0;

/*
 * This implements the hysteresis for the exposure adjustment.
 * It is small enough to have the exposure close to the optimal, and is big
 * enough to prevent the exposure from wobbling around the optimal value.
 */
static constexpr float kHysteresisDefault = 0.2;

/*
 * Proportional gain for exposure/gain adjustment. Maps the MSV error to a
 * multiplicative correction factor:
 *
 *   factor = 1.0 + proportionalGain_ * error
 *
 * With proportionalGain_ = 0.04:
 *   - max error ~2.5 -> factor 1.10 (~10% step, same as before)
 *   - error 1.0      -> factor 1.04 (~4% step)
 *   - error 0.3      -> factor 1.012 (~1.2% step)
 *
 * Overridable via YAML proportionalGain.
 */
static constexpr float kProportionalGainDefault = 0.04;

Agc::Agc()
{
}

int Agc::init([[maybe_unused]] IPAContext &context, const ValueNode &tuningData)
{
	exposureTarget_ = tuningData["exposureTarget"].get<float>()
		.value_or(kExposureTargetDefault);
	hysteresis_ = tuningData["hysteresis"].get<float>()
		.value_or(kHysteresisDefault);
	proportionalGain_ = tuningData["proportionalGain"].get<float>()
		.value_or(kProportionalGainDefault);

	return 0;
}

void Agc::updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV)
{
	int32_t &exposure = frameContext.sensor.exposure;
	double &again = frameContext.sensor.gain;

	double error = exposureTarget_ - exposureMSV;

	if (std::abs(error) <= hysteresis_)
		return;

	/*
	 * Compute a proportional correction factor. The sign of the error
	 * determines the direction: positive error means too dark (increase),
	 * negative means too bright (decrease).
	 */
	float factor = 1.0f + static_cast<float>(error) * proportionalGain_;

	if (factor > 1.0f) {
		/* Scene too dark: increase exposure first, then gain. */
		if (exposure < context.configuration.agc.exposureMax) {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::max(next, exposure + 1);
		} else {
			double next = again * factor;
			if (next - again < context.configuration.agc.againMinStep)
				again += context.configuration.agc.againMinStep;
			else
				again = next;
		}
	} else {
		/* Scene too bright: decrease gain first, then exposure. */
		if (again > context.configuration.agc.again10) {
			double next = again * factor;
			if (again - next < context.configuration.agc.againMinStep)
				again -= context.configuration.agc.againMinStep;
			else
				again = next;
		} else {
			int32_t next = static_cast<int32_t>(exposure * factor);
			exposure = std::min(next, exposure - 1);
		}
	}

	exposure = std::clamp(exposure, context.configuration.agc.exposureMin,
			      context.configuration.agc.exposureMax);
	again = std::clamp(again, context.configuration.agc.againMin,
			   context.configuration.agc.againMax);

	context.activeState.agc.exposure = exposure;
	context.activeState.agc.again = again;

	LOG(IPASoftExposure, Debug)
		<< "exposureMSV " << exposureMSV
		<< " error " << error << " factor " << factor
		<< " exp " << exposure << " again " << again;
}

void Agc::process(IPAContext &context,
		  [[maybe_unused]] const uint32_t frame,
		  IPAFrameContext &frameContext,
		  const SwIspStats *stats,
		  ControlList &metadata)
{
	utils::Duration exposureTime =
		context.configuration.agc.lineDuration * frameContext.sensor.exposure;
	metadata.set(controls::ExposureTime, exposureTime.get<std::micro>());
	metadata.set(controls::AnalogueGain, frameContext.sensor.gain);

	if (!context.activeState.agc.valid) {
		/*
		 * Init active-state from sensor values in case updateExposure()
		 * does not run for the first frame.
		 */
		context.activeState.agc.exposure = frameContext.sensor.exposure;
		context.activeState.agc.again = frameContext.sensor.gain;
		context.activeState.agc.valid = true;
	}

	if (!stats->valid) {
		/*
		 * Use the new exposure and gain values calculated the last time
		 * there were valid stats.
		 */
		frameContext.sensor.exposure = context.activeState.agc.exposure;
		frameContext.sensor.gain = context.activeState.agc.again;
		return;
	}

	/*
	 * Calculate Mean Sample Value (MSV) according to formula from:
	 * https://www.araa.asn.au/acra/acra2007/papers/paper84final.pdf
	 */
	const auto &histogram = stats->yHistogram;
	const unsigned int blackLevelHistIdx =
		context.activeState.blc.level / (256 / SwIspStats::kYHistogramSize);
	const unsigned int histogramSize =
		SwIspStats::kYHistogramSize - blackLevelHistIdx;
	const unsigned int yHistValsPerBin = histogramSize / kExposureBinsCount;
	const unsigned int yHistValsPerBinMod =
		histogramSize / (histogramSize % kExposureBinsCount + 1);
	int exposureBins[kExposureBinsCount] = {};
	unsigned int denom = 0;
	unsigned int num = 0;

	if (yHistValsPerBin == 0) {
		LOG(IPASoftExposure, Debug)
			<< "Not adjusting exposure due to insufficient histogram data";
		return;
	}

	for (unsigned int i = 0; i < histogramSize; i++) {
		unsigned int idx = (i - (i / yHistValsPerBinMod)) / yHistValsPerBin;
		exposureBins[idx] += histogram[blackLevelHistIdx + i];
	}

	for (unsigned int i = 0; i < kExposureBinsCount; i++) {
		LOG(IPASoftExposure, Debug) << i << ": " << exposureBins[i];
		denom += exposureBins[i];
		num += exposureBins[i] * (i + 1);
	}

	float exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);
	updateExposure(context, frameContext, exposureMSV);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
