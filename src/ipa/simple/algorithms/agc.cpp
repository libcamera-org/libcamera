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
 * Damping coefficient for the exposure approach curve.
 *
 * On each frame we compute the *full* correction factor needed to reach the
 * target (correctionFull = exposureTarget / exposureMSV) and then move
 * a fraction of that distance:
 *
 *   exposureNew = exposureCurrent * (1 - damping + damping * correctionFull)
 *
 * Equivalently in log/stops space this is an exponential approach. A damping
 * of 1.0 jumps directly to the target (no smoothing); 0.0 never moves.
 * The default 0.25 reaches ~94% of the target in 10 frames -- smooth without
 * being sluggish.
 *
 * Overridable via YAML damping.
 */
static constexpr float kDampingDefault = 0.25f;

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

/*
 * Percentile of the luminance histogram used for metering.
 * 0.5 = median (expose for the middle pixel), 1.0 = brightest pixel.
 * Values below 1.0 protect highlights: e.g. 0.9 means expose so that
 * 90% of pixels are below the target bin, preventing bright areas from
 * blowing out at the expense of slightly darker midtones.
 * Overridable via YAML meteringPercentile.
 */
static constexpr float kMeteringPercentileDefault = 1.0;

/*
 * Asymmetric EMA alpha for the metered MSV.
 * Two separate smoothing constants:
 *   - alphaUp: applied when MSV rises (scene gets darker -> increase exposure).
 *     Slower response avoids pumping up exposure on transient dark frames.
 *   - alphaDown: applied when MSV falls (scene gets brighter -> decrease exposure).
 *     Faster response prevents overexposure when a bright scene is encountered.
 * Range: (0, 1]. 1.0 = no filtering (instant response).
 * Overridable via YAML msvFilterAlphaUp / msvFilterAlphaDown.
 */
static constexpr float kMsvFilterAlphaUpDefault = 0.2f;
static constexpr float kMsvFilterAlphaDownDefault = 0.6f;

Agc::Agc()
	: filteredMSV_(-1.0f)
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
	damping_ = std::clamp(
		tuningData["damping"].get<float>().value_or(kDampingDefault),
		0.01f, 1.0f);
	meteringPercentile_ = tuningData["meteringPercentile"].get<float>()
		.value_or(kMeteringPercentileDefault);
	msvFilterAlphaUp_ = std::clamp(
		tuningData["msvFilterAlphaUp"].get<float>().value_or(kMsvFilterAlphaUpDefault),
		0.01f, 1.0f);
	msvFilterAlphaDown_ = std::clamp(
		tuningData["msvFilterAlphaDown"].get<float>().value_or(kMsvFilterAlphaDownDefault),
		0.01f, 1.0f);

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
	 * Compute the full correction factor needed to reach the target,
	 * then move only a fraction (damping_) of the way there. This produces
	 * a smooth exponential approach curve rather than discrete steps.
	 *
	 * correctionFull = exposureTarget / exposureMSV
	 * factor = 1 + damping * (correctionFull - 1)
	 *
	 * Clamp exposureMSV to a small positive number to avoid division by
	 * zero and runaway correction factors on near-black frames.
	 */
	const double msvClamped = std::max(exposureMSV, 0.1);
	const double correctionFull = exposureTarget_ / msvClamped;
	float factor = 1.0f + damping_ * static_cast<float>(correctionFull - 1.0);

	/*
	 * Limit the per-frame factor to a reasonable range to prevent extreme
	 * jumps if the metering is briefly very wrong (e.g. occlusion, sudden
	 * scene change). The bounds also ensure stability of the asymptotic
	 * convergence.
	 */
	factor = std::clamp(factor, 0.5f, 2.0f);

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
		<< " error " << error
		<< " correctionFull " << correctionFull
		<< " factor " << factor
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

	float exposureMSV;
	if (meteringPercentile_ >= 1.0f) {
		/* Default: mean sample value across all bins. */
		exposureMSV = (denom == 0 ? 0 : static_cast<float>(num) / denom);
	} else {
		/*
		 * Percentile metering: find the histogram bin (in the full
		 * 64-bin space) at which the cumulative pixel count reaches
		 * meteringPercentile_ of total pixels.
		 *
		 * We then express the result directly on the kExposureBinsCount
		 * MSV scale so it can be compared to exposureTarget.  The
		 * exposureTarget should be set close to kExposureBinsCount
		 * (e.g. 4.5 out of 5) so that the percentile pixel lands near
		 * the top of the range -- protecting highlights while keeping
		 * the subject bright.
		 *
		 * Example: meteringPercentile_=0.90, exposureTarget=4.5 means
		 * "expose so the 90th percentile pixel is at 90% of full scale".
		 */
		unsigned int totalPixels = denom;
		unsigned int threshold = static_cast<unsigned int>(totalPixels * meteringPercentile_);
		unsigned int cumulative = 0;
		unsigned int percentileHistBin = histogramSize - 1;
		for (unsigned int i = 0; i < histogramSize; i++) {
			cumulative += histogram[blackLevelHistIdx + i];
			if (cumulative >= threshold) {
				percentileHistBin = i;
				break;
			}
		}
		/* Map from [0, histogramSize) to [1, kExposureBinsCount]. */
		exposureMSV = 1.0f + static_cast<float>(percentileHistBin) *
			      (kExposureBinsCount - 1) / (histogramSize - 1);
		LOG(IPASoftExposure, Debug)
			<< "percentile " << meteringPercentile_
			<< " -> histBin " << percentileHistBin
			<< " (MSV=" << exposureMSV << ")";
	}

	/*
	 * Apply an asymmetric EMA to the metered MSV:
	 * - When MSV rises (scene darker, need more exposure): smooth slowly to
	 *   avoid pumping exposure up on transient dark frames.
	 * - When MSV falls (scene brighter, need less exposure): react quickly
	 *   to prevent overexposure.
	 * Seed the filter on the first valid frame.
	 */
	if (filteredMSV_ < 0.0f) {
		filteredMSV_ = exposureMSV;
	} else {
		float alpha = (exposureMSV > filteredMSV_) ? msvFilterAlphaUp_
							   : msvFilterAlphaDown_;
		filteredMSV_ = alpha * exposureMSV + (1.0f - alpha) * filteredMSV_;
	}

	LOG(IPASoftExposure, Debug)
		<< "raw MSV=" << exposureMSV << " filtered=" << filteredMSV_;

	updateExposure(context, frameContext, filteredMSV_);
}

REGISTER_IPA_ALGORITHM(Agc, "Agc")

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
