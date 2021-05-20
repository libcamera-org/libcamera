/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_agc.cpp - AGC/AEC control algorithm
 */

#include "ipu3_agc.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdint.h>

#include <linux/v4l2-controls.h>

#include "libcamera/internal/log.h"

#include "libipa/histogram.h"

namespace libcamera {

namespace ipa::ipu3 {

LOG_DEFINE_CATEGORY(IPU3Agc)

/* Histogram constants */
static constexpr uint32_t knumHistogramBins = 256;

/* seems to be a 8-bit pipeline */
static constexpr uint8_t kPipelineBits = 8;

IPU3Agc::IPU3Agc()
	: frameCount_(0), lastFrame_(0), converged_(false),
	  updateControls_(false), iqMean_(0.0), gamma_(1.4),
	  prevExposure_(0.0), prevExposureNoDg_(0.0),
	  currentExposure_(0.0), currentExposureNoDg_(0.0),
	  currentShutter_(1.0), currentAnalogueGain_(1.0)
{
}

void IPU3Agc::initialise(struct ipu3_uapi_grid_config &bdsGrid, const IPAConfigInfo &configInfo)
{
	aeGrid_ = bdsGrid;
	ctrls_ = configInfo.entityControls.at(0);

	const auto itExp = ctrls_.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls_.end()) {
		LOG(IPU3Agc, Debug) << "Can't find exposure control";
		return;
	}
	minExposure_ = itExp->second.min().get<int32_t>();
	maxExposure_ = itExp->second.max().get<int32_t>();
	lineDuration_ = configInfo.sensorInfo.lineLength / (configInfo.sensorInfo.pixelRate / 1e6);
	maxExposureTime_ = maxExposure_ * lineDuration_;

	const auto itGain = ctrls_.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls_.end()) {
		LOG(IPU3Agc, Debug) << "Can't find gain control";
		return;
	}
	minGain_ = std::max(itGain->second.min().get<int32_t>(), 1);
	maxGain_ = itGain->second.max().get<int32_t>();

	/* \todo: those values need to be extracted from a configuration file */
	shutterConstraints_.push_back(100);
	shutterConstraints_.push_back(10000);
	shutterConstraints_.push_back(33000);
	gainConstraints_.push_back(1.0);
	gainConstraints_.push_back(4.0);
	gainConstraints_.push_back(16.0);

	fixedShutter_ = 0.0;
	fixedAnalogueGain_ = 0.0;
}

/* Translate the IPU3 statistics into the default statistics region array */
void IPU3Agc::generateStats(const ipu3_uapi_stats_3a *stats)
{
	uint32_t regionWidth = round(aeGrid_.width / static_cast<double>(kAgcStatsSizeX));
	uint32_t regionHeight = round(aeGrid_.height / static_cast<double>(kAgcStatsSizeY));
	uint32_t hist[knumHistogramBins] = { 0 };

	LOG(IPU3Agc, Debug) << "[" << regionWidth << "x" << regionHeight << "] regions";

	/*
	 * Generate a (kAgcStatsSizeX x kAgcStatsSizeY) array from the IPU3 grid which is
	 * (aeGrid_.width x aeGrid_.height).
	 */
	for (unsigned int j = 0; j < kAgcStatsSizeY * regionHeight; j++) {
		for (unsigned int i = 0; i < kAgcStatsSizeX * regionWidth; i++) {
			uint32_t cellPosition = j * aeGrid_.width + i;
			uint32_t cellX = (cellPosition / regionWidth) % kAgcStatsSizeX;
			uint32_t cellY = ((cellPosition / aeGrid_.width) / regionHeight) % kAgcStatsSizeY;

			uint32_t agcRegionPosition = kAgcStatsRegions[cellY * kAgcStatsSizeX + cellX];
			cellPosition *= sizeof(Ipu3AwbCell);

			/* Cast the initial IPU3 structure to simplify the reading */
			Ipu3AwbCell *currentCell = reinterpret_cast<Ipu3AwbCell *>(const_cast<uint8_t *>(&stats->awb_raw_buffer.meta_data[cellPosition]));
			if (currentCell->satRatio == 0) {
				/* The cell is not saturated, use the current cell */
				agcStats_[agcRegionPosition].counted++;
				uint32_t greenValue = currentCell->greenRedAvg + currentCell->greenBlueAvg;
				hist[greenValue / 2]++;
				agcStats_[agcRegionPosition].gSum += greenValue / 2;
				agcStats_[agcRegionPosition].rSum += currentCell->redAvg;
				agcStats_[agcRegionPosition].bSum += currentCell->blueAvg;
			}
		}
	}

	/* Estimate the quantile mean of the top 2% of the histogram */
	iqMean_ = Histogram(Span<uint32_t>(hist)).interQuantileMean(0.98, 1.0);
}

void IPU3Agc::clearStats()
{
	for (unsigned int i = 0; i < kAgcStatsSize; i++) {
		agcStats_[i].bSum = 0;
		agcStats_[i].rSum = 0;
		agcStats_[i].gSum = 0;
		agcStats_[i].counted = 0;
		agcStats_[i].uncounted = 0;
	}

	awb_.blueGain = 1.0;
	awb_.greenGain = 1.0;
	awb_.redGain = 1.0;

	std::copy(std::begin(kCenteredWeights), std::end(kCenteredWeights), std::begin(weights_));
}

void IPU3Agc::filterExposure()
{
	double speed = 0.08;
	if (prevExposure_ == 0.0) {
		/* DG stands for digital gain.*/
		prevExposure_ = currentExposure_;
		prevExposureNoDg_ = currentExposureNoDg_;
	} else {
		/*
		 * If we are close to the desired result, go faster to avoid making
		 * multiple micro-adjustments.
		 * \ todo: Make this customisable?
		 */
		if (prevExposure_ < 1.2 * currentExposure_ &&
		    prevExposure_ > 0.8 * currentExposure_)
			speed = sqrt(speed);

		prevExposure_ = speed * currentExposure_ +
				prevExposure_ * (1.0 - speed);
		prevExposureNoDg_ = speed * currentExposureNoDg_ +
				prevExposureNoDg_ * (1.0 - speed);
	}
	/*
	 * We can't let the no_dg exposure deviate too far below the
	 * total exposure, as there might not be enough digital gain available
	 * in the ISP to hide it (which will cause nasty oscillation).
	 */
	double fastReduceThreshold = 0.3;
	if (prevExposureNoDg_ <
	    prevExposure_ * fastReduceThreshold)
		prevExposureNoDg_ = prevExposure_ * fastReduceThreshold;
	LOG(IPU3Agc, Debug) << "After filtering, total_exposure " << prevExposure_;
}

double IPU3Agc::computeInitialY(IspStatsRegion regions[], AwbStatus const &awb,
				double weights[], double gain)
{
	/* Note how the calculation below means that equal weights give you
	 * "average" metering (i.e. all pixels equally important). */
	double redSum = 0, greenSum = 0, blueSum = 0, pixelSum = 0;
	for (unsigned int i = 0; i < kAwbStatsSizeX * kAwbStatsSizeY; i++) {
		double counted = regions[i].counted;
		double rSum = std::min(regions[i].rSum * gain, ((1 << kPipelineBits) - 1) * counted);
		double gSum = std::min(regions[i].gSum * gain, ((1 << kPipelineBits) - 1) * counted);
		double bSum = std::min(regions[i].bSum * gain, ((1 << kPipelineBits) - 1) * counted);
		redSum += rSum * weights[i];
		greenSum += gSum * weights[i];
		blueSum += bSum * weights[i];
		pixelSum += counted * weights[i];
	}
	if (pixelSum == 0.0) {
		LOG(IPU3Agc, Warning) << "computeInitialY: pixel_sum is zero";
		return 0;
	}
	double Y_sum = redSum * awb.redGain * .299 +
		       greenSum * awb.greenGain * .587 +
		       blueSum * awb.blueGain * .114;
	return Y_sum / pixelSum / (1 << kPipelineBits);
}

void IPU3Agc::computeTargetExposure(double gain)
{
	currentExposure_ = currentExposureNoDg_ * gain;
	/* \todo: have a list of shutter speeds */
	double maxShutterSpeed = shutterConstraints_.back();
	double maxTotalExposure = maxShutterSpeed * gainConstraints_.back();

	currentExposure_ = std::min(currentExposure_, maxTotalExposure);
	LOG(IPU3Agc, Debug) << "Target total_exposure " << currentExposure_;
}

void IPU3Agc::divideUpExposure()
{
	double exposureValue = prevExposure_;
	double shutterTime, analogueGain;
	shutterTime = shutterConstraints_[0];
	shutterTime = std::min(shutterTime, shutterConstraints_.back());
	analogueGain = gainConstraints_[0];

	if (shutterTime * analogueGain < exposureValue) {
		for (unsigned int stage = 1;
		     stage < gainConstraints_.size(); stage++) {
			if (fixedShutter_ == 0.0) {
				double stageShutter =
					std::min(shutterConstraints_[stage], shutterConstraints_.back());
				if (stageShutter * analogueGain >=
				    exposureValue) {
					shutterTime =
						exposureValue / analogueGain;
					break;
				}
				shutterTime = stageShutter;
			}
			if (fixedAnalogueGain_ == 0.0) {
				if (gainConstraints_[stage] * shutterTime >= exposureValue) {
					analogueGain = exposureValue / shutterTime;
					break;
				}
				analogueGain = gainConstraints_[stage];
			}
		}
	}
	LOG(IPU3Agc, Debug) << "Divided up shutter and gain are " << shutterTime << " and "
			    << analogueGain;

	/* \todo: flickering avoidance ? */
	filteredShutter_ = shutterTime;
	filteredAnalogueGain_ = analogueGain;
}

void IPU3Agc::computeGain(double &currentGain)
{
	currentGain = 1.0;
	/* \todo: the target Y needs to be grabbed from a configuration */
	double targetY = 0.162;
	for (int i = 0; i < 8; i++) {
		double initialY = computeInitialY(agcStats_, awb_, weights_, currentGain);
		double extra_gain = std::min(10.0, targetY / (initialY + .001));

		currentGain *= extra_gain;
		LOG(IPU3Agc, Debug) << "Initial Y " << initialY << " target " << targetY
				    << " gives gain " << currentGain;
		if (extra_gain < 1.01)
			break;
	}

	double newGain = 128 / iqMean_;
	LOG(IPU3Agc, Debug) << "gain: " << currentGain << " new gain: " << newGain;
}

void IPU3Agc::process(const ipu3_uapi_stats_3a *stats, uint32_t &exposure, uint32_t &analogueGain)
{
	ASSERT(stats->stats_3a_status.awb_en);
	clearStats();
	generateStats(stats);
	currentShutter_ = exposure * lineDuration_;
	/* \todo: the gain needs to be calculated based on sensor informations */
	currentAnalogueGain_ = std::max(minGain_, analogueGain / 16);
	currentExposureNoDg_ = currentShutter_ * currentAnalogueGain_;

	double currentGain;
	computeGain(currentGain);
	computeTargetExposure(currentGain);
	filterExposure();
	divideUpExposure();

	exposure = filteredShutter_ / lineDuration_;
	analogueGain = filteredAnalogueGain_ * 16;

	updateControls_ = true;
	frameCount_++;
}

} /* namespace ipa::ipu3 */

} /* namespace libcamera */
