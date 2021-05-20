/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * ipu3_awb.h - IPU3 AWB control algorithm
 */
#ifndef __LIBCAMERA_IPU3_AWB_H__
#define __LIBCAMERA_IPU3_AWB_H__

#include "ipu3_common.h"

#include <vector>

#include <linux/intel-ipu3.h>

#include <libcamera/geometry.h>

#include "libipa/algorithm.h"
#include "libipa/isp.h"

namespace libcamera {

namespace ipa::ipu3 {

class IPU3Awb : public Algorithm
{
public:
	IPU3Awb();
	~IPU3Awb();

	void initialise(ipu3_uapi_params &params, const Size &bdsOutputSize, struct ipu3_uapi_grid_config &bdsGrid);
	void process(const ipu3_uapi_stats_3a *stats);
	void updateWbParameters(ipu3_uapi_params &params, double agcGamma);

private:
	void generateZones(std::vector<RGB> &zones);
	void generateAwbStats(const ipu3_uapi_stats_3a *stats);
	void clearAwbStats();
	void awbGreyWorld();
	uint32_t estimateCCT(double red, double green, double blue);
	void calculateWBGains(const ipu3_uapi_stats_3a *stats);

	struct ipu3_uapi_grid_config awbGrid_;

	std::vector<RGB> zones_;
	IspStatsRegion awbStats_[kAwbStatsSizeX * kAwbStatsSizeY];
	AwbStatus asyncResults_;
	uint32_t minZonesCounted_;
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera*/
#endif /* __LIBCAMERA_IPU3_AWB_H__ */
