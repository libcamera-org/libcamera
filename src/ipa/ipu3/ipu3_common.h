/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * isp.h - ISP statistics interface
 */
#ifndef __LIBCAMERA_IPA_IPU3_COMMON_H__
#define __LIBCAMERA_IPA_IPU3_COMMON_H__

#include <stdint.h>

namespace libcamera {

namespace ipa {

/* Region size for the statistics generation algorithm */
static constexpr uint32_t kAwbStatsSizeX = 16;
static constexpr uint32_t kAwbStatsSizeY = 12;
static constexpr uint32_t kAwbStatsSize = kAwbStatsSizeX * kAwbStatsSizeY + 1;

static constexpr uint32_t kAgcStatsSizeX = 7;
static constexpr uint32_t kAgcStatsSizeY = 5;
static constexpr uint32_t kAgcStatsSize = kAgcStatsSizeX * kAgcStatsSizeY + 1;
static constexpr double kCenteredWeights[] = { 3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0 };
static constexpr uint32_t kAgcStatsRegions[] = {
	11, 9, 9, 9, 9, 9, 12,
	7, 5, 3, 3, 3, 6, 8,
	7, 5, 1, 0, 2, 6, 8,
	7, 5, 4, 4, 4, 6, 8,
	13, 10, 10, 10, 10, 10, 14
};

static constexpr uint32_t kMinGreenLevelInZone = 16;

struct Ipu3AwbCell {
	unsigned char greenRedAvg;
	unsigned char redAvg;
	unsigned char blueAvg;
	unsigned char greenBlueAvg;
	unsigned char satRatio;
	unsigned char padding[3];
} __attribute__((packed));

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_IPU3_COMMON_H__ */
