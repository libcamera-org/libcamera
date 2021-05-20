/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * isp.h - ISP statistics interface
 */
#ifndef __LIBCAMERA_IPA_LIBIPA_ISP_H__
#define __LIBCAMERA_IPA_LIBIPA_ISP_H__

namespace libcamera {

namespace ipa {
/**
 * \struct RGB
 * \brief RGB
 *
 * \var RGB::R
 * \brief Red value of the RGB structure
 *
 * \var RGB::G
 * \brief Green value of the RGB structure
 *
 * \var RGB::B
 * \brief Blue value of the RGB structure
 */
struct RGB {
	RGB(double _R = 0, double _G = 0, double _B = 0)
		: R(_R), G(_G), B(_B)
	{
	}
	double R, G, B;
	RGB &operator+=(RGB const &other)
	{
		R += other.R, G += other.G, B += other.B;
		return *this;
	}
};

/**
 * \struct IspStatsRegion
 * \brief RGB statistics for a given region
 *
 * The IspStatsRegion structure is intended to abstract the ISP specific
 * statistics and use an agnostic algorithm to compute AWB.
 *
 * \var IspStatsRegion::counted
 * \brief Number of pixels used to calculate the sums
 *
 * \var IspStatsRegion::uncounted
 * \brief Remaining number of pixels in the region
 *
 * \var IspStatsRegion::rSum
 * \brief Sum of the red values in the region
 *
 * \var IspStatsRegion::gSum
 * \brief Sum of the green values in the region
 *
 * \var IspStatsRegion::bSum
 * \brief Sum of the blue values in the region
 */
struct IspStatsRegion {
	unsigned int counted;
	unsigned int uncounted;
	unsigned long long rSum;
	unsigned long long gSum;
	unsigned long long bSum;
};

/**
 * \struct AwbStatus
 * \brief AWB parameters calculated
 *
 * The AwbStatus structure is intended to store the AWB
 * parameters calculated by the algorithm
 *
 * \var AwbStatus::temperatureK
 * \brief Color temperature calculated
 *
 * \var AwbStatus::redGain
 * \brief Gain calculated for the red channel
 *
 * \var AwbStatus::greenGain
 * \brief Gain calculated for the green channel
 *
 * \var AwbStatus::blueGain
 * \brief Gain calculated for the blue channel
 */
struct AwbStatus {
	double temperatureK;
	double redGain;
	double greenGain;
	double blueGain;
};

} /* namespace ipa */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_LIBIPA_ISP_H__ */
