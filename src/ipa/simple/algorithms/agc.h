/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Exposure and gain
 */

#pragma once

#include <libcamera/internal/yaml_parser.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class Agc : public Algorithm
{
public:
	Agc();
	~Agc() = default;

	int init(IPAContext &context, const ValueNode &tuningData) override;

	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	void updateExposure(IPAContext &context, IPAFrameContext &frameContext, double exposureMSV);

	float exposureTarget_;
	float hysteresis_;
	float proportionalGain_;
	float damping_;
	float meteringPercentile_;
	float msvFilterAlphaUp_;
	float msvFilterAlphaDown_;
	float filteredMSV_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
