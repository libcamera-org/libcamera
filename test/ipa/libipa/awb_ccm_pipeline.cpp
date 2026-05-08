/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026, Red Hat Inc.
 *
 * AWB + CCM pipeline highlight clipping tests
 *
 * The CCM rows sum to 1.0, meaning it is designed to receive a
 * white-balanced signal in [0,1].  AWB gains must be applied and clamped
 * to [0,1] BEFORE the CCM multiply; otherwise the combined matrix has row
 * sums far from 1.0 and bright neutral pixels produce magenta/cyan output.
 *
 * These tests verify:
 *   1. The old (broken) approach: baking AWB into the matrix causes magenta.
 *   2. The new (correct) approach: separate AWB+clamp then CCM gives neutral.
 *   3. Mid-tones are unaffected by the clamp (no colour shift for normal pixels).
 *   4. The fix works across multiple CCM colour temperatures.
 */

#include <cmath>
#include <iostream>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

#include "test.h"

using namespace std;
using namespace libcamera;

/* Tolerance for output channel comparison. */
static constexpr float kTol = 1e-3f;

#define ASSERT_TRUE(cond)                                        \
	do {                                                     \
		if (!(cond)) {                                   \
			cerr << "FAIL: " #cond " (line "        \
			     << __LINE__ << ")\n";               \
			return TestFail;                         \
		}                                                \
	} while (0)

#define ASSERT_NEAR(a, b, tol)                                   \
	do {                                                     \
		if (std::abs((a) - (b)) > (tol)) {               \
			cerr << "FAIL: |" << (a) << " - "       \
			     << (b) << "| > " << (tol)           \
			     << " (line " << __LINE__ << ")\n";  \
			return TestFail;                         \
		}                                                \
	} while (0)

/*
 * Apply AWB gains by baking them into the CCM (old broken approach).
 * Returns the output RGB for a given raw input pixel.
 */
static RGB<float> applyBakedMatrix(const Matrix<float, 3, 3> &ccm,
				   const RGB<float> &awbGains,
				   const RGB<float> &rawPixel)
{
	/* Combined = CCM * diag(awbGains) */
	Matrix<float, 3, 3> awbDiag = { { awbGains.r(), 0, 0,
					  0, awbGains.g(), 0,
					  0, 0, awbGains.b() } };
	Matrix<float, 3, 3> combined = ccm * awbDiag;

	float r = combined[0][0] * rawPixel.r() + combined[0][1] * rawPixel.g() + combined[0][2] * rawPixel.b();
	float g = combined[1][0] * rawPixel.r() + combined[1][1] * rawPixel.g() + combined[1][2] * rawPixel.b();
	float b = combined[2][0] * rawPixel.r() + combined[2][1] * rawPixel.g() + combined[2][2] * rawPixel.b();

	return RGB<float>({ r, g, b });
}

/*
 * Apply AWB gains with clamp to [0,1], then CCM (new correct approach).
 * Returns the output RGB for a given raw input pixel.
 */
static RGB<float> applyClampedAwbThenCcm(const Matrix<float, 3, 3> &ccm,
					  const RGB<float> &awbGains,
					  const RGB<float> &rawPixel)
{
	/* Step 1: AWB gains + clamp */
	float rin = std::clamp(rawPixel.r() * awbGains.r(), 0.0f, 1.0f);
	float gin = std::clamp(rawPixel.g() * awbGains.g(), 0.0f, 1.0f);
	float bin = std::clamp(rawPixel.b() * awbGains.b(), 0.0f, 1.0f);

	/* Step 2: CCM multiply */
	float r = ccm[0][0] * rin + ccm[0][1] * gin + ccm[0][2] * bin;
	float g = ccm[1][0] * rin + ccm[1][1] * gin + ccm[1][2] * bin;
	float b = ccm[2][0] * rin + ccm[2][1] * gin + ccm[2][2] * bin;

	return RGB<float>({ r, g, b });
}

/*
 * Returns true if the pixel is "magenta" -- R and B significantly higher
 * than G.
 */
static bool isMagenta(const RGB<float> &p)
{
	return (p.r() - p.g() > 0.2f) && (p.b() - p.g() > 0.2f);
}

/*
 * Returns true if the pixel is approximately neutral (R ~= G ~= B).
 */
static bool isNeutral(const RGB<float> &p, float tol = 0.05f)
{
	return std::abs(p.r() - p.g()) < tol &&
	       std::abs(p.b() - p.g()) < tol;
}

class AwbCcmPipelineTest : public Test
{
protected:
	int run()
	{
		/*
		 * OV01A10 CCM at 4000K (from AIQB calibration binary).
		 * Row sums are exactly 1.0.
		 */
		const Matrix<float, 3, 3> ccm4000{ {
			 1.5414f, -0.4024f, -0.1390f,
			-0.3304f,  1.6352f, -0.3048f,
			-0.1237f, -0.6699f,  1.7936f,
		} };

		/*
		 * Typical AWB gains at ~4500K for OV01A10.
		 * R and B gains > 1 because the sensor is more sensitive to G.
		 */
		const RGB<float> awbGains({ 1.47f, 1.0f, 1.72f });

		/* --- Test 1: old approach produces magenta on bright pixels --- */
		{
			/*
			 * A fully saturated neutral raw pixel [1,1,1].
			 * With the old baked-matrix approach the combined row
			 * sums are [1.62, 0.63, 2.23], so R and B clip hard
			 * while G stays low -> magenta.
			 */
			RGB<float> brightNeutral({ 1.0f, 1.0f, 1.0f });
			RGB<float> out = applyBakedMatrix(ccm4000, awbGains, brightNeutral);

			/*
			 * We expect R > 1, G < 1, B > 1 (magenta before clamp).
			 * After hard clamp to [0,1]: R=1, G<1, B=1 -> magenta.
			 */
			ASSERT_TRUE(out.r() > 1.0f);
			ASSERT_TRUE(out.b() > 1.0f);
			ASSERT_TRUE(out.g() < 1.0f);

			RGB<float> clamped({ std::clamp(out.r(), 0.0f, 1.0f),
					     std::clamp(out.g(), 0.0f, 1.0f),
					     std::clamp(out.b(), 0.0f, 1.0f) });
			ASSERT_TRUE(isMagenta(clamped));
		}

		/* --- Test 2: new approach produces neutral on bright pixels --- */
		{
			RGB<float> brightNeutral({ 1.0f, 1.0f, 1.0f });
			RGB<float> out = applyClampedAwbThenCcm(ccm4000, awbGains, brightNeutral);

			/*
			 * After clamping AWB output to [0,1], the CCM receives
			 * [1,1,1] (all channels hit the ceiling equally for a
			 * neutral pixel).  CCM row sums = 1.0, so output = [1,1,1].
			 */
			ASSERT_NEAR(out.r(), 1.0f, kTol);
			ASSERT_NEAR(out.g(), 1.0f, kTol);
			ASSERT_NEAR(out.b(), 1.0f, kTol);
			ASSERT_TRUE(isNeutral(out));
		}

		/* --- Test 3: mid-tones are unaffected (no colour shift) --- */
		{
			/*
			 * A neutral grey at a level where AWB gains don't cause
			 * clipping: raw value scaled so that after AWB gains the
			 * result is [0.5, 0.5, 0.5].  With awbGains=[1.47,1.0,1.72]
			 * the raw pixel must be [0.34, 0.5, 0.29].
			 * Both approaches should give identical results here.
			 */
			float level = 0.5f;
			RGB<float> midGrey({ level / awbGains.r(),
					     level / awbGains.g(),
					     level / awbGains.b() });

			RGB<float> outBaked   = applyBakedMatrix(ccm4000, awbGains, midGrey);
			RGB<float> outClamped = applyClampedAwbThenCcm(ccm4000, awbGains, midGrey);

			/* Both approaches identical when no clipping occurs. */
			ASSERT_NEAR(outBaked.r(), outClamped.r(), kTol);
			ASSERT_NEAR(outBaked.g(), outClamped.g(), kTol);
			ASSERT_NEAR(outBaked.b(), outClamped.b(), kTol);

			/* Output should be neutral (CCM row sums = 1, input is balanced). */
			ASSERT_TRUE(isNeutral(outClamped));
		}

		/* --- Test 4: fix works across multiple CCMs --- */
		{
			/* OV01A10 CCM at 6500K */
			const Matrix<float, 3, 3> ccm6500{ {
				 1.8163f, -0.7062f, -0.1100f,
				-0.1640f,  1.5736f, -0.4096f,
				-0.0084f, -0.8294f,  1.8378f,
			} };

			/* Cooler AWB gains at 6500K (less R boost needed) */
			const RGB<float> awbGains6500({ 1.2f, 1.0f, 1.5f });

			RGB<float> brightNeutral({ 1.0f, 1.0f, 1.0f });

			/* Old approach: still magenta */
			RGB<float> outBaked = applyBakedMatrix(ccm6500, awbGains6500, brightNeutral);
			RGB<float> clampedBaked({ std::clamp(outBaked.r(), 0.0f, 1.0f),
						  std::clamp(outBaked.g(), 0.0f, 1.0f),
						  std::clamp(outBaked.b(), 0.0f, 1.0f) });
			ASSERT_TRUE(isMagenta(clampedBaked));

			/* New approach: neutral white */
			RGB<float> outClamped = applyClampedAwbThenCcm(ccm6500, awbGains6500, brightNeutral);
			ASSERT_TRUE(isNeutral(outClamped));
		}

		/* --- Test 5: black pixel is unaffected --- */
		{
			RGB<float> black({ 0.0f, 0.0f, 0.0f });
			RGB<float> out = applyClampedAwbThenCcm(ccm4000, awbGains, black);
			ASSERT_NEAR(out.r(), 0.0f, kTol);
			ASSERT_NEAR(out.g(), 0.0f, kTol);
			ASSERT_NEAR(out.b(), 0.0f, kTol);
		}

		return TestPass;
	}
};

TEST_REGISTER(AwbCcmPipelineTest)
