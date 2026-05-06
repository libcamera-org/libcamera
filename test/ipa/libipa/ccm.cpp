/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2024-2026, Red Hat Inc.
 *
 * CCM matrix row-sum validation tests
 *
 * Each row of a colour correction matrix must sum to 1.0 (luminance
 * preservation).  This test verifies that property for inline matrix data
 * and for matrices parsed from a YAML CCM table.
 */

#include "../../../src/ipa/libipa/interpolator.h"

#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>

#include "libcamera/base/file.h"
#include "libcamera/internal/matrix.h"
#include "libcamera/internal/yaml_parser.h"

#include "test.h"

using namespace std;
using namespace libcamera;
using namespace ipa;

/* Tolerance for floating-point row-sum comparison.
 * CCM values in tuning files are typically given to 4 decimal places,
 * which can introduce up to ~0.5e-3 rounding error per row. */
static constexpr float kRowSumTolerance = 5e-4f;

#define ASSERT_TRUE(cond)                             \
	do {                                          \
		if (!(cond)) {                        \
			cerr << "FAIL: " #cond "\n";  \
			return TestFail;              \
		}                                     \
	} while (0)

static bool allRowsSumToOne(const Matrix<float, 3, 3> &m)
{
	for (unsigned int row = 0; row < 3; row++) {
		float sum = 0.0f;
		for (unsigned int col = 0; col < 3; col++)
			sum += m[row][col];
		if (std::abs(sum - 1.0f) > kRowSumTolerance)
			return false;
	}
	return true;
}

class CcmRowSumTest : public Test
{
protected:
	bool writeTempYaml(const std::string &content, std::string &filename)
	{
		filename = "/tmp/libcamera.ccm.test.XXXXXX";
		int fd = mkstemp(&filename.front());
		if (fd == -1)
			return false;
		ssize_t ret = write(fd, content.c_str(), content.size());
		close(fd);
		return ret == static_cast<ssize_t>(content.size());
	}

	std::unique_ptr<ValueNode> parseYaml(const std::string &content)
	{
		std::string filename;
		if (!writeTempYaml(content, filename))
			return nullptr;

		File file{ filename };
		if (!file.open(File::OpenModeFlag::ReadOnly))
			return nullptr;

		auto root = YamlParser::parse(file);
		unlink(filename.c_str());
		return root;
	}

	int run()
	{
		/* --- 1. Known-good identity matrix --- */
		Matrix<float, 3, 3> identity{ { 1, 0, 0,
						0, 1, 0,
						0, 0, 1 } };
		ASSERT_TRUE(allRowsSumToOne(identity));

		/* --- 2. Known-bad matrix (rows do not sum to 1) --- */
		Matrix<float, 3, 3> bad{ { 2, 0, 0,
					   0, 1, 0,
					   0, 0, 1 } };
		ASSERT_TRUE(!allRowsSumToOne(bad));

		/* --- 3. Typical calibrated CCM (D65, OV01A10) --- */
		Matrix<float, 3, 3> d65{ {  1.8163f, -0.7062f, -0.1100f,
					   -0.1640f,  1.5736f, -0.4096f,
					   -0.0084f, -0.8294f,  1.8378f } };
		ASSERT_TRUE(allRowsSumToOne(d65));

		/* --- 4. Parse a valid CCM table from YAML and validate --- */
		const std::string validYaml =
			"- ct: 2856\n"
			"  ccm: [  1.1248,  0.2210, -0.3458,\n"
			"         -0.4616,  1.7736, -0.3120,\n"
			"         -0.4342, -0.9348,  2.3690 ]\n"
			"- ct: 6500\n"
			"  ccm: [  1.8163, -0.7062, -0.1100,\n"
			"         -0.1640,  1.5736, -0.4096,\n"
			"         -0.0084, -0.8294,  1.8378 ]\n"
			"- ct: 7500\n"
			"  ccm: [  1.8953, -0.7980, -0.0973,\n"
			"         -0.1539,  1.6001, -0.4462,\n"
			"         -0.0101, -0.7800,  1.7902 ]\n";

		auto root = parseYaml(validYaml);
		ASSERT_TRUE(root);

		Interpolator<Matrix<float, 3, 3>> interp;
		ASSERT_TRUE(interp.readYaml(*root, "ct", "ccm") == 0);
		ASSERT_TRUE(interp.data().size() == 3);

		for (const auto &[ct, m] : interp.data()) {
			if (!allRowsSumToOne(m)) {
				cerr << "CCM at ct=" << ct
				     << " has a row that does not sum to 1.0\n";
				return TestFail;
			}
		}

		/* --- 5. Detect a bad entry in YAML --- */
		const std::string badYaml =
			"- ct: 5000\n"
			"  ccm: [  2.0000, -0.7062, -0.1100,\n"
			"         -0.1640,  1.5736, -0.4096,\n"
			"         -0.0084, -0.8294,  1.8378 ]\n";

		auto badRoot = parseYaml(badYaml);
		ASSERT_TRUE(badRoot);

		Interpolator<Matrix<float, 3, 3>> badInterp;
		ASSERT_TRUE(badInterp.readYaml(*badRoot, "ct", "ccm") == 0);

		for (const auto &[ct, m] : badInterp.data()) {
			if (allRowsSumToOne(m)) {
				cerr << "Expected bad CCM at ct=" << ct
				     << " to fail row-sum check, but it passed\n";
				return TestFail;
			}
		}

		return TestPass;
	}
};

TEST_REGISTER(CcmRowSumTest)
