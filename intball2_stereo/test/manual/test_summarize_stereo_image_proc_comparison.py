"""Offline timing populations retain conversion failures and invalid clocks."""

import unittest

from summarize_stereo_image_proc_comparison import distribution, summarize_rows


class SipTimingSummaryTests(unittest.TestCase):
    def test_failed_conversion_does_not_discard_valid_native_receipt_age(self):
        row = {"native_timing_valid": "True", "native_timing_invalid_reason": "",
               "timing_valid": "False", "timing_invalid_reason": "missing_info",
               "conversion_success": "False", "conversion_rejection_reason": "missing_info",
               "source_to_disparity_receive_sim_s": "0.4"}
        result = summarize_rows([row])
        self.assertEqual(result["source_to_disparity_receive_sim_s"]["p50_s"], .4)
        self.assertEqual(result["source_to_adapter_publish_sim_s"]["count"], 0)
        self.assertEqual(result["timing_invalid_reasons"], {"missing_info": 1})

    def test_invalid_clock_and_negative_numeric_samples_keep_explicit_denominators(self):
        invalid = {"native_timing_valid": "False", "native_timing_invalid_reason": "rewound",
                   "timing_valid": "False", "timing_invalid_reason": "rewound",
                   "conversion_success": "True", "conversion_rejection_reason": "",
                   "source_to_disparity_receive_sim_s": "0.9"}
        negative = dict(invalid, native_timing_valid="True", native_timing_invalid_reason="",
                        source_to_disparity_receive_sim_s="-0.1")
        result = summarize_rows([invalid, negative])
        self.assertEqual(result["row_count"], 2)
        self.assertEqual(result["native_timing_valid_count"], 1)
        self.assertEqual(result["native_timing_invalid_reasons"], {"rewound": 1})
        self.assertEqual(result["source_to_disparity_receive_sim_s"]["count"], 0)
        self.assertEqual(result["source_to_disparity_receive_sim_s"]["excluded_missing_negative_nonfinite_count"], 1)
        self.assertIsNone(distribution([])["p95_s"])
        self.assertAlmostEqual(distribution([.4, .8])["p95_s"], .78)


if __name__ == "__main__":
    unittest.main()
