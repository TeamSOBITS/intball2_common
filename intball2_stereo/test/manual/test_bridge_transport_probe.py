import struct
import unittest

from types import SimpleNamespace

from bridge_transport_probe import clock_stamp, distribution, join_rows, serialized_header


class BridgeTransportProbeTest(unittest.TestCase):
    def test_header_exact_integer_precision_and_pixel_bytes_ignored(self):
        payload = struct.pack('<IIII', 2, 46000, 999999999, 7) + b'optical' + b'pixels'
        self.assertEqual(serialized_header(payload), (2, 46000999999999, 'optical'))
        for invalid in (b'', struct.pack('<IIII', 0, 1, 1000000000, 0),
                        struct.pack('<IIII', 0, 1, 0, 2) + b'a'):
            with self.assertRaises(ValueError):
                serialized_header(invalid)
        self.assertEqual(clock_stamp(SimpleNamespace(clock=SimpleNamespace(
            to_nsec=lambda: 46000000000001))), 46000000000001)
        self.assertEqual(clock_stamp(SimpleNamespace(clock=SimpleNamespace(to_nsec=lambda: 0))), 0)
        with self.assertRaises(ValueError):
            clock_stamp(SimpleNamespace(clock=SimpleNamespace(to_nsec=lambda: -1)))

    def test_exact_common_population_duplicates_missing_and_negative_proxy(self):
        def probe(stamp, arrival, valid='1'):
            return dict(stream='left', source_stamp_ns=str(stamp), observer_sim_ns=str(arrival), clock_valid=valid)
        def node(stamp, arrival):
            return dict(left_image_stamp_ns=str(stamp), left_callback_sim_s=str(arrival / 1e9))
        report = join_rows([probe(10, 20), probe(20, 30), probe(20, 99), probe(30, 40),
                            probe(40, 50), probe(50, 60, '0')],
                           [node(20, 29), node(25, 35), node(40, 70), node(60, 80)])['streams']['left']
        self.assertEqual(report['paired_count'], 2)
        self.assertEqual(report['probe_duplicate_count'], 1)
        self.assertEqual(report['probe_invalid_clock_count'], 1)
        self.assertEqual(report['probe_only_common_window_count'], 1)
        self.assertEqual(report['ros2_only_common_window_count'], 1)
        self.assertEqual(report['ros1_observer_to_ros2_callback_sim_proxy']['negative_count'], 1)
        self.assertEqual(report['ros1_observer_to_ros2_callback_sim_proxy']['count'], 1)
        self.assertEqual(report['common_source_window_start_ns'], 20)
        self.assertEqual(report['common_source_window_end_ns'], 40)

    def test_empty_and_bad_numeric_population_is_explicit(self):
        self.assertEqual(join_rows([], [])['streams']['left']['paired_count'], 0)
        result = distribution([-1, float('nan'), 1, 3])
        self.assertEqual(result['p50_s'], 2)
        self.assertEqual(result['negative_count'], 1)
        self.assertEqual(result['nonfinite_count'], 1)


if __name__ == '__main__':
    unittest.main()
