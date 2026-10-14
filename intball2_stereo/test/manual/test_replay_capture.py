from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from replay_capture import CONFIG_KEYS, frame_manifest, output_directory, parameters


class ReplayCaptureTests(unittest.TestCase):
    def test_nested_live_dump_and_flat_startup_config_agree(self):
        root = Path(__file__).resolve().parents[2]
        startup = parameters(root / 'config/stereo_params.yaml')
        import yaml
        nested = {}
        for key in CONFIG_KEYS:
            group, name = key.split('.')
            nested.setdefault(group, {})[name] = startup[key]
        with TemporaryDirectory() as temporary:
            dump = Path(temporary) / 'live.yaml'
            dump.write_text(yaml.safe_dump({'/stereo_node': {'ros__parameters': nested}}))
            actual = parameters(dump)
        self.assertEqual({key: startup[key] for key in CONFIG_KEYS}, actual)

    def test_outputs_cannot_be_written_into_checkout(self):
        checkout = Path(__file__).resolve().parents[2]
        with self.assertRaises(ValueError):
            output_directory(checkout / 'test/manual/results')
        with TemporaryDirectory() as temporary:
            self.assertEqual(output_directory(Path(temporary)), Path(temporary).resolve())
            alias = Path(temporary) / 'checkout_alias'
            alias.symlink_to(checkout, target_is_directory=True)
            with self.assertRaises(ValueError):
                output_directory(alias / 'results')

    def test_wrong_raw_stamp_rejected_before_any_byte_file_is_written(self):
        metadata = {'images': {'camera': {'stamp_ns': 42}},
                    'camera_info': {'camera_info': {}},
                    'source_stamps': {'left_image_stamp_ns': 43}}
        with TemporaryDirectory() as temporary:
            directory = Path(temporary)
            with self.assertRaises(ValueError):
                frame_manifest(directory, {}, metadata, 'camera', 'camera_info', 'left_image_stamp_ns')
            self.assertEqual(list(directory.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
