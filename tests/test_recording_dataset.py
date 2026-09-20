"""Exercise the thin adapter without adding torch/DP to robot dependencies."""

import importlib
import importlib.util
from pathlib import Path
import pickle
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np


class FakeBaseImageDataset:
    pass


class FakeReplayBuffer:
    copied_keys = None

    @classmethod
    def copy_from_path(cls, path, *, store, keys):
        import zarr

        source = zarr.open_group(path, mode="r")
        result = cls()
        result.values = {key: source[f"data/{key}"][:] for key in keys}
        result.episode_ends = source["meta/episode_ends"][:]
        result.n_episodes = len(result.episode_ends)
        cls.copied_keys = list(keys)
        return result

    def __getitem__(self, key):
        return self.values[key]


class FakeSequenceSampler:
    def __init__(self, replay_buffer, sequence_length, pad_before, pad_after, keys, episode_mask):
        self.buffer, self.keys, self.length = replay_buffer, keys, sequence_length
        self.mask = episode_mask
        self.windows = []
        start = 0
        for end, include in zip(replay_buffer.episode_ends, episode_mask):
            if include:
                for i in range(start - pad_before, end - sequence_length + pad_after + 1):
                    self.windows.append(np.clip(np.arange(i, i + sequence_length), start, end - 1))
            start = end

    def __len__(self):
        return len(self.windows)

    def sample_sequence(self, index):
        return {key: self.buffer[key][self.windows[index]] for key in self.keys}


class FakeNormalizer(dict):
    def fit(self, *, data, last_n_dims, **kwargs):
        self.update({key: {"shape": value.shape, "last_n_dims": last_n_dims} for key, value in data.items()})


def fake_modules():
    names = ("diffusion_policy", "diffusion_policy.dataset", "diffusion_policy.dataset.base_dataset",
             "diffusion_policy.common", "diffusion_policy.common.replay_buffer", "diffusion_policy.common.sampler",
             "diffusion_policy.model", "diffusion_policy.model.common", "diffusion_policy.model.common.normalizer",
             "diffusion_policy.common.normalize_util", "torch")
    modules = {name: types.ModuleType(name) for name in names}
    modules["diffusion_policy.dataset.base_dataset"].BaseImageDataset = FakeBaseImageDataset
    modules["diffusion_policy.common.replay_buffer"].ReplayBuffer = FakeReplayBuffer
    modules["diffusion_policy.common.sampler"].SequenceSampler = FakeSequenceSampler

    def val_mask(n_episodes, val_ratio, seed):
        mask = np.zeros(n_episodes, bool)
        if val_ratio and n_episodes > 1:
            mask[-1] = True
        return mask

    modules["diffusion_policy.common.sampler"].get_val_mask = val_mask
    modules["diffusion_policy.common.sampler"].downsample_mask = lambda mask, max_n, seed: mask
    modules["diffusion_policy.model.common.normalizer"].LinearNormalizer = FakeNormalizer
    modules["diffusion_policy.common.normalize_util"].get_image_range_normalizer = lambda: "rgb-range"
    modules["torch"].from_numpy = lambda array: array
    return modules


@unittest.skipUnless(importlib.util.find_spec("zarr"), "recording dependencies are not installed")
class RecordingDatasetTests(unittest.TestCase):
    def setUp(self):
        import zarr

        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "dataset.zarr"
        self.root = zarr.open_group(str(self.path), mode="w")
        data = self.root.create_group("data")
        data.array("camera_0", np.full((9, 4, 6, 3), 255, np.uint8))
        data.array("camera_1", np.full((9, 4, 6, 3), 128, np.uint8))
        data.array("camera_0_depth", np.ones((9, 4, 6), np.uint16))
        data.array("robot_joint", np.arange(9 * 14).reshape(9, 14).astype(np.float32))
        data.array("wrench", np.zeros((9, 12), np.float32))
        data.array("action", np.arange(9 * 52).reshape(9, 52).astype(np.float32))
        meta = self.root.create_group("meta")
        meta.array("episode_ends", np.array([3, 6, 9], dtype=np.int64))
        meta.attrs["segments"] = [{"source_episode": "a"}, {"source_episode": "a"}, {"source_episode": "b"}]
        self.shape_meta = {"obs": {"camera_0": {"type": "rgb", "shape": [3, 4, 6]},
                                   "robot_joint": {"type": "low_dim", "shape": [14]}},
                           "action": {"shape": [52]}}
        self.module = importlib.import_module("bimanual_teleop.recording.dataset")
        self.module.__dict__.pop("BimanualImageDataset", None)
        self.addCleanup(lambda: self.module.__dict__.pop("BimanualImageDataset", None))
        self.patch = patch.dict(sys.modules, fake_modules())
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def dataset(self, **kwargs):
        return self.module.BimanualImageDataset(self.path, self.shape_meta, horizon=2, n_obs_steps=1, **kwargs)

    def test_only_selected_observations_are_loaded_and_images_are_chw(self):
        dataset = self.dataset()
        self.assertIsInstance(dataset, FakeBaseImageDataset)
        self.assertEqual(FakeReplayBuffer.copied_keys, ["camera_0", "robot_joint", "action"])
        sample = dataset[0]
        self.assertEqual(set(sample["obs"]), {"camera_0", "robot_joint"})
        self.assertEqual(sample["obs"]["camera_0"].shape, (1, 3, 4, 6))
        self.assertEqual(sample["obs"]["camera_0"].dtype, np.float32)
        np.testing.assert_array_equal(sample["obs"]["camera_0"], 1.)
        self.assertEqual(sample["action"].shape, (2, 52))
        self.assertEqual(dataset.get_all_actions().shape, (9, 52))
        self.assertEqual(set(dataset.get_normalizer()), {"camera_0", "robot_joint", "action"})

    def test_validation_keeps_fragments_of_one_demonstration_together(self):
        dataset = self.dataset(val_ratio=.5)
        np.testing.assert_array_equal(dataset.train_mask, [True, True, False])
        np.testing.assert_array_equal(dataset.val_mask, [False, False, True])
        validation = dataset.get_validation_dataset()
        self.assertEqual(len(dataset), 4)
        self.assertEqual(len(validation), 2)
        np.testing.assert_array_equal(validation[0]["action"], self.root["data/action"][6:8])
        # Last sample of the first fragment cannot reach the next fragment.
        np.testing.assert_array_equal(dataset[1]["action"], self.root["data/action"][1:3])

    def test_shape_mismatch_is_rejected_before_training(self):
        self.shape_meta["action"]["shape"] = [54]
        with self.assertRaisesRegex(ValueError, "action"):
            self.dataset()
        self.shape_meta["action"]["shape"] = [52]
        self.shape_meta["obs"]["camera_0"]["shape"] = [3, 6, 4]
        with self.assertRaisesRegex(ValueError, "stored shape"):
            self.dataset()

    def test_depth_is_not_accepted_as_rgb(self):
        self.shape_meta["obs"] = {"camera_0_depth": {"type": "rgb", "shape": [3, 4, 6]}}
        with self.assertRaisesRegex(ValueError, "uint8"):
            self.dataset()

    def test_public_class_can_be_resolved_during_unpickling(self):
        dataset_type = self.module.BimanualImageDataset
        payload = pickle.dumps(dataset_type)
        self.module.__dict__.pop("BimanualImageDataset")
        restored = pickle.loads(payload)
        self.assertTrue(issubclass(restored, FakeBaseImageDataset))
        self.assertEqual(restored.__name__, "BimanualImageDataset")


if __name__ == "__main__":
    unittest.main()
