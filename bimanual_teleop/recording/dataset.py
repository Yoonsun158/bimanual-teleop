"""Thin, optional training adapter for the official diffusion_policy package.

Hydra target: bimanual_teleop.recording.dataset.BimanualImageDataset.
Only shape_meta['obs'] keys enter training. RGB is converted HWC uint8 ->
CHW float32 / 255; optional recorded depth is not selected automatically.
Importing this module does not import torch or any training dependencies.
"""

from __future__ import annotations

import copy
from pathlib import Path


class _DatasetMethods:
    def __init__(self, dataset_path, shape_meta, horizon=16, pad_before=0, pad_after=0,
                 n_obs_steps=None, seed=42, val_ratio=0., max_train_episodes=None):
        import numpy as np
        import zarr
        from diffusion_policy.common.replay_buffer import ReplayBuffer
        from diffusion_policy.common.sampler import downsample_mask, get_val_mask

        if not isinstance(horizon, int) or horizon < 1:
            raise ValueError("horizon must be a positive integer")
        n_obs_steps = horizon if n_obs_steps is None else n_obs_steps
        if not isinstance(n_obs_steps, int) or not 1 <= n_obs_steps <= horizon:
            raise ValueError("n_obs_steps must be between 1 and horizon")
        if not 0 <= val_ratio < 1:
            raise ValueError("val_ratio must be in [0,1)")
        if not 0 <= pad_before < horizon or not 0 <= pad_after < horizon:
            raise ValueError("padding must be nonnegative and smaller than horizon")
        if max_train_episodes is not None and max_train_episodes < 1:
            raise ValueError("max_train_episodes must be positive")
        path = Path(dataset_path).expanduser()
        root = zarr.open_group(str(path), mode="r")
        obs_meta = shape_meta["obs"]
        if not obs_meta or "action" in obs_meta:
            raise ValueError("shape_meta.obs must select observations and must not include action")
        self.rgb_keys, self.lowdim_keys = [], []
        for key, specification in obs_meta.items():
            kind = specification.get("type", "low_dim")
            shape = tuple(specification["shape"])
            if key not in root["data"]:
                raise ValueError(f"Observation is missing from dataset: {key}")
            array = root["data"][key]
            if kind == "rgb":
                if len(shape) != 3 or shape[0] != 3 or array.dtype != np.dtype("uint8"):
                    raise ValueError(f"{key}: rgb requires shape [3,H,W] and uint8 source images")
                expected = shape[1:] + (shape[0],)
                self.rgb_keys.append(key)
            elif kind == "low_dim":
                if len(shape) != 1:
                    raise ValueError(f"{key}: low_dim must be a vector; depth needs a dedicated encoder")
                expected = shape
                self.lowdim_keys.append(key)
            else:
                raise ValueError(f"Unsupported observation type for {key}: {kind}")
            if array.shape[1:] != expected:
                raise ValueError(f"{key}: stored shape {array.shape[1:]} does not match shape_meta {shape}")
        action_shape = tuple(shape_meta["action"]["shape"])
        if len(action_shape) != 1 or root["data/action"].shape[1:] != action_shape:
            raise ValueError("shape_meta.action does not match the converted action dimension")
        self.keys = list(obs_meta) + ["action"]
        self.replay_buffer = ReplayBuffer.copy_from_path(str(path), store=zarr.MemoryStore(), keys=self.keys)
        self.horizon, self.n_obs_steps = horizon, n_obs_steps
        self.pad_before, self.pad_after = pad_before, pad_after

        # Splits remain independent when gaps split one demonstration into
        # multiple DP episodes: all fragments stay in the same partition.
        n_episodes = self.replay_buffer.n_episodes
        segments = root["meta"].attrs.get("segments", [])
        if segments and len(segments) != n_episodes:
            raise ValueError("Segment metadata does not match episode_ends")
        sources = [row["source_episode"] for row in segments] if segments else list(range(n_episodes))
        source_ids = {source: i for i, source in enumerate(dict.fromkeys(sources))}
        indices = np.asarray([source_ids[source] for source in sources], dtype=np.int64)
        source_val = get_val_mask(n_episodes=len(source_ids), val_ratio=val_ratio, seed=seed)
        source_train = downsample_mask(~source_val, max_n=max_train_episodes, seed=seed)
        self.val_mask = source_val[indices]
        self.train_mask = source_train[indices]
        self.sampler = self._make_sampler(self.train_mask)

    def _make_sampler(self, mask):
        from diffusion_policy.common.sampler import SequenceSampler

        return SequenceSampler(replay_buffer=self.replay_buffer, sequence_length=self.horizon,
                               pad_before=self.pad_before, pad_after=self.pad_after,
                               keys=self.keys, episode_mask=mask)

    def get_validation_dataset(self):
        result = copy.copy(self)
        result.sampler = self._make_sampler(self.val_mask)
        return result

    def get_normalizer(self, **kwargs):
        from diffusion_policy.model.common.normalizer import LinearNormalizer
        from diffusion_policy.common.normalize_util import get_image_range_normalizer

        normalizer = LinearNormalizer()
        fields = {key: self.replay_buffer[key][:] for key in self.lowdim_keys + ["action"]}
        normalizer.fit(data=fields, last_n_dims=1, **kwargs)
        for key in self.rgb_keys:
            normalizer[key] = get_image_range_normalizer()
        return normalizer

    def get_all_actions(self):
        import numpy as np
        import torch

        return torch.from_numpy(np.asarray(self.replay_buffer["action"][:], dtype=np.float32))

    def __len__(self):
        return len(self.sampler)

    def __getitem__(self, index):
        import numpy as np
        import torch

        sample = self.sampler.sample_sequence(index)
        obs = {}
        for key in self.rgb_keys:
            image = np.moveaxis(sample[key][:self.n_obs_steps], -1, 1).astype(np.float32) / 255.
            obs[key] = torch.from_numpy(image)
        for key in self.lowdim_keys:
            obs[key] = torch.from_numpy(sample[key][:self.n_obs_steps].astype(np.float32))
        return {"obs": obs, "action": torch.from_numpy(sample["action"].astype(np.float32))}


def __getattr__(name):
    if name != "BimanualImageDataset":
        raise AttributeError(name)
    try:
        from diffusion_policy.dataset.base_dataset import BaseImageDataset
    except ImportError as error:
        raise ImportError("BimanualImageDataset requires the official diffusion_policy training environment") from error
    # Delayed class creation preserves the true DP base class and a module-level
    # name, including when a spawned DataLoader worker unpickles the dataset.
    dataset_type = type(name, (_DatasetMethods, BaseImageDataset), {"__module__": __name__})
    globals()[name] = dataset_type
    return dataset_type
