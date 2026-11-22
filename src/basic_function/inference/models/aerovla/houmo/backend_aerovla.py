"""AeroVLA Houmo xh2 backend: vision + chunked prefill + greedy decode.

Verbatim port of the validated board runner (aerovla_infer/runner.py,
2026-10-18 round). KV cache handoff follows the official houmo modelzoo pattern
(laguna demo): prefill and decode share one WeightManager; decode cache inputs
are dummy tensors bound to prefill device-side cache pointers.

Differences from the original runner are layering only:
  - parse_action removed (the planner parses ``text``; migration ruling);
  - model_version comes from a pinned artifacts.json instead of hashing the
    13 GiB HMM files at every service start (full re-hash stays available via
    AEROVLA_VERIFY_SHA=1);
  - inputs/paths resolve from the deployment directory passed by the adapter.
"""

import hashlib
import json
import os
import sys
import threading
import time

import numpy as np


def _import_tcim():
    path = os.environ.get("TCIM_PYTHON_PATH", "/data/local/houmo/python")
    if path not in sys.path:
        sys.path.insert(0, path)
    import tcim_lite as tcim  # noqa: F401
    return tcim


def _set(model, key, arr):
    for i in range(model.get_num_inputs()):
        n = model.get_input_name(i)
        if key in n.lower() and "cache" not in n.lower():
            model.set_input(n, arr.astype(model.get_input_info(n).dtype))
            return True
    return False


def _logits(model):
    for i in range(model.get_num_outputs()):
        n = model.get_output_name(i)
        if "logit" in n.lower():
            return model.get_output(n).numpy()
    raise RuntimeError("no logits output")


class AeroVlaBackend:
    def __init__(self, infer_dir, device_id=0, verify_sha=False):
        self.tcim = _import_tcim()
        root = os.path.abspath(infer_dir)
        self.assets_dir = os.path.join(root, "assets")
        self.models_dir = os.path.join(root, "models")

        with open(os.path.join(self.assets_dir, "assets.json"), encoding="utf-8") as f:
            self.cfg = json.load(f)
        with open(os.path.join(self.assets_dir, "id2token.json"), encoding="utf-8") as f:
            self.id2token = {int(k): v for k, v in json.load(f).items()}
        self.emb = np.load(os.path.join(self.assets_dir, "token_embedding.npy"))

        hmm_files = ["aerovla_vision.hmm", "aerovla_prefill.hmm", "aerovla_decode.hmm"]
        artifacts = self._load_artifacts(root)
        pinned = artifacts.get("files", {})
        if verify_sha:
            for name in hmm_files:
                recorded = pinned.get(name, {}).get("sha256")
                digest = self._sha256_file(os.path.join(self.models_dir, name))
                if recorded is not None and recorded != digest:
                    raise RuntimeError("HMM checksum mismatch for %s" % name)

        self.vision = self.tcim.runtime.load(os.path.join(self.models_dir, hmm_files[0]))
        self.vision_in = self.vision.get_input_name(0)

        dm = self.tcim.runtime.DevManager([device_id], "Xh2HalBackend")
        wm = self.tcim.runtime.WeightManager(dm)
        self.prefill = self.tcim.runtime.load(
            os.path.join(self.models_dir, hmm_files[1]),
            option=self.tcim.runtime.Option(wm))
        self.cache_names = [
            self.prefill.get_input_name(i)
            for i in range(self.prefill.get_num_inputs())
            if "cache" in self.prefill.get_input_name(i).lower()]
        option = self.tcim.runtime.Option(wm)
        option.set_dummy_tensors(self.cache_names)
        self.decode = self.tcim.runtime.load(
            os.path.join(self.models_dir, hmm_files[2]), option=option)
        self.CHUNK = self.prefill.get_input_info("input_1").shape[1]
        self.bind_caches()

        self._lock = threading.Lock()
        self._closed = False
        try:
            tcim_version = str(self.prefill.get_model_version())
        except AttributeError:
            tcim_version = "unknown"
        version = "hmm=" + ",".join(
            "%s:%s" % (n, pinned[n]["sha256"][:16]) for n in hmm_files if n in pinned)
        self.metadata = {
            "model_id": "aerovla",
            "model_version": version or "unpinned",
            "device": "houmo:xh2:%d" % device_id,
            "tcim_model_version": tcim_version,
            "max_new_tokens": int(self.cfg["max_new_tokens"]),
            "num_bins": int(self.cfg["num_bins"]),
        }

    @staticmethod
    def _sha256_file(path):
        digest = hashlib.sha256()
        with open(path, "rb") as handle:
            for chunk in iter(lambda: handle.read(1 << 22), b""):
                digest.update(chunk)
        return digest.hexdigest()

    @staticmethod
    def _load_artifacts(root):
        # The pinned record ships inside the packed runtime; the infer_dir may
        # be the legacy deployment directory without it.
        for candidate in (os.path.join(root, "artifacts.json"),
                          os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                       "artifacts.json")):
            if os.path.isfile(candidate):
                with open(candidate, encoding="utf-8") as f:
                    return json.load(f)
        return {}

    def bind_caches(self):
        """Zero prefill caches; alias decode cache inputs to the same device buffers."""
        for n in self.cache_names:
            info = self.prefill.get_input_info(n)
            self.prefill.set_input(n, np.zeros(tuple(info.shape), dtype=np.dtype(info.dtype)))
            self.decode.set_input(n, self.prefill.get_dev_input(n))

    def _vision_embed(self, pixel_values):
        m = self.vision
        m.set_input(self.vision_in, pixel_values.astype(m.get_input_info(self.vision_in).dtype))
        m.run()
        m.sync()
        return m.get_output(m.get_output_name(0)).numpy().astype(np.float16)

    def build_inputs_embeds(self, input_ids, img_emb):
        # Prismatic convention: patch embeddings are inserted right after BOS.
        ids = list(input_ids)
        return np.concatenate([self.emb[ids[:1]], img_emb[0], self.emb[ids[1:]]], axis=0)

    def generate(self, input_ids, pixel_values):
        """Greedy autoregressive decode, verbatim runner semantics."""
        with self._lock:
            if self._closed:
                raise RuntimeError("backend is closed")
            started = time.perf_counter()
            img_emb = self._vision_embed(pixel_values)
            seq = self.build_inputs_embeds(input_ids, img_emb)
            total = seq.shape[0]
            self.bind_caches()

            logits = None
            n_chunks = max(1, (total + self.CHUNK - 1) // self.CHUNK)
            for ci in range(n_chunks):
                part = seq[ci * self.CHUNK:(ci + 1) * self.CHUNK]
                valid = part.shape[0]
                if valid < self.CHUNK:
                    part = np.concatenate(
                        [part, np.repeat(self.emb[32000][None, :], self.CHUNK - valid, 0)], 0)
                pos = np.concatenate([
                    np.arange(ci * self.CHUNK, ci * self.CHUNK + valid),
                    np.ones(self.CHUNK - valid, np.int64)]).reshape(1, self.CHUNK)
                _set(self.prefill, "input_1", part[None])
                _set(self.prefill, "current_length", np.array([valid], np.int32))
                _set(self.prefill, "valid", np.array([ci * self.CHUNK], np.int32))
                _set(self.prefill, "position", pos)
                self.prefill.run()
                self.prefill.sync()
                logits = _logits(self.prefill)

            token = int(np.argmax(logits[0, 0]))
            eos = self.cfg["eos_token_id"]
            gen = [] if token == eos else [token]
            for i in range(self.cfg["max_new_tokens"] - 1):
                p = total + i
                _set(self.decode, "input_1", self.emb[token][None, None, :])
                _set(self.decode, "current_length", np.array([1], np.int32))
                _set(self.decode, "valid", np.array([p], np.int32))
                _set(self.decode, "position", np.array([[p]], np.int64))
                self.decode.run()
                self.decode.sync()
                token = int(np.argmax(_logits(self.decode)[0, 0]))
                if token == eos:
                    break
                gen.append(token)
            self.last_generate_s = time.perf_counter() - started
            return gen

    def warmup(self):
        zeros = np.zeros(tuple(self.vision.get_input_info(self.vision_in).shape),
                         dtype=np.float32)
        self._vision_embed(zeros)

    def close(self):
        with self._lock:
            if not self._closed:
                for module in (self.vision, self.prefill, self.decode):
                    try:
                        module.sync()
                    except Exception:  # noqa: BLE001 - shutdown must not raise
                        pass
                self._closed = True
