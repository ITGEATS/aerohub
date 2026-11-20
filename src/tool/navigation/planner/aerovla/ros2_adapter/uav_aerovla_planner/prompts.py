"""AeroVLA prompt assembly + tokenization (host/ROS-free importable).

Tokenization convention (verified 8/8 EXACT against the board table by
``log/aerovla/stage1/make_prompts.py`` L22-24, the fidelity source for the
prompt string):

- BOS=1 is added automatically by the openvla-7b tokenizer (default
  ``add_special_tokens=True``); no manual special tokens.
- ``<image>`` stays a PLAIN TEXT token sequence (it is not replaced by a
  vision placeholder here; the board worker owns the mosaic embedding).
- The prompt ends with the literal ``"Action: "`` (trailing space).

The template literal below is AST-locked by
``tests/test_prompts_compat.py``: any edit that changes the string bytes
fails the test, so the prompt cannot silently drift from the locked
upstream contract.
"""

import os

# AST-LOCKED literal. Byte-identical to make_prompts.py L23 /
# aerovla_wrapper_ui.py L146-150:
#   "<image>\nFly {hint}and find the target. {obj_desc}\nAction: "
# NOTE the missing space between {hint} and "and": the 7 hint literals from
# policy.semantic_hint each end with a trailing space, and the empty hint
# ("") yields "Fly and find the target." exactly as upstream.
PROMPT_TEMPLATE = "<image>\nFly {hint}and find the target. {obj_desc}\nAction: "

# make_prompts.py: the openvla-7b tokenizer prepends BOS id 1.
BOS_TOKEN_ID = 1

# Tokenizer location. On the board the openvla-7b base sits next to the
# inference workspace (/data/local/workspace/aerovla_infer); override with
# AEROVLA_TOKENIZER_PATH for host or relocated layouts.
DEFAULT_TOKENIZER_PATH = "/data/local/workspace/openvla-7b"
TOKENIZER_PATH_ENV = "AEROVLA_TOKENIZER_PATH"


def resolve_tokenizer_path(path=None):
    """Env-configurable tokenizer path without loading anything."""
    if path:
        return path
    return os.environ.get(TOKENIZER_PATH_ENV, DEFAULT_TOKENIZER_PATH)


_tokenizers = {}


def load_tokenizer(path=None):
    """Lazy transformers import: host unit tests never touch this path."""
    resolved = resolve_tokenizer_path(path)
    if resolved not in _tokenizers:
        from transformers import AutoTokenizer
        _tokenizers[resolved] = AutoTokenizer.from_pretrained(
            resolved, trust_remote_code=True)
    return _tokenizers[resolved]


def build_prompt_text(hint, obj_desc):
    return PROMPT_TEMPLATE.format(hint=hint, obj_desc=obj_desc)


def tokenize(text, tokenizer=None):
    """Verbatim convention of make_prompts.py L24: ``tok(text)["input_ids"]``
    with default special tokens (BOS=1 first id)."""
    tok = tokenizer if tokenizer is not None else load_tokenizer()
    return [int(v) for v in tok(text)["input_ids"]]


def prompt_token_ids(hint, obj_desc, tokenizer=None):
    return tokenize(build_prompt_text(hint, obj_desc), tokenizer=tokenizer)
