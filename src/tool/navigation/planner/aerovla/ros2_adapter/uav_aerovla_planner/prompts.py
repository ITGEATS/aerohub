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

import json
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
# Pre-tokenized prompt table (preferred on the board: the openvla-7b
# tokenizer is not installed there). Upstream board contract: the assets
# prompt_ids.json / prompt_ids_blocks.json tables are keyed by the 7 hint
# literals and consumed verbatim by the deployed orchestrator; the ids in
# log/aerovla/stage1/prompt_ids_blocks.json were generated with the
# convention verified 8/8 EXACT and then ran a real episode (SUCCESS).
PROMPT_TABLE_ENV = "AEROVLA_PROMPT_TABLE"


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


def resolve_prompt_table_path(path=None):
    """Env-configurable prompt table path without touching the filesystem."""
    if path:
        return path
    return os.environ.get(PROMPT_TABLE_ENV)


def load_prompt_table(path=None):
    """Load a pre-tokenized prompt table.

    Two accepted formats:

    - wrapped (produced by the deployment tooling): ``{"obj_desc": <str>,
      "prompts": {hint: [ids...]}}`` -- ``obj_desc`` is pinned so a node
      configured with a different description fails LOUDLY instead of
      silently feeding mismatched ids to the policy;
    - bare upstream board format ``{hint: [ids...]}`` (e.g. the assets
      prompt_ids.json) -- ``obj_desc`` is then unknown (``None``) and not
      checked.
    """
    resolved = resolve_prompt_table_path(path)
    if not resolved:
        raise ValueError("prompt table path is empty; set the prompt_table "
                         "parameter or " + PROMPT_TABLE_ENV)
    with open(resolved, encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or not data:
        raise ValueError("prompt table must be a non-empty object: " + str(resolved))
    if "prompts" in data:
        prompts_map = data["prompts"]
        obj_desc = data.get("obj_desc")
        if obj_desc is not None and not isinstance(obj_desc, str):
            raise ValueError("prompt table obj_desc must be a string")
    else:
        prompts_map, obj_desc = data, None
    if not isinstance(prompts_map, dict) or not prompts_map:
        raise ValueError("prompt table has no hint entries: " + str(resolved))
    table = {}
    for hint, ids in prompts_map.items():
        if not isinstance(ids, list) or not ids or \
                any(type(v) is not int or v < 0 for v in ids):
            raise ValueError("prompt table entry %r must be nonnegative int ids" % hint)
        table[str(hint)] = ids
    return {"obj_desc": obj_desc, "prompts": table}


def table_prompt_ids(table, hint, obj_desc):
    """Lookup ids for a hint; refuses a table pinned to another obj_desc."""
    if table["obj_desc"] is not None and table["obj_desc"] != obj_desc:
        raise ValueError("prompt table was generated for obj_desc %r, node "
                         "is configured with %r" % (table["obj_desc"], obj_desc))
    try:
        return list(table["prompts"][hint])
    except KeyError:
        raise KeyError("prompt table has no entry for hint %r; available: %s"
                       % (hint, sorted(table["prompts"]))) from None
