# Vision-language generation Function adapter

Public ROS Action: `uav_inference_interfaces/action/InferVisionLanguage` at
`/<vehicle>/inference/vision_language/infer`. Callers provide **model_name**, prompt,
positive timeout and maximum image-result age. The Function snapshots its configured
vehicle-local RGB camera and returns generic text with the actual acquisition header.
No navigation prompt, point/depth parser, geometric projection or flight execution
belongs here. Image encoding supports rgb8/bgr8 and row padding; no resizing or FOV change.

The worker reuses `aerohub_inference.app.create_app`; full-request busy survives a
cancelled HTTP caller. ROS cancellation returns promptly but keeps the Adapter's
busy slot until its blocking call exits. The HTTP worker separately retains busy
until the real cloud operation completes; a timeout is not proof the provider stopped.
There are no automatic retries or unbounded queues. Old-image and identity-mismatched
results are rejected, not re-stamped.

## Frames variant (multi-image requests, added 2026-11-18 for CityNav Tool-ification)

Public ROS Action: `uav_inference_interfaces/action/InferVisionLanguageFrames` at
`/<vehicle>/inference/vision_language_frames/infer`, served by
`uav_inference.vision_language_frames_node`. Unlike the single-image Function above,
**the caller supplies the frames explicitly** (ordered `image_labels` + images,
empty list = text-only request) plus `prompt`, optional `system_prompt`, positive
`timeout` and `max_result_age`. This exists because callers such as the CityNav
Navigation Tool hold upstream prompts verbatim and must send the exact batch they
act on; the provider message shape is the upstream one (per-frame
`"{label} image: "` text block + image, prompt block last, system message only
when `system_prompt` is non-empty). No navigation semantics (landmark/viewpoint
JSON, is_found parsing) live here — the Function returns raw model text. HTTP
surface: `/infer_frames` on the same uv-isolated worker
(`INFERENCE_MODEL_NAMES` registers the logical entries, e.g. `citynav`).

## Configuration

```bash
cp config/inference.cloud.example.yaml config/inference.cloud.local.yaml
```

The ignored file maps **logical model_name -> declared engines -> engine configuration**:

```yaml
models:
  spf:
    cloud:
      base_url: https://open.bigmodel.cn/api/paas/v4
      model: glm-4v-flash
      api_key: "your-api-key"
```

Callers choose `model_name="spf"`; `glm-4v-flash` is a private provider model,
not the public identity. URL defaults to the example endpoint if omitted; the
provider model defaults to the logical name. No omitted engine is supported.
With one engine it is selected automatically. Multiple declarations require:

```yaml
active_engines:
  spf: cloud
```

Selection never falls back on failure. A declaration is not an implementation:
this VLM adapter currently implements Cloud only. Explicitly selecting CUDA/Houmo
for SPF fails before loading an engine. Other models keep their own adapters.

Credentials can be inline `api_key` or an explicit per-entry `api_key_env` reference,
e.g. `api_key_env: OPENAI_API_KEY`. Optional `base_url_env` and `model_env` work the
same way. A referenced environment value overrides that entry's inline value;
unreferenced global OPENAI_* variables do not override unrelated models. Environment
variables alone cannot create a missing model or engine declaration.

No wizard, XDG directory or mandatory permission checks. Keep local config out of
Git/logs and restrict permissions where appropriate. Docker admins can read files
and container environments. Old files are not rewritten or deleted. Restart after
changes. `INFERENCE_MODEL_CONFIG` selects a different catalog if needed.

The deployment loads the logical `spf` entry. The generic VLM worker can explicitly
serve other logical entries of the same input/output contract through the internal
`INFERENCE_MODEL_NAMES` comma-separated deployment setting (default: `spf`);
unrelated entries such as YOPO/CUDA are left to their own model adapters.

## Prepare and run

`bash setup.sh` uses `uv sync --locked`, no weights or cloud calls, no CUDA requirement.
`UV_PROJECT_ENVIRONMENT` may point to a deployment-specific external workspace.
For ROS Humble, prepare using Python 3.10 in the same environment/path used to run it.
The SPF deployment prepares its worker in the stable compute image, outside the image.
The public setup selector registers logical ID `spf`; changing its remote provider
model does not change that ID or register another preparation entry.

```bash
# After preparation; use local YAML with api_key or an explicit api_key_env reference.
cd src/basic_function/inference/models/vision_language
INFERENCE_MODEL_CONFIG=/path/to/local.yaml \
  .venv/bin/python -m uvicorn serve:app --host 127.0.0.1 --port 8013
# In the separate ROS environment, source interfaces/adapter overlays:
python3 -m uav_inference.vision_language_node --ros-args -r __ns:=/drone0
```

Cloud protocol errors are sanitized; no provider response body, API key or authenticated
URL is returned. HTTPS is mandatory except explicit loopback HTTP in protocol tests.
OpenAI-compatible and Gemini REST adapters are implemented; real provider acceptance
requires separately recorded authorized API tests. No real cloud or flight acceptance
is implied by local HTTP/ROS tests. No GPU test applies to the cloud-only VLM adapter;
this does not waive GPU testing for actual CUDA models.
