"""Cloud execution plugin. No robotics, navigation prompts or point parsing."""
import base64
import json
import math
import os
import socket
import threading
import urllib.error
import urllib.parse
import urllib.request


class CloudError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def build_openai_payload(prompt, jpeg_b64, model, temperature, max_tokens):
    return dict(model=model, messages=[dict(role='user', content=[
        dict(type='text', text=prompt),
        dict(type='image_url', image_url=dict(url='data:image/jpeg;base64,' + jpeg_b64))])],
        temperature=temperature, max_tokens=max_tokens)


def build_gemini_payload(prompt, jpeg_b64, temperature, max_tokens):
    return dict(contents=[dict(role='user', parts=[dict(text=prompt),
                dict(inline_data=dict(mime_type='image/jpeg', data=jpeg_b64))])],
                generationConfig=dict(temperature=temperature, topP=0.95,
                                      topK=40, maxOutputTokens=max_tokens))


def extract_text(provider, document):
    try:
        if provider == 'openai':
            text = document['choices'][0]['message']['content'] or ''
        else:
            text = ''.join(part.get('text', '') for part in document['candidates'][0]['content']['parts'])
        if not isinstance(text, str):
            raise TypeError('text is not a string')
        return text
    except (KeyError, IndexError, TypeError, AttributeError):
        raise CloudError('BAD_PAYLOAD', 'provider response has no valid text') from None


class CloudBackend:
    def __init__(self, provider, model, base_url, api_key_env="OPENAI_API_KEY", temperature=0.4,
                 max_tokens=1024, timeout_s=40.0, allow_http_localhost=False, api_key=None):
        if provider not in ('openai', 'gemini') or not model:
            raise ValueError('unsupported cloud provider or empty remote model')
        parsed = urllib.parse.urlsplit(base_url)
        local_http = (allow_http_localhost and parsed.scheme == 'http'
                      and parsed.hostname in ('127.0.0.1', 'localhost', '::1'))
        if (parsed.scheme != 'https' and not local_http) or not parsed.hostname \
                or parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError('cloud endpoint must be HTTPS without credentials/query/fragment')
        if not isinstance(api_key_env, str) or not (os.environ.get(api_key_env) or api_key):
            raise ValueError('cloud credential environment is not configured')
        if not math.isfinite(timeout_s) or timeout_s <= 0 or not math.isfinite(temperature) \
                or not 0 <= temperature <= 2 or type(max_tokens) is not int or max_tokens <= 0:
            raise ValueError('invalid cloud generation limits')
        self.provider, self.model = provider, model
        self.base_url, self.api_key_env = base_url.rstrip('/'), api_key_env
        self._api_key = api_key
        self.temperature, self.max_tokens, self.timeout_s = temperature, max_tokens, timeout_s
        self._closed = False
        self._lock = threading.Lock()
        self.metadata = dict(engine='cloud', provider=provider, remote_model=model)

    def infer(self, prompt, image_base64, timeout):
        if not isinstance(prompt, str) or not prompt.strip() or not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('non-empty prompt and positive timeout required')
        try:
            if not base64.b64decode(image_base64, validate=True):
                raise ValueError('empty image')
        except (ValueError, TypeError):
            raise ValueError('invalid base64 image') from None
        with self._lock:
            if self._closed:
                raise RuntimeError('cloud backend is closed')
            key = os.environ.get(self.api_key_env) or self._api_key
            if not key:
                raise CloudError('NOT_READY', 'cloud credential unavailable')
            if self.provider == 'openai':
                url = self.base_url + '/chat/completions'
                body = build_openai_payload(prompt, image_base64, self.model,
                                            self.temperature, self.max_tokens)
                headers = {'Authorization': 'Bearer ' + key}
            else:
                url = self.base_url + '/models/' + urllib.parse.quote(self.model, safe='') + ':generateContent'
                body = build_gemini_payload(prompt, image_base64, self.temperature, self.max_tokens)
                headers = {'x-goog-api-key': key}
            request = urllib.request.Request(url, data=json.dumps(body, allow_nan=False).encode(),
                                             headers=dict(headers, **{'Content-Type': 'application/json'}))
            try:
                with urllib.request.urlopen(request, timeout=min(timeout, self.timeout_s)) as response:
                    document = json.loads(response.read(16 * 1024 * 1024).decode())
            except urllib.error.HTTPError as error:
                codes = {401: 'AUTH_ERROR', 403: 'AUTH_ERROR', 429: 'RATE_LIMITED'}
                # Never forward vendor response bodies/URLs: they may contain secrets.
                status = error.code
                error.close()
                raise CloudError(codes.get(status, 'HTTP_ERROR'),
                                 'cloud HTTP status {}'.format(status)) from None
            except (TimeoutError, socket.timeout):
                raise CloudError('TIMEOUT', 'cloud request timed out') from None
            except urllib.error.URLError as error:
                code = 'TIMEOUT' if isinstance(error.reason, (TimeoutError, socket.timeout)) else 'TRANSPORT_ERROR'
                raise CloudError(code, 'cloud transport failed') from None
            except (ValueError, UnicodeError):
                raise CloudError('BAD_PAYLOAD', 'cloud response is not JSON') from None
            return extract_text(self.provider, document)

    def close(self):
        with self._lock:
            self._closed = True
            self._api_key = None
