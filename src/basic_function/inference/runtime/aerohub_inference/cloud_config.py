"""Model-name -> declared engines -> private execution configuration.

No engine is inferred from environment variables. A single declared engine is
selected automatically; multiple alternatives require active_engines[model_name].
"""
import os
from pathlib import Path
import urllib.parse

import yaml

DEFAULT_URL = 'https://open.bigmodel.cn/api/paas/v4'
ENGINE_NAMES = {'cloud', 'cuda', 'houmo'}


def read_model_catalog(path):
    try:
        document = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError, TypeError):
        raise ValueError('cannot read model configuration YAML') from None
    if not isinstance(document, dict) or set(document) - {'models', 'active_engines'} \
            or not isinstance(document.get('models'), dict) or not document['models']:
        raise ValueError('expected a non-empty models mapping')
    for name, engines in document['models'].items():
        if not isinstance(name, str) or not name.strip() or len(name) > 128 \
                or not isinstance(engines, dict) or not engines or set(engines) - ENGINE_NAMES \
                or any(not isinstance(config, dict) for config in engines.values()):
            raise ValueError('each model must declare one or more known engine configuration mappings')
    selected = document.get('active_engines', {})
    if not isinstance(selected, dict) or set(selected) - set(document['models']):
        raise ValueError('active_engines must refer to declared models')
    for name, engine in selected.items():
        if not isinstance(engine, str) or engine not in document['models'][name]:
            raise ValueError('active engine is not declared for its model')
    return document


def _cloud_configuration(model_name, configuration, environ):
    result = dict(configuration)
    for field in ('base_url', 'model', 'api_key'):
        reference = result.pop(field + '_env', None)
        if reference is not None:
            if not isinstance(reference, str) or not reference:
                raise ValueError('environment references must be non-empty variable names')
            if reference in environ:
                result[field] = environ[reference]
    # Environment credentials are resolved per model, not globally across all entries.
    result.setdefault('base_url', DEFAULT_URL)
    result.setdefault('model', model_name)
    result.setdefault('provider', 'openai')
    for field in ('base_url', 'model', 'api_key'):
        if not isinstance(result.get(field), str) or not result[field].strip():
            raise ValueError('selected cloud engine needs URL/model and api_key or api_key_env')
    parsed = urllib.parse.urlsplit(result['base_url'])
    local_test = (result.get('allow_http_localhost') is True and parsed.scheme == 'http'
                  and parsed.hostname in ('127.0.0.1', 'localhost', '::1'))
    if (parsed.scheme != 'https' and not local_test) or not parsed.hostname \
            or parsed.username or parsed.password or parsed.query or parsed.fragment:
        raise ValueError('cloud base_url must be HTTPS without credentials/query/fragment')
    if len(result['model']) > 128 or any(ord(c) < 32 for c in result['model']) \
            or any(c.isspace() or ord(c) < 33 or ord(c) > 126 for c in result['api_key']):
        raise ValueError('invalid remote model or API key format')
    result['base_url'] = result['base_url'].rstrip('/')
    # Prevent CloudBackend's legacy default OPENAI_API_KEY from cross-binding models.
    result['api_key_env'] = ''
    return result


def select_model(catalog, model_name, environ=None):
    env = os.environ if environ is None else environ
    if model_name not in catalog['models']:
        raise ValueError('model_name is not declared')
    engines = catalog['models'][model_name]
    active = catalog.get('active_engines', {}).get(model_name)
    if active is None:
        if len(engines) != 1:
            raise ValueError('multiple engines require an explicit active_engines selection')
        active = next(iter(engines))
    configuration = dict(engines[active])
    if active == 'cloud':
        configuration = _cloud_configuration(model_name, configuration, env)
    return dict(engine=active, configuration=configuration)


def load_model_config(path, model_name, environ=None):
    return select_model(read_model_catalog(path), model_name, environ)
