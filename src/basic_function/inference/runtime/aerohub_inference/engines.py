"""Lazy alternative execution plugins; model adapters own their internal contracts.

The public Function chooses model_name, not a tensor engine. A model adapter may
use tensor I/O or a remote protocol; these are deliberately not one tensor ABI.
"""
from importlib import import_module

ENGINES = {
    'cuda': ('aerohub_inference.backends.torch', 'TorchBackend'),
    'houmo': ('aerohub_houmo.backend', 'HmmBackend'),
    'cloud': ('aerohub_inference.backends.cloud', 'CloudBackend'),
}


def create_engine(name, **configuration):
    if name not in ENGINES:
        raise ValueError('unsupported inference engine: {}'.format(name))
    module, factory = ENGINES[name]
    return getattr(import_module(module), factory)(**configuration)
