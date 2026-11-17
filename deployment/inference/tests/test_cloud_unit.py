"""Cloud plugin/configuration tests; local protocol peer, never a real provider."""
import base64
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import threading
import unittest
import yaml
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

from aerohub_inference.backends.cloud import CloudBackend, CloudError
from aerohub_inference.engines import create_engine
from aerohub_inference.cloud_config import load_model_config, read_model_catalog, select_model, DEFAULT_URL

class ConfigurationTest(unittest.TestCase):
    def setUp(self):
        self.workspace=tempfile.TemporaryDirectory()
        self.addCleanup(self.workspace.cleanup)
        self.config=Path(self.workspace.name)/'local.yaml'

    def write(self, models, active=None):
        body={'models':models}
        if active is not None:
            body['active_engines']=active
        self.config.write_text(yaml.safe_dump(body))
        return self.config

    def cloud(self, **changes):
        return dict(model='glm-4v-flash',api_key='file-key',**changes)

    def adapter(self):
        root=Path(__file__).resolve().parents[3]
        spec=importlib.util.spec_from_file_location('vlm_adapter',root/'src/basic_function/inference/models/vision_language/adapter.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_logical_model_name_is_not_provider_model(self):
        self.write({'spf':{'cloud':self.cloud()}})
        binding=load_model_config(self.config,'spf',{})
        self.assertEqual(binding['engine'],'cloud')
        self.assertEqual(binding['configuration']['model'],'glm-4v-flash')
        self.assertEqual(binding['configuration']['base_url'],DEFAULT_URL)
        self.assertEqual(binding['configuration']['api_key'],'file-key')

    def test_single_local_engine_can_be_selected_without_cloud(self):
        self.write({'another-model':{'cuda':{'weights':'path'}}})
        self.assertEqual(load_model_config(self.config,'another-model',{}),
                         dict(engine='cuda',configuration={'weights':'path'}))
        self.write({'another-model':{'houmo':{'hmm_path':'model.hmm'}}})
        self.assertEqual(load_model_config(self.config,'another-model',{})['engine'],'houmo')

    def test_multiple_engines_require_deployment_selection(self):
        self.write({'spf':{'cloud':self.cloud(),'cuda':{'weights':'path'}}})
        with self.assertRaises(ValueError):
            load_model_config(self.config,'spf',{})
        self.write({'spf':{'cloud':self.cloud(),'cuda':{'weights':'path'}}},{'spf':'cloud'})
        self.assertEqual(load_model_config(self.config,'spf',{})['engine'],'cloud')
        self.write({'spf':{'cloud':self.cloud(),'cuda':{'weights':'path'}}},{'spf':'cuda'})
        self.assertEqual(load_model_config(self.config,'spf',{})['engine'],'cuda')

    def test_absent_models_and_engines_have_no_default_support(self):
        self.write({'spf':{'cloud':self.cloud()}})
        with self.assertRaises(ValueError):
            load_model_config(self.config,'missing',{'OPENAI_API_KEY':'env-key'})
        for models,active in [({'spf':{}},None),({'spf':{'cloud':self.cloud()}},{'spf':'cuda'}),
                              ({'spf':{'unknown':{}}},None),({'spf':{'cloud':self.cloud()}},{'missing':'cloud'})]:
            self.write(models,active)
            with self.assertRaises(ValueError):
                read_model_catalog(self.config)

    def test_environment_does_not_implicitly_declare_an_engine(self):
        with self.assertRaises(ValueError):
            load_model_config(None,'spf',{'OPENAI_API_KEY':'env-key'})
        self.config.write_text('api_key: old-flat-format')
        with self.assertRaises(ValueError):
            load_model_config(self.config,'spf',{})

    def test_environment_overrides_only_its_declared_model_binding(self):
        self.write({'spf':{'cloud':dict(model='glm',api_key='old',api_key_env='SPF_KEY',model_env='SPF_REMOTE')},
                    'other':{'cloud':dict(model='other-model',api_key='other-key')}})
        catalog=read_model_catalog(self.config)
        env={'SPF_KEY':'new-key','SPF_REMOTE':'new-remote','OPENAI_API_KEY':'unrelated-key','OPENAI_MODEL':'unrelated-model'}
        first=select_model(catalog,'spf',env)['configuration']
        second=select_model(catalog,'other',env)['configuration']
        self.assertEqual((first['api_key'],first['model']),('new-key','new-remote'))
        self.assertEqual((second['api_key'],second['model']),('other-key','other-model'))
        self.assertEqual(second['api_key_env'],'')

    def test_nonselected_model_does_not_need_credentials(self):
        self.write({'spf':{'cloud':self.cloud()},'other':{'cloud':{}}})
        self.assertEqual(load_model_config(self.config,'spf',{})['engine'],'cloud')

    def test_bad_yaml_does_not_echo_secret(self):
        self.config.write_text('models: [not-a-real-secret')
        with self.assertRaises(ValueError) as caught:
            read_model_catalog(self.config)
        self.assertNotIn('not-a-real-secret',str(caught.exception))

    def test_predictor_uses_spf_identity_and_ignores_unrelated_local_models(self):
        self.write({'spf':{'cloud':self.cloud()},'yopo':{'cuda':{'weights':'not-loaded'}}})
        with patch.dict(os.environ,{'INFERENCE_MODEL_CONFIG':str(self.config)},clear=True):
            predictor=self.adapter().Predictor()
            try:
                self.assertEqual(predictor.metadata['model_names'],['spf'])
                self.assertEqual(predictor.engines['spf'].metadata['remote_model'],'glm-4v-flash')
            finally:
                predictor.close()

    def test_selecting_unimplemented_local_spf_never_falls_back_to_cloud(self):
        self.write({'spf':{'cloud':self.cloud(),'cuda':{}}},{'spf':'cuda'})
        module=self.adapter()
        with patch.dict(os.environ,{'INFERENCE_MODEL_CONFIG':str(self.config)},clear=True), \
                patch.object(module,'create_engine') as create:
            with self.assertRaises(ValueError):
                module.Predictor()
            create.assert_not_called()

    def test_engine_initialization_failure_never_tries_another_alternative(self):
        self.write({'spf':{'cloud':self.cloud(),'cuda':{}}},{'spf':'cloud'})
        module=self.adapter()
        with patch.dict(os.environ,{'INFERENCE_MODEL_CONFIG':str(self.config)},clear=True), \
                patch.object(module,'create_engine',side_effect=RuntimeError('unavailable')) as create:
            with self.assertRaises(RuntimeError):
                module.Predictor()
            self.assertEqual(create.call_count,1)
            self.assertEqual(create.call_args.args[0],'cloud')

    def demo_module(self):
        root=Path(__file__).resolve().parents[3]
        spec=importlib.util.spec_from_file_location('spf_demo',root/'deployment/airsim/demos/spf_language_nav/run.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_demo_resolves_logical_spf_entry(self):
        root=Path(self.workspace.name)/'repo'
        helper=root/'src/basic_function/inference/runtime/aerohub_inference/cloud_config.py'
        helper.parent.mkdir(parents=True)
        source=Path(__file__).resolve().parents[3]/'src/basic_function/inference/runtime/aerohub_inference/cloud_config.py'
        helper.write_text(source.read_text())
        config=root/'config/inference.cloud.local.yaml'
        config.parent.mkdir()
        config.write_text(yaml.safe_dump({'models':{'spf':{'cloud':self.cloud()}}}))
        module=self.demo_module()
        with patch.object(module,'ROOT',root),patch.dict(os.environ,{},clear=True):
            self.assertEqual(module.cloud_binding(),('spf',config))

    def test_demo_forwards_model_specific_environment_names_not_secret_values(self):
        self.write({'spf':{'cloud':dict(model='glm',api_key_env='SPF_KEY')}})
        module=self.demo_module()
        with patch.object(module,'cloud_binding',return_value=('spf',self.config)), \
                patch.object(module,'launch') as launch, \
                patch.dict(os.environ,{'SPF_KEY':'not-a-real-secret','OPENAI_API_KEY':'unrelated'},clear=True):
            module.launch_inference(Path(self.workspace.name))
        env=launch.call_args.kwargs['env']
        self.assertIn('SPF_KEY',env)
        self.assertNotIn('OPENAI_API_KEY',env)
        self.assertNotIn('not-a-real-secret',repr(launch.call_args))

    def test_engine_selection_is_explicit_and_lazy(self):
        with self.assertRaises(ValueError):
            create_engine('unknown')
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaises(ValueError):
                create_engine('cloud',provider='openai',model='a',base_url='https://example.com',api_key_env='MISSING')


class CloudProtocolTest(unittest.TestCase):
    def setUp(self):
        self.requests = []
        self.status = 200
        self.body = {'choices': [{'message': {'content': 'generic text'}}]}
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                owner.requests.append((self.path, {k.lower():v for k,v in self.headers.items()}, body))
                self.send_response(owner.status)
                self.end_headers()
                self.wfile.write(json.dumps(owner.body).encode())

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.env = patch.dict(os.environ, {'TEST_CLOUD_KEY': 'not-a-real-key'})
        self.env.start()
        self.engine = create_engine('cloud', provider='openai', model='test-model',
            base_url='http://127.0.0.1:{}'.format(self.server.server_port),
            api_key_env='TEST_CLOUD_KEY', allow_http_localhost=True)

    def tearDown(self):
        self.engine.close()
        self.env.stop()
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def test_request_identity_and_text_are_preserved(self):
        self.assertEqual(self.engine.infer('exact prompt', base64.b64encode(b'jpeg').decode(), 1.), 'generic text')
        path, headers, body = self.requests[0]
        self.assertEqual(path, '/chat/completions')
        self.assertEqual(headers['authorization'], 'Bearer not-a-real-key')
        self.assertEqual(body['model'], 'test-model')
        self.assertEqual(body['messages'][0]['content'][0]['text'], 'exact prompt')
        self.engine.close()
        with self.assertRaises(RuntimeError):
            self.engine.infer('p', 'eA==', 1.)

    def test_gemini_wire_format_uses_private_header_and_generic_text(self):
        self.body = {'candidates': [{'content': {'parts': [{'text': 'a'}, {'text': 'b'}]}}]}
        engine = create_engine('cloud', provider='gemini', model='test-gemini',
            base_url='http://127.0.0.1:{}'.format(self.server.server_port),
            api_key_env='TEST_CLOUD_KEY', allow_http_localhost=True)
        try:
            self.assertEqual(engine.infer('exact prompt', 'eA==', 1.), 'ab')
            path, headers, body = self.requests[0]
            self.assertEqual(path, '/models/test-gemini:generateContent')
            self.assertEqual(headers['x-goog-api-key'], 'not-a-real-key')
            self.assertEqual(body['contents'][0]['parts'][0]['text'], 'exact prompt')
            self.assertNotIn('key=', path)
        finally:
            engine.close()

    def test_key_from_file_can_execute_without_key_environment(self):
        engine = CloudBackend('openai','file-model',
            'http://127.0.0.1:{}'.format(self.server.server_port),
            api_key_env='ABSENT_TEST_KEY',api_key='file-only-key',allow_http_localhost=True)
        try:
            self.assertEqual(engine.infer('p','eA==',1.),'generic text')
            self.assertEqual(self.requests[0][1]['authorization'],'Bearer file-only-key')
        finally:
            engine.close()

    def test_rate_limit_error_does_not_expose_vendor_body_or_key(self):
        self.status = 429
        self.body = {'error': 'echo not-a-real-key and private input'}
        with self.assertRaises(CloudError) as caught:
            self.engine.infer('p', 'eA==', 1.)
        self.assertEqual(caught.exception.code, 'RATE_LIMITED')
        self.assertNotIn('not-a-real-key', str(caught.exception))
        self.assertNotIn('private input', str(caught.exception))

    def test_malformed_response_is_explicit(self):
        self.body = {}
        with self.assertRaises(CloudError) as caught:
            self.engine.infer('p', 'eA==', 1.)
        self.assertEqual(caught.exception.code, 'BAD_PAYLOAD')

    def test_insecure_remote_endpoint_is_rejected(self):
        with self.assertRaises(ValueError):
            CloudBackend('openai', 'a', 'http://remote.invalid', 'TEST_CLOUD_KEY')


if __name__ == '__main__':
    unittest.main(verbosity=2)
