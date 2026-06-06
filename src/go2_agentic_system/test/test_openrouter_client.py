from __future__ import annotations

import importlib
import os
from pathlib import Path


def test_openrouter_client_loads_local_env(tmp_path, monkeypatch):
    monkeypatch.delenv('OPENROUTER_API_KEY', raising=False)
    monkeypatch.chdir(tmp_path)
    (tmp_path / '.env.local').write_text('OPENROUTER_API_KEY=test-local-key\n', encoding='utf-8')

    module = importlib.import_module('go2_agentic_system.openrouter_client')
    module = importlib.reload(module)
    client = module.OpenRouterClient()

    assert client.available is True
    assert os.environ.get('OPENROUTER_API_KEY') == 'test-local-key'
    assert client.api_key == 'test-local-key'
    assert client.base_url == 'https://openrouter.ai/api/v1/chat/completions'


def test_openrouter_client_respects_explicit_base_url(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    (tmp_path / '.env.local').write_text('OPENROUTER_API_KEY=test-explicit-key\n', encoding='utf-8')

    module = importlib.import_module('go2_agentic_system.openrouter_client')
    module = importlib.reload(module)
    client = module.OpenRouterClient(base_url='https://example.invalid/api')

    assert client.available is True
    assert client.base_url == 'https://example.invalid/api'


def test_describe_scene_requests_resume_and_tour_fields(monkeypatch):
    module = importlib.import_module('go2_agentic_system.openrouter_client')
    module = importlib.reload(module)
    client = module.OpenRouterClient(api_key='test-key')

    captured = {}

    def fake_complete_json(prompt, **kwargs):
        captured['prompt'] = prompt
        captured['kwargs'] = kwargs
        return {
            'ok': True,
            'raw': {},
            'text': '{}',
            'parsed': {
                'label': 'alpha_lab',
                'summary': 'alpha lab demo area',
                'room': 'lab',
                'category': 'workstation',
                'aliases': ['alpha'],
                'tags': ['demo'],
                'tour_fact': 'This is the alpha lab.',
                'navigation_hint': 'approach the doorway',
                'resume_hook': 'resume at the doorway marker',
                'safety_notes': 'door clearance matters',
                'scene_context': 'desk and signage',
                'capture_kind': 'place',
                'confidence': 0.81,
                'objects': ['desk', 'door'],
            },
        }

    monkeypatch.setattr(client, 'complete_json', fake_complete_json)

    result = client.describe_scene('data:image/jpeg;base64,ZmFrZQ==', hint='label the doorway')

    assert result['parsed']['navigation_hint'] == 'approach the doorway'
    assert 'resume_hook' in captured['prompt']
    assert 'tour_fact' in captured['prompt']
    assert 'capture_kind' in captured['prompt']
    assert captured['kwargs']['image_data_urls'] == ['data:image/jpeg;base64,ZmFrZQ==']
