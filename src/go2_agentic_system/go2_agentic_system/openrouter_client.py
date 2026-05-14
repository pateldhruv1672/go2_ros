from __future__ import annotations

import json
import os
import re
import urllib.request
from typing import Any, Dict, Optional


class OpenRouterClient:
    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None) -> None:
        self.api_key = api_key or os.getenv('OPENROUTER_API_KEY', '')
        self.model = model or os.getenv('OPENROUTER_MODEL', 'google/gemini-2.5-flash')
        self.base_url = 'https://openrouter.ai/api/v1/chat/completions'
        self.site_url = os.getenv('OPENROUTER_SITE_URL', 'http://localhost')
        self.site_name = os.getenv('OPENROUTER_SITE_NAME', 'go2-agentic-system')

    @property
    def available(self) -> bool:
        return bool(self.api_key)

    def _extract_text(self, response: Dict[str, Any]) -> str:
        try:
            content = response['choices'][0]['message']['content']
        except Exception:
            return ''
        if isinstance(content, str):
            return content
        if isinstance(content, list):
            parts = []
            for item in content:
                if isinstance(item, dict) and item.get('type') == 'text':
                    parts.append(item.get('text', ''))
            return '\n'.join(parts)
        return str(content)

    def describe_scene(self, image_data_url: str, hint: Optional[str] = None) -> Dict[str, Any]:
        if not self.available:
            return {'ok': False, 'error': 'OPENROUTER_API_KEY not set'}
        prompt = (
            'You are creating a semantic memory for a mobile robot. '
            'Return strict JSON with keys: label, summary, aliases, objects. '
            'label should be a short place name if obvious. '
            'aliases should be a short list of alternative names. '
            'objects should be a short list of visible meaningful objects. '
            'Do not include markdown.'
        )
        if hint:
            prompt += f' Optional operator hint: {hint}.'
        body = {
            'model': self.model,
            'temperature': 0.1,
            'max_tokens': 250,
            'messages': [
                {
                    'role': 'user',
                    'content': [
                        {'type': 'text', 'text': prompt},
                        {'type': 'image_url', 'image_url': {'url': image_data_url}},
                    ],
                }
            ],
        }
        data = json.dumps(body).encode('utf-8')
        req = urllib.request.Request(
            self.base_url,
            data=data,
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
                'HTTP-Referer': self.site_url,
                'X-Title': self.site_name,
            },
            method='POST',
        )
        try:
            with urllib.request.urlopen(req, timeout=45) as resp:
                payload = json.loads(resp.read().decode('utf-8'))
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}
        text = self._extract_text(payload)
        try:
            obj = json.loads(text)
        except Exception:
            match = re.search(r'\{.*\}', text, re.S)
            if match:
                try:
                    obj = json.loads(match.group(0))
                except Exception:
                    obj = {'label': '', 'summary': text.strip(), 'aliases': [], 'objects': []}
            else:
                obj = {'label': '', 'summary': text.strip(), 'aliases': [], 'objects': []}
        return {'ok': True, 'raw': payload, 'parsed': obj}
