from __future__ import annotations

import json
import os
import re
import urllib.request
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

try:
    from dotenv import load_dotenv
except Exception:
    load_dotenv = None


def _load_local_env() -> None:
    cwd = Path.cwd().resolve()
    for candidate in [cwd, *cwd.parents]:
        for name in ('.env.local', '.env'):
            env_path = candidate / name
            if env_path.exists():
                if load_dotenv is not None:
                    load_dotenv(env_path, override=False)
                else:
                    try:
                        with env_path.open('r', encoding='utf-8') as handle:
                            for raw in handle:
                                line = raw.strip()
                                if not line or line.startswith('#') or '=' not in line:
                                    continue
                                key, value = line.split('=', 1)
                                key = key.strip()
                                value = value.strip().strip('"').strip("'")
                                if key and key not in os.environ:
                                    os.environ[key] = value
                    except Exception:
                        pass
                return


_load_local_env()


class OpenRouterClient:
    def __init__(
        self,
        api_key: Optional[str] = None,
        model: Optional[str] = None,
        base_url: Optional[str] = None,
        site_url: Optional[str] = None,
        site_name: Optional[str] = None,
    ) -> None:
        self.api_key = api_key or os.getenv('OPENROUTER_API_KEY', '')
        self.model = model or os.getenv('OPENROUTER_MODEL', 'google/gemini-2.5-flash')
        self.base_url = base_url or os.getenv('OPENROUTER_BASE_URL', 'https://openrouter.ai/api/v1/chat/completions')
        self.site_url = site_url or os.getenv('OPENROUTER_SITE_URL', 'http://localhost')
        self.site_name = site_name or os.getenv('OPENROUTER_SITE_NAME', 'go2-agentic-system')

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

    def _request(self, messages: List[Dict[str, Any]], *, temperature: float = 0.2, max_tokens: int = 512) -> Dict[str, Any]:
        body = {
            'model': self.model,
            'temperature': temperature,
            'max_tokens': max_tokens,
            'messages': messages,
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
        with urllib.request.urlopen(req, timeout=45) as resp:
            return json.loads(resp.read().decode('utf-8'))

    def complete_text(
        self,
        prompt: str,
        *,
        system_prompt: Optional[str] = None,
        image_data_urls: Optional[Iterable[str]] = None,
        temperature: float = 0.2,
        max_tokens: int = 512,
    ) -> Dict[str, Any]:
        if not self.available:
            return {'ok': False, 'error': 'OPENROUTER_API_KEY not set'}
        content: List[Dict[str, Any]] = []
        if system_prompt:
            content.append({'role': 'system', 'content': system_prompt})
        user_content: List[Dict[str, Any]] = [{'type': 'text', 'text': prompt}]
        for image_url in list(image_data_urls or []):
            user_content.append({'type': 'image_url', 'image_url': {'url': image_url}})
        messages = []
        if system_prompt:
            messages.append({'role': 'system', 'content': system_prompt})
        messages.append({'role': 'user', 'content': user_content})
        try:
            payload = self._request(messages, temperature=temperature, max_tokens=max_tokens)
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}
        return {'ok': True, 'raw': payload, 'text': self._extract_text(payload)}

    def complete_json(
        self,
        prompt: str,
        *,
        system_prompt: Optional[str] = None,
        image_data_urls: Optional[Iterable[str]] = None,
        temperature: float = 0.1,
        max_tokens: int = 512,
        default: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        result = self.complete_text(
            prompt,
            system_prompt=system_prompt,
            image_data_urls=image_data_urls,
            temperature=temperature,
            max_tokens=max_tokens,
        )
        if not result.get('ok'):
            return {'ok': False, 'error': result.get('error', 'unknown'), 'parsed': default or {}}
        text = str(result.get('text', '')).strip()
        try:
            obj = json.loads(text)
        except Exception:
            match = re.search(r'\{.*\}', text, re.S)
            if match:
                try:
                    obj = json.loads(match.group(0))
                except Exception:
                    obj = default or {'label': '', 'summary': text.strip(), 'aliases': [], 'objects': []}
            else:
                obj = default or {'label': '', 'summary': text.strip(), 'aliases': [], 'objects': []}
        return {'ok': True, 'raw': result.get('raw'), 'parsed': obj, 'text': text}

    def describe_scene(self, image_data_url: str, hint: Optional[str] = None) -> Dict[str, Any]:
        prompt = (
            'You are creating a semantic memory for a mobile robot. '
            'Return strict JSON with keys: label, summary, room, category, aliases, tags, tour_fact, navigation_hint, '
            'resume_hook, safety_notes, scene_context, capture_kind, confidence, objects. '
            'label should be a short place name if obvious. '
            'Keep every string short and concrete, with at most 12 words per field. '
            'Use at most 3 aliases, 5 tags, and 5 objects. '
            'summary should be one sentence about what matters for navigation and tour continuity. '
            'tour_fact should be a guest-facing fact grounded in the scene. '
            'navigation_hint should tell the robot how to approach or re-find this location. '
            'resume_hook should explain what to remember when resuming between nearby nodes. '
            'safety_notes should mention blocked paths, hazards, or clearances if relevant. '
            'scene_context should capture distinctive visible landmarks. '
            'capture_kind should be one of place, landmark, transition, safe_anchor, obstacle, viewpoint. '
            'confidence must be a float between 0.0 and 1.0. '
            'Do not include markdown. '
        )
        if hint:
            prompt += f' Optional operator hint: {hint}.'
        return self.complete_json(prompt, image_data_urls=[image_data_url], temperature=0.1, max_tokens=700, default={
            'label': '',
            'summary': '',
            'room': '',
            'category': '',
            'aliases': [],
            'tags': [],
            'tour_fact': '',
            'navigation_hint': '',
            'resume_hook': '',
            'safety_notes': '',
            'scene_context': '',
            'capture_kind': '',
            'confidence': 0.0,
            'objects': [],
        })

    def analyze_navigation_failure(self, *, reason: str, goal: Dict[str, Any], route_summary: Dict[str, Any], scan_summary: Dict[str, Any], pose_summary: Dict[str, Any]) -> Dict[str, Any]:
        prompt = (
            'You are a robot navigation recovery planner. Return ONLY JSON. '
            'Classify the failure and recommend the next recovery action. '
            'Allowed failure_type values: localization_lost, map_mismatch, dynamic_blockage, ambiguous_goal, unreachable_goal, unknown. '
            'Allowed action values: retry_nav, relocalize, rotate_left, rotate_right, backup, creep_forward, alternate_goal, safe_anchor, wait, stop. '
            'Include fields: failure_type, action, confidence, safe_anchor, alternate_query, note, speech. '
            f'Failure reason: {reason}. Goal: {json.dumps(goal)}. Route: {json.dumps(route_summary)}. '
            f'Scan summary: {json.dumps(scan_summary)}. Pose summary: {json.dumps(pose_summary)}.'
        )
        return self.complete_json(prompt, temperature=0.1, max_tokens=350, default={
            'failure_type': 'unknown',
            'action': 'retry_nav',
            'confidence': 0.2,
            'safe_anchor': '',
            'alternate_query': '',
            'note': '',
            'speech': 'I am recovering from a navigation failure.',
        })

    def propose_waypoint(self, *, context: Dict[str, Any], image_data_url: Optional[str] = None) -> Dict[str, Any]:
        prompt = (
            'You are a visual waypointing assistant for a robot. '
            'Return ONLY JSON with keys: action, label, confidence, speech. '
            'Use action values forward, turn_left, turn_right, backup, stop, wait, defer. '
            f'Context: {json.dumps(context)}.'
        )
        urls = [image_data_url] if image_data_url else None
        return self.complete_json(prompt, image_data_urls=urls, temperature=0.15, max_tokens=250, default={
            'action': 'defer',
            'label': '',
            'confidence': 0.0,
            'speech': '',
        })

    def debate_role(self, role: str, *, context: Dict[str, Any], image_data_url: Optional[str] = None) -> Dict[str, Any]:
        prompt = (
            f'You are the {role} debate agent for Sparky. '
            'Return ONLY JSON with keys: event_worthy, label, replay_variations, update_strength, conclusion, confidence, speech. '
            'Keep the response brief and structured. '
            f'Context: {json.dumps(context)}.'
        )
        urls = [image_data_url] if image_data_url else None
        return self.complete_json(prompt, image_data_urls=urls, temperature=0.2, max_tokens=300, default={
            'event_worthy': False,
            'label': role,
            'replay_variations': [],
            'update_strength': 0.0,
            'conclusion': '',
            'confidence': 0.0,
            'speech': '',
        })
