from __future__ import annotations

from typing import Dict, List, Tuple
import re

from .place_store import Place

DEFAULT_SYNONYMS = {
    'passage': ['hallway', 'corridor', 'aisle', 'walkway'],
    'cabinet': ['rack', 'server_cabinet', 'storage_cabinet'],
    'chairs': ['chair', 'seating', 'seating_area'],
    'desk': ['workstation', 'table', 'office_desk'],
    'entrance': ['entry', 'door', 'lab_entrance'],
}


def slugify(text: str) -> str:
    text = (text or '').strip().lower()
    text = re.sub(r'[^a-z0-9]+', '_', text)
    text = re.sub(r'_+', '_', text).strip('_')
    return text


class SemanticMemory:
    def __init__(self, synonyms: Dict[str, List[str]] | None = None) -> None:
        self.synonyms = {**DEFAULT_SYNONYMS}
        if synonyms:
            for k, vals in synonyms.items():
                self.synonyms[slugify(k)] = [slugify(v) for v in vals]

    def expand_tokens(self, query: str) -> List[str]:
        tokens = [slugify(t) for t in query.split() if slugify(t)]
        expanded = set(tokens)
        for t in list(tokens):
            vals = self.synonyms.get(t, [])
            expanded.update(vals)
            for k, synonyms in self.synonyms.items():
                if t in synonyms:
                    expanded.add(k)
                    expanded.update(synonyms)
        return sorted(expanded)

    def build_relationships(self, places: List[Place]) -> Dict[str, Dict[str, List[str]]]:
        graph: Dict[str, Dict[str, List[str]]] = {}
        for p in places:
            graph[p.name] = {'same_room': [], 'same_category': [], 'near': []}
        for i, a in enumerate(places):
            for b in places[i + 1:]:
                if a.room and b.room and a.room == b.room:
                    graph[a.name]['same_room'].append(b.name)
                    graph[b.name]['same_room'].append(a.name)
                if a.category and b.category and a.category == b.category:
                    graph[a.name]['same_category'].append(b.name)
                    graph[b.name]['same_category'].append(a.name)
                dx = a.x - b.x
                dy = a.y - b.y
                if (dx * dx + dy * dy) ** 0.5 <= 2.0:
                    graph[a.name]['near'].append(b.name)
                    graph[b.name]['near'].append(a.name)
        return graph

    def resolve(self, query: str, places: List[Place]) -> Tuple[Place | None, float, str]:
        q = slugify(query.replace('go to', '').replace('near', '').strip())
        expanded = self.expand_tokens(query)
        best: Place | None = None
        best_score = 0.0
        why = ''
        for p in places:
            fields = [p.name, p.room, p.category, p.description, *(p.aliases or []), *(p.tags or [])]
            score = 0.0
            reasons = []
            for field in fields:
                sf = slugify(str(field))
                if not sf:
                    continue
                if sf == q:
                    score += 100
                    reasons.append('exact')
                if q and q in sf:
                    score += 50
                    reasons.append('substring')
                for tok in expanded:
                    if tok == sf:
                        score += 40
                    elif tok and tok in sf:
                        score += 15
                q_tokens = set(q.split('_')) if q else set()
                f_tokens = set(sf.split('_'))
                overlap = len(q_tokens & f_tokens)
                if overlap:
                    score += overlap * 10
                    reasons.append('token_overlap')
            if query.lower().startswith('near '):
                score += 5
            if score > best_score:
                best_score = score
                best = p
                why = ','.join(sorted(set(reasons))) or 'semantic_match'
        return best, best_score, why
