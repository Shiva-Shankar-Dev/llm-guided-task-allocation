"""Named places, explicit aliases, and distinct robot arrival slots."""
import copy
import re

from .fleet import vector

ARRIVAL_TOLERANCE = .15


def normalize_stations(data):
    if not isinstance(data, dict) or not data:
        raise ValueError('Stations must be a nonempty object')
    result = {}
    aliases = {}
    for name, value in data.items():
        spec = value if isinstance(value, dict) else {'slots': {'arrival': value}}
        slots = spec.get('slots', {})
        if not isinstance(slots, dict) or not slots:
            raise ValueError(f'{name}: configure at least one arrival slot')
        names = spec.get('aliases', [])
        if not isinstance(names, list) or any(not isinstance(a, str) or not a.strip() for a in names):
            raise ValueError(f'{name}: invalid aliases')
        result[name] = {'aliases': names, 'slots': {s: vector(p, 2, s).tolist() for s, p in slots.items()}}
        for alias in [name, name.replace('_', ' '), *names]:
            key = alias.casefold().strip()
            if key in aliases and aliases[key] != name:
                raise ValueError(f'Ambiguous destination alias: {alias}')
            aliases[key] = name
    return result


def canonical_instruction(text, stations):
    aliases = {}
    for name, spec in normalize_stations(stations).items():
        for alias in [name, name.replace('_', ' '), *spec['aliases']]:
            aliases[alias.casefold()] = name
    pattern = r'(?<!\w)(' + '|'.join(re.escape(a) for a in sorted(aliases, key=len, reverse=True)) + r')(?!\w)'
    return re.sub(pattern, lambda m: aliases[m[0].casefold()], text, flags=re.IGNORECASE)


def canonical_intent(intent, stations):
    result = copy.deepcopy(intent)
    if isinstance(result, dict) and isinstance(result.get('jobs'), list):
        for job in result['jobs']:
            if isinstance(job, dict) and isinstance(job.get('locations'), list):
                job['locations'] = [canonical_instruction(s, stations) if isinstance(s, str) else s
                                    for s in job['locations']]
    return result
