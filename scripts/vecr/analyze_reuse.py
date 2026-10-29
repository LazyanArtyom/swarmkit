#!/usr/bin/env python3
"""Audit the v8 direct Client versus routed SwarmClient experiment."""
import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path

FIELDS = ('phase', 'distance_m', 'time_ms', 'reported_covered_m',
          'truth_covered_m', 'recoveries', 'reported_arrived',
          'reported_complete', 'truth_complete', 'unresolved_m')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.output
    manifest = json.loads((root / 'manifest.json').read_text())
    assert manifest['status'] == 'COMPLETE'
    with (root / 'per_attempt.csv').open(newline='') as stream:
        attempts = list(csv.DictReader(stream))
    assert len(attempts) == 100
    pairs = defaultdict(dict)
    phases = Counter()
    for attempt in attempts:
        key = (attempt['scenario'], attempt['seed'])
        kind = attempt['client_kind']
        assert kind in ('direct', 'routed') and kind not in pairs[key]
        pairs[key][kind] = attempt
        phases[(attempt['scenario'], kind, attempt['phase'])] += 1
    assert len(pairs) == 50 and all(set(pair) == {'direct', 'routed'} for pair in pairs.values())
    differences = []
    for (case, seed), pair in sorted(pairs.items()):
        for field in FIELDS:
            if pair['direct'][field] != pair['routed'][field]:
                differences.append({'scenario': case, 'seed': seed, 'field': field,
                                    'direct': pair['direct'][field], 'routed': pair['routed'][field]})
    analysis = root / 'analysis'
    analysis.mkdir(exist_ok=True)
    result = {'protocol': 'v8', 'source_revision': manifest['source_revision'],
              'attempts': len(attempts), 'matched_pairs': len(pairs),
              'fields_compared': FIELDS, 'field_differences': differences,
              'phases': [{'scenario': case, 'client': kind, 'phase': phase, 'count': count}
                         for (case, kind, phase), count in sorted(phases.items())]}
    (analysis / 'reuse_results.json').write_text(json.dumps(result, indent=2) + '\n')
    lines = ['# Frozen v8 client-path reuse audit', '',
             f"Source revision: `{manifest['source_revision']}`. Agent-backed attempts: **100**, paired direct/routed seeds: **50**.",
             '', f"Exact differences across {len(FIELDS)} predeclared result fields: **{len(differences)}**.",
             '', '| Case | Direct terminal | Routed terminal | Matched pairs |',
             '|---|---|---|---:|']
    for case in sorted({key[0] for key in pairs}):
        direct = Counter(pair['direct']['phase'] for (c, _), pair in pairs.items() if c == case)
        routed = Counter(pair['routed']['phase'] for (c, _), pair in pairs.items() if c == case)
        lines.append(f"| {case} | {dict(direct)} | {dict(routed)} | 10 |")
    lines += ['', 'This tests two SDK transport entry paths to the same agent method. It does not test two independent graph algorithms or multi-UAV scheduling.',
              'The earlier sandbox-blocked 100-attempt run is preserved separately under `results/vecr/v8/reuse/` as an unsuccessful infrastructure attempt.', '']
    (analysis / 'REUSE_RESULTS.md').write_text('\n'.join(lines))
    assert not differences, differences[:10]
    print(f"VALID: {len(attempts)} attempts, {len(pairs)} pairs, zero differences across {len(FIELDS)} fields")


if __name__ == '__main__':
    main()
