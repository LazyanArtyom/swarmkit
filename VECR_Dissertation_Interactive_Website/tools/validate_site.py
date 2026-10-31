#!/usr/bin/env python3
"""Cross-check reader claims and recorded replay rows against frozen evidence."""
import argparse
import csv
import gzip
import hashlib
import json
import re
from pathlib import Path

from bs4 import BeautifulSoup

SITE = Path(__file__).resolve().parents[1]
ROOT = SITE.parent
DATA = ROOT / 'results/vecr/v8/primary'


def rows(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', newline='') as stream:
        return list(csv.DictReader(stream))


def validate(browser=False):
    markup = (SITE / 'index.htm').read_text()
    document = BeautifulSoup(markup, 'lxml')
    manuscript = (SITE / 'source/dissertation.md').read_text()
    js = (SITE / 'assets/experiment-data.js').read_text()
    assert js.startswith('window.VECR_DATA = ')
    replay = json.loads(js.removeprefix('window.VECR_DATA = ').rstrip().removesuffix(';'))
    summary = json.loads((DATA / 'analysis/summary.json').read_text())
    raw = rows(DATA / 'per_attempt.csv')
    conditions = rows(DATA / 'analysis/condition_summary.csv')
    assert len(raw) == 1320 and len(conditions) == 44
    assert summary['protocol'] == replay['protocol'] == 'v8'
    assert summary['source_revision'] == replay['source_revision']
    metadata = json.loads((SITE / 'assets/research-metadata.json').read_text())
    assert metadata['digital']['source_revision'] == summary['source_revision']
    assert metadata['digital']['binary_sha256'] == json.loads((DATA / 'manifest.json').read_text())['binary_sha256']
    assert metadata['reuse']['attempts'] == 100 and metadata['sitl_revised'] == 'TBD'
    assert 'results/vecr/current/' not in json.dumps(metadata)
    assert (SITE / 'assets/figures/inflight_effect.svg').exists()
    assert len(document.select('section.chapter')) == 20
    assert len(re.findall(r'^# \d\d /', manuscript, re.M)) == 20
    assert len(document.select('footer')) == 1
    scorecards = document.select('.metric-grid strong')
    assert [card.get_text(strip=True) for card in scorecards] == [
        '1,320', '0 / 1,200', '1,080 / 1,320', '540 / 1,320', '120 / 120'
    ]
    assert sum(int(row['reported_arrived']) for row in raw) == 1080
    assert sum(int(row['reported_complete']) for row in raw) == 540
    assert 'DIGITAL VALIDATION COMPLETE' in markup and 'SITL PENDING' in markup
    visible = BeautifulSoup(markup, 'lxml')
    for element in visible.select('script, style'):
        element.decompose()
    assert not re.search(r'\bv[78]\b', visible.get_text(' ', strip=True), re.I)
    hero_cards = document.select('.hero-stats > div')
    assert len(hero_cards) == 4
    assert hero_cards[2].select_one('strong').get_text(strip=True) == '100'
    assert 'validated client-path attempts' in hero_cards[2].get_text()
    assert 'one terminal' in manuscript.lower() and 'no post-arrival' in manuscript.lower()
    assert 'ARRIVED/INCOMPLETE' in manuscript and 'GetEdgeStatus' in manuscript
    assert 'proposed revised API contract' not in (SITE / 'assets/animations.js').read_text()
    assert 'FULFILLED' not in (SITE / 'assets/animations.js').read_text()
    assert 'AT_B_FINALIZING' not in markup and 'AT_B_FINALIZING' not in (SITE / 'assets/animations.js').read_text()
    assert 'RECOVERY_COMMAND_REJECTED' not in markup and 'RECOVERY_COMMAND_REJECTED' not in (SITE / 'assets/animations.js').read_text()
    assert 'qualified SITL' not in manuscript
    assert 'EdgeExecutionRequest' not in manuscript and 'ExecutionId' not in manuscript
    assert 'GetEdgeStatus(drone_id, attempt_id)' in manuscript
    assert 'q=min(25,18)=18' in markup
    assert len(document.select('.recovery-walkthrough')) == 1
    assert len(document.select('.outcome-guide > div')) == 4
    assert r'\bigcup' not in markup and r'\setminus' not in markup
    assert set(replay['replays']) == {'N00', 'N03', 'N07', 'N11', 'N19'}
    for case, policies in replay['replays'].items():
        assert set(policies) == {'B2', 'VECR'}
        for policy, recording in policies.items():
            folder = DATA / 'trials' / recording['run_id']
            assert recording['trace'] == rows(folder / 'trace.csv.gz'), (case, policy, 'trace')
            assert recording['events'] == rows(folder / 'events.csv.gz'), (case, policy, 'events')
    for name in ('condition_summary.csv', 'paired_comparisons.csv', 'summary.json', 'manifest.json'):
        assert hashlib.sha256((SITE / 'assets/evidence/v8' / name).read_bytes()).digest() == hashlib.sha256((DATA / ('analysis/' if name != 'manifest.json' else '') / name).read_bytes()).digest()
    for figure in document.select('figure.research-figure img'):
        assert (SITE / figure['src']).exists(), figure['src']
    for script in document.select('script[src]'):
        assert (SITE / script['src']).exists(), script['src']
    anchors = {node['id'] for node in document.select('[id]')}
    for link in document.select('[href]'):
        target = link['href']
        if target.startswith('#'):
            assert target[1:] in anchors, target
        elif not target.startswith(('http:', 'https:', 'mailto:')):
            assert (SITE / target).exists(), target
    assert not (SITE / 'assets/evidence/legacy_v7').exists()
    assert (SITE / 'assets/evidence/v8/SITL_PROTOCOL.md').exists()
    reuse = json.loads((SITE / 'assets/evidence/v8/reuse_results.json').read_text())
    assert reuse['attempts'] == 100 and reuse['matched_pairs'] == 50
    assert not reuse['field_differences']
    assert reuse == json.loads((ROOT / 'results/vecr/v8/reuse-qualified/analysis/reuse_results.json').read_text())
    result = {'static': 'PASS', 'chapters': 20, 'attempts': 1320,
              'conditions': 22, 'replay_conditions': sorted(replay['replays']),
              'source_revision': summary['source_revision']}
    if browser:
        from playwright.sync_api import sync_playwright
        content = markup
        for filename in ('style.css', 'animations.css'):
            content = re.sub(r'<link\b[^>]*href="assets/' + filename + r'"[^>]*>',
                             '<style>' + (SITE / 'assets' / filename).read_text() + '</style>', content)
        for filename in ('experiment-data.js', 'site.js', 'animations.js'):
            content = re.sub(r'<script\b[^>]*src="assets/' + filename + r'"[^>]*></script>', '', content)
        scripts = ''.join('<script>' + (SITE / 'assets' / name).read_text() + '</script>'
                          for name in ('experiment-data.js', 'site.js', 'animations.js'))
        content = content.replace('</body>', scripts + '</body>')
        chrome = Path('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome')
        with sync_playwright() as playwright:
            application = playwright.chromium.launch(executable_path=str(chrome) if chrome.exists() else None,
                                                      headless=True, args=['--no-sandbox'])
            for width in (1440, 390):
                page = application.new_page(viewport={'width': width, 'height': 880})
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.set_content(content, wait_until='load')
                page.wait_for_timeout(300)
                assert not errors, errors
                assert page.locator('.motion-atlas').count() == 8
                assert page.locator('#replayMetrics .replay-card').count() == 2
                assert page.evaluate('document.body.scrollWidth') <= width + 1
                page.locator('.motion-atlas--hero input[type="range"]').evaluate(
                    "node => { node.value = '700'; node.dispatchEvent(new Event('input', {bubbles: true})); }")
                assert '04 / 04' in page.locator('.motion-atlas--hero .motion-step').inner_text()
                page.locator('[data-scene="graph"] input[type="range"]').evaluate(
                    "node => { node.value = '550'; node.dispatchEvent(new Event('input', {bubbles: true})); }")
                assert 'EXECUTE LOCALLY' in page.locator('[data-scene="graph"] .motion-step').inner_text().upper()
                graph = page.locator('[data-scene="graph"]')
                graph.locator('input[type="range"]').evaluate(
                    "node => { node.value = '750'; node.dispatchEvent(new Event('input', {bubbles: true})); }")
                assert 'ARRIVED · survey unverified' in graph.locator('[data-el="graphAgentStatus"]').text_content()
                graph.locator('input[type="range"]').evaluate(
                    "node => { node.value = '960'; node.dispatchEvent(new Event('input', {bubbles: true})); }")
                assert 'Decide the next task' in graph.locator('[data-el="graphAppLabel"]').text_content()
                contract = page.locator('[data-scene="contract"]')
                contract.locator('input[type="range"]').evaluate(
                    "node => { node.value = '980'; node.dispatchEvent(new Event('input', {bubbles: true})); }")
                assert 'CALLBACK: ARRIVED' in contract.locator('[data-el="contractOutcome"]').text_content()
                assert 'survey incomplete' in contract.locator('.motion-summary').inner_text().lower() or \
                    'missing survey interval' in contract.locator('.motion-summary').inner_text().lower()
                contract.locator('[data-mode="exhausted"]').click()
                contract.locator('input[type="range"]').evaluate(
                    "node => { node.value = '980'; node.dispatchEvent(new Event('input', {bubbles: true})); }")
                assert 'CALLBACK: FAILED' in contract.locator('[data-el="contractOutcome"]').text_content()
                assert 'B not reached' in contract.locator('[data-el="contractOutcome"]').text_content()
                if width == 1440:
                    for name in ('N00', 'N03', 'N07', 'N11', 'N19'):
                        page.locator('#replayCase').select_option(name)
                        assert page.locator('#replayMetrics .replay-card').count() == 2
                page.close()
            application.close()
        result['browser'] = 'PASS'
    (SITE / 'validation').mkdir(exist_ok=True)
    (SITE / 'validation/digital_check.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--browser', action='store_true')
    validate(parser.parse_args().browser)
