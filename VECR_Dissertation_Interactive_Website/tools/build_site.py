#!/usr/bin/env python3
"""Build the offline dissertation reader from the frozen digital campaign."""
import csv
import gzip
import html
import json
import re
import shutil
from pathlib import Path

from bs4 import BeautifulSoup
from markdown_it import MarkdownIt

SITE = Path(__file__).resolve().parents[1]
ROOT = SITE.parent
DATA = ROOT / 'results/vecr/v8/primary'
ASSETS = SITE / 'assets'
EVIDENCE = ASSETS / 'evidence'
FIGURES = ASSETS / 'figures'


def csv_rows(path):
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rt', newline='') as stream:
        return list(csv.DictReader(stream))


manifest = json.loads((DATA / 'manifest.json').read_text())
summary = json.loads((DATA / 'analysis/summary.json').read_text())
conditions = csv_rows(DATA / 'analysis/condition_summary.csv')
pairs = csv_rows(DATA / 'analysis/paired_comparisons.csv')
attempts = csv_rows(DATA / 'per_attempt.csv')
assert summary['protocol'] == 'v8' and len(attempts) == 1320
assert len(conditions) == 44 and len(pairs) == 22
assert summary['source_revision'] == manifest['source_revision']
by_condition = {(r['scenario'], r['policy']): r for r in conditions}
condition_names = dict(re.findall(r'^\| (N\d\d) \| ([^|]+?) \|$',
                                  (ROOT / 'docs/vecr/SIMULATION_PROTOCOL.md').read_text(), re.M))
assert len(condition_names) == 22
inflight = [r for r in pairs if r['scenario'] in ('N07', 'N08', 'N09')]
honest = [r for r in attempts if r['scenario'] not in ('N19', 'N20')]
dishonest = [r for r in attempts if r['scenario'] in ('N19', 'N20')]
count = lambda rows, field: sum(int(row[field]) for row in rows)
assert len(honest) == 1200 and len(dishonest) == 120
assert count(honest, 'false_complete') == 0
assert count(dishonest, 'false_complete') == 120
assert count(attempts, 'reported_arrived') == 1080
assert count(attempts, 'reported_complete') == 540

# The full raw v8 evidence remains in the repository; these small copies make
# the offline reader independently inspectable.
current = EVIDENCE / 'v8'
current.mkdir(parents=True, exist_ok=True)
for source, name in [
    (DATA / 'manifest.json', 'manifest.json'),
    (DATA / 'analysis/RESULTS.md', 'RESULTS.md'),
    (DATA / 'analysis/summary.json', 'summary.json'),
    (DATA / 'analysis/condition_summary.csv', 'condition_summary.csv'),
    (DATA / 'analysis/paired_comparisons.csv', 'paired_comparisons.csv'),
    (ROOT / 'docs/vecr/METHOD.md', 'METHOD.md'),
    (ROOT / 'docs/vecr/SIMULATION_PROTOCOL.md', 'SIMULATION_PROTOCOL.md'),
    (ROOT / 'docs/vecr/SITL_PROTOCOL.md', 'SITL_PROTOCOL.md'),
    (ROOT / 'docs/vecr/DEFENSE_BRIEF.md', 'DEFENSE_BRIEF.md'),
    (ROOT / 'docs/vecr/LITERATURE.md', 'LITERATURE.md'),
    (ROOT / 'docs/vecr/EXPERIMENT_REVALIDATION.md', 'EXPERIMENT_REVALIDATION.md'),
    (ROOT / 'docs/vecr/TRACEABILITY.md', 'TRACEABILITY.md'),
    (ROOT / 'docs/vecr/FINAL_CHANGE_REPORT.md', 'FINAL_CHANGE_REPORT.md'),
    (ROOT / 'results/vecr/v8/reuse-qualified/analysis/REUSE_RESULTS.md', 'REUSE_RESULTS.md'),
    (ROOT / 'results/vecr/v8/reuse-qualified/analysis/reuse_results.json', 'reuse_results.json'),
]:
    shutil.copyfile(source, current / name)
for name in ('completion_matrix', 'paired_distance'):
    for ext in ('svg', 'png'):
        shutil.copyfile(DATA / 'analysis' / f'{name}.{ext}', FIGURES / f'{name}.{ext}')
for stale in FIGURES.glob('inflight_distance.*'):
    stale.unlink()

# A compact current-data chart fits the landing view; the complete 22-case
# paired chart remains in the measured-results chapter.
bars = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 350" role="img" aria-labelledby="effect-title effect-desc">',
        '<title id="effect-title">Measured in-flight recovery distance</title>',
        '<desc id="effect-desc">B2 and VECR mean distances in N07, N08 and N09, with paired distance reductions and confidence intervals.</desc>',
        '<rect width="720" height="350" rx="14" fill="#102d3c"/>',
        '<text x="30" y="34" fill="#e5f8f3" font-size="20" font-family="Arial" font-weight="bold">Shorter return target, measured flight cost</text>',
        '<text x="30" y="56" fill="#9bbeb8" font-size="12" font-family="Arial">Frozen digital campaign · 30 jointly successful seed pairs per case</text>']
for i, row in enumerate(inflight):
    case = row['scenario']
    b2 = float(by_condition[case, 'B2']['mean_distance_m'])
    vecr = float(by_condition[case, 'VECR']['mean_distance_m'])
    reduction = -float(row['successful_distance_delta_m'])
    low = -float(row['successful_distance_delta_m_ci_high'])
    high = -float(row['successful_distance_delta_m_ci_low'])
    y = 99 + i * 80
    case_label = {'N07': 'departure', 'N08': 'wide sensing', 'N09': 'narrow sensing'}[case]
    bars += [f'<text x="30" y="{y}" fill="#e5f8f3" font-size="15" font-family="Arial" font-weight="bold">{case} · {case_label}</text>',
             f'<rect x="200" y="{y-15}" width="{b2*4.5:.2f}" height="17" rx="3" fill="#d5a36e"/>',
             f'<rect x="200" y="{y+8}" width="{vecr*4.5:.2f}" height="17" rx="3" fill="#5ad6bf"/>',
             f'<text x="{210+b2*4.5:.2f}" y="{y-2}" fill="#f6d8b4" font-size="12" font-family="Arial">B2 {b2:.1f} m</text>',
             f'<text x="{210+vecr*4.5:.2f}" y="{y+22}" fill="#b4f1e6" font-size="12" font-family="Arial">VECR {vecr:.1f} m</text>',
             f'<text x="{200}" y="{y+43}" fill="#a8c9c3" font-size="11" font-family="Arial">Saved {reduction:.2f} m · paired 95% CI [{low:.2f}, {high:.2f}]</text>']
bars += ['<text x="30" y="337" fill="#8fb6af" font-size="11" font-family="Arial">Distance advantage is conditional on joint true and reported completion.</text>', '</svg>']
(FIGURES / 'inflight_effect.svg').write_text(''.join(bars))

# Replay rows are direct copies of recorded digital traces, not generated animation.
replays = {}
for case in ('N00', 'N03', 'N07', 'N11', 'N19'):
    replays[case] = {}
    for policy in ('B2', 'VECR'):
        run_id = f'{case}-70001-{policy}-direct'
        folder = DATA / 'trials' / run_id
        replays[case][policy] = {
            'run_id': run_id,
            'trace': csv_rows(folder / 'trace.csv.gz'),
            'events': csv_rows(folder / 'events.csv.gz'),
        }
payload = {
    'protocol': 'v8', 'source_revision': summary['source_revision'],
    'scope': 'Frozen agent-backed digital simulation; SITL evidence is separate.',
    'counts': {'attempts': len(attempts), 'conditions': 22, 'paired_seeds': 30},
    'scenarios': conditions, 'effects': pairs,
    'configs': [{'scenario': c, 'policy': p} for c in replays for p in ('B2', 'VECR')],
    'replays': replays,
}
(ASSETS / 'experiment-data.js').write_text(
    'window.VECR_DATA = ' + json.dumps(payload, separators=(',', ':'), ensure_ascii=False) + ';\n'
)


def table(headers, rows, caption):
    out = '<div class="table-scroll" role="region" tabindex="0" aria-label="Research table"><table>'
    out += f'<caption>{html.escape(caption)}</caption><thead><tr>'
    out += ''.join(f'<th>{html.escape(str(h))}</th>' for h in headers)
    out += '</tr></thead><tbody>'
    for row in rows:
        out += '<tr>' + ''.join(f'<td>{html.escape(str(v))}</td>' for v in row) + '</tr>'
    return out + '</tbody></table></div>'


condition_table = table(
    ['Case', 'Condition', 'Policy', 'Arrived / 30', 'Reported survey complete / 30',
     'True complete / 30', 'False complete', 'False negative', 'Mean distance (m)',
     'Mean unresolved (m)'],
    [[r['scenario'], condition_names[r['scenario']], r['policy'], r['arrived'], r['reported_complete'],
      r['true_complete'], r['false_complete'], r['false_negative'],
      f"{float(r['mean_distance_m']):.2f}", f"{float(r['mean_unresolved_m']):.2f}"]
     for r in conditions],
    'Frozen digital campaign; 30 common seeds per condition and policy. Dishonest controls N19/N20 are excluded from honest-sensor safety counts.',
)
paired_table = table(
    ['Case', 'Condition', 'Joint true/report complete', 'B2 mean (m)', 'VECR mean (m)',
     'VECR − B2 (m)', '95% paired bootstrap interval (m)'],
    [[r['scenario'], condition_names[r['scenario']], f"{r['joint_success_pairs']}/30",
      f"{float(by_condition[r['scenario'], 'B2']['mean_distance_m']):.2f}",
      f"{float(by_condition[r['scenario'], 'VECR']['mean_distance_m']):.2f}",
      f"{float(r['successful_distance_delta_m']):.2f}",
      f"[{float(r['successful_distance_delta_m_ci_low']):.2f}, {float(r['successful_distance_delta_m_ci_high']):.2f}]m"]
     for r in inflight],
    'Recorded N07–N09; 30 matched seed pairs per case; distance contrast among joint successful attempts.',
)
summary_html = (
    '<div class="metric-grid">'
    f'<div><strong>{len(attempts):,}</strong><span>recorded attempts; 22 cases × 30 seeds × 2 policies</span></div>'
    f'<div><strong>{count(honest, "false_complete"):,} / {len(honest):,}</strong><span>false reported completions with honest sensors</span></div>'
    f'<div><strong>{count(attempts, "reported_arrived"):,} / {len(attempts):,}</strong><span>qualified B arrivals, all controls</span></div>'
    f'<div><strong>{count(attempts, "reported_complete"):,} / {len(attempts):,}</strong><span>reported survey complete, all controls</span></div>'
    f'<div><strong>{count(dishonest, "false_complete"):,} / {len(dishonest):,}</strong><span>false reported completions under dishonest-sensor controls</span></div>'
    '</div>'
    f'<p>Honest-sensor subset: {count(honest, "reported_arrived"):,}/{len(honest):,} qualified arrivals, '
    f'{count(honest, "reported_complete"):,}/{len(honest):,} reported complete surveys, '
    f'{count(honest, "truth_complete"):,}/{len(honest):,} independently true complete surveys. '
    'Dishonest controls are reported separately.</p>'
)
formulas = {
    'identity': ('id = (mission, revision, edge, attempt, drone)', 'Stable execution identity'),
    'footprint': ('h⁻ = ĥ − εh; a⁻ = h⁻ tan(θa/2) − εp; c⁻ = h⁻ tan(θc/2) − εp; accept only if d_cross + required strip half-width ≤ c⁻; I⁻ = [ŝ − a⁻, ŝ + a⁻] ∩ [0,L]', 'Conservative footprint with the required cross-strip test; fixed calibrated FOV and valid hard bounds are assumptions'),
    'coverage': ('V(t) = ⋃ accepted I⁻j; M(t) = [0,L] ∖ V(t); unresolved = Σ length(Mi)', 'Accepted union and missing complement'),
    'recovery': ('q = clip[0,L](min(p,g)); trigger = qualified sustained departure before B', 'Single automatic local re-entry rule'),
    'arrival': ('terminal = (ARRIVED | FAILED | CANCELLED, reported survey completeness, missing intervals, reason, identity)', 'Arrival and survey coverage are distinct'),
    'traversal': ('distance(actual trajectory, route) ≤ (D0 + D1 + Vmax Δt)/2 ≤ corridor half-width', 'Conservative bounded-traversal sufficient condition'),
    'soundness': ('∀j: acceptedFootprint(j) ⊆ actualFootprint(j) ⇒ ⋃ acceptedFootprint ⊆ ⋃ actualFootprint', 'Conditional set-containment implication'),
}


FIGURE_CAPTIONS = {
    'completion_matrix': 'Reported survey completion by case and policy. Zero cells include deliberate faults and arrived attempts with unverified gaps; compare arrival and independent truth in the table.',
    'paired_distance': 'VECR minus B2 distance in frozen digital simulation. N07–N09 are the successful in-flight comparisons; intervals are pointwise paired 95% bootstrap intervals.',
    'recovery': 'In-flight target selection and physical re-entry; the terminal B result does not start another return.'
}


def figure(name, directory='diagrams'):
    caption = FIGURE_CAPTIONS.get(name, 'VECR method diagram.')
    path = f'assets/{directory}/{name}.svg'
    escaped = html.escape(caption)
    return (f'<figure class="research-figure"><button type="button" class="figure-open" '
            f'data-image="{path}" data-caption="{escaped}" aria-label="Enlarge: {escaped}">'
            f'<img loading="lazy" src="{path}" alt="{escaped}"/><span class="enlarge">Enlarge ↗</span>'
            f'</button><figcaption>{escaped}</figcaption></figure>')


choices = [('N00', 'Nominal'), ('N03', 'Endpoint evidence gap'),
           ('N07', 'In-flight departure'), ('N11', 'Command refusal'),
           ('N19', 'Dishonest sensor')]
options = ''.join(f'<option value="{c}">{c} · {html.escape(label)}</option>' for c, label in choices)
trace_html = f'''<div class="trace-explorer"><div class="trace-toolbar"><label>Recorded condition<select id="replayCase">{options}</select></label><button id="playReplay" class="button">Play replay</button><button id="resetReplay" class="button secondary">Reset</button><label>Speed<select id="replaySpeed"><option value="1">1×</option><option value="4" selected>4×</option><option value="10">10×</option></select></label></div><p class="source-note">Recorded agent-backed digital trajectories from seed 70001; full raw traces and independent checks are linked in the campaign manifest. N03 ends ARRIVED with an unresolved evidence gap and no endpoint return.</p><div class="trace-legend"><span class="legend-b2">B2 · in-flight return to A</span><span class="legend-vecr">VECR · local return</span><span>Line: recorded path · bars: agent-accepted coverage · ×: accepted target</span></div><svg id="replayPlot" viewBox="0 0 880 370" role="img" aria-labelledby="replayTitle replayDesc"><title id="replayTitle">Recorded physical trajectories</title><desc id="replayDesc">Use the time slider to compare paired recorded digital traces.</desc><g id="replayDrawing"></g></svg><label class="time-control">Simulation time <output id="replayTime">0.0 s</output><input id="replaySlider" type="range" min="100" max="100000" step="100" value="100" aria-label="Recorded simulation time"></label><div id="replayMetrics" class="replay-metrics" aria-live="off"></div><div class="trace-events"><h4>Recorded event timeline</h4><p>Click recorded phase, accepted command, or fault events.</p><div id="eventList"></div></div></div>'''

old = BeautifulSoup((SITE / 'index.htm').read_text(), 'lxml')
profile = old.select_one('.author-profile')
assert profile is not None
outcome_guide = '''<div class="outcome-guide" role="note" aria-label="How to read the outcome matrix">
<div><strong>1 · Physical arrival</strong><span>Did bounded stopped evidence establish arrival at B?</span></div>
<div><strong>2 · Survey verdict</strong><span>Did accepted evidence satisfy the requested strip tolerance?</span></div>
<div><strong>3 · Independent truth</strong><span>Did the simulator oracle reconstruct actual acquisition?</span></div>
<div><strong>4 · Fault controls</strong><span>Zero completion can be the intended safe result; dishonest-sensor cases test the trust boundary.</span></div>
</div>'''
recovery_walkthrough = '''<div class="recovery-walkthrough" role="group" aria-label="Forty metre recovery example">
<h4>A 40 m edge, explained in four steps</h4>
<div class="walk-strip" role="img" aria-label="The accepted survey covers 0 to 18 metres and 22 to 25 metres. The gap starts at 18 metres; the drone is at 25 metres.">
<span class="walk-covered" style="left:0;width:45%"></span><span class="walk-gap" style="left:45%;width:10%"></span><span class="walk-covered" style="left:55%;width:7.5%"></span>
<i class="walk-q" style="left:45%">q=18</i><i class="walk-p" style="left:62.5%">p=25</i></div>
<ol><li><strong>Accepted photos:</strong> 0–18 m and 22–25 m are credited; 18–22 m remains unverified.</li>
<li><strong>Deviation:</strong> the drone departs the corridor while progressing at 25 m, before it reaches B.</li>
<li><strong>Decision:</strong> the first gap starts at 18 m, so q=min(25,18)=18 m. A future gap would not send the drone forward.</li>
<li><strong>Physical check:</strong> after the return command, bounded position, altitude and speed plus a 200 ms sampled dwell qualify re-entry. The drone then resumes toward B.</li></ol>
<p>Green = accepted survey evidence; amber = unverified interval. Arrival at B later has a separate survey verdict.</p></div>'''
subs = {'author_profile': str(profile), 'primary_summary': summary_html,
        'primary_table': condition_table, 'primary_paired_table': paired_table,
        'trace_explorer': trace_html, 'outcome_guide': outcome_guide,
        'recovery_walkthrough': recovery_walkthrough}


def placeholder(match):
    key = match.group(1)
    if key.startswith('diagram:'):
        return figure(key.split(':', 1)[1])
    if key.startswith('figure:'):
        return figure(key.split(':', 1)[1], 'figures')
    if key.startswith('equation:'):
        value, caption = formulas[key.split(':', 1)[1]]
        return f'<div class="equation" role="group" aria-label="{html.escape(caption)}"><div>{html.escape(value)}</div><small>{html.escape(caption)}</small></div>'
    return subs[key]


def headings(fragment):
    def replace(match):
        level = min(6, int(match.group(1)) + 1)
        inner = match.group(2)
        anchor = re.search(r'\s*\{#([a-z0-9-]+)\}\s*$', inner)
        if anchor:
            inner, slug = inner[:anchor.start()], anchor.group(1)
        else:
            slug = re.sub(r'[^a-z0-9]+', '-', BeautifulSoup(inner, 'html.parser').get_text().lower()).strip('-')
        return f'<h{level} id="{slug}">{inner}</h{level}>'
    return re.sub(r'<h([23])>(.*?)</h\1>', replace, fragment, flags=re.S)


source = (SITE / 'source/dissertation.md').read_text()
chunks = re.split(r'(?=^# \d\d / )', source, flags=re.M)
markdown = MarkdownIt('commonmark', {'html': True, 'linkify': True}).enable('table')
sections, nav = [], []
for chunk in chunks:
    if not chunk.startswith('# '):
        continue
    title, body = chunk.split('\n', 1)
    match = re.match(r'# (\d\d) / (.*?) \{#([a-z0-9-]+)\}', title)
    assert match, title
    number, label, ident = match.groups()
    nav.append((number, label, ident))
    body = re.sub(r'\{\{([^}]+)\}\}', placeholder, body)
    def readable_math(match):
        expression = match.group(1)
        for command, replacement in ((r'\bigcup', '⋃'), (r'\setminus', '∖'),
                                     (r'\inf', 'inf'), (r'\min', 'min'), (r'\to', '→'),
                                     (r'\in', '∈'), (r'\le', '≤')):
            expression = expression.replace(command, replacement)
        return '<span class="inline-math">' + html.escape(expression) + '</span>'
    body = re.sub(r'\$([^$\n]+)\$', readable_math, body)
    rendered = headings(markdown.render(body))
    section = BeautifulSoup(f'<section class="chapter" aria-labelledby="{ident}"><h2 id="{ident}">{number} / {html.escape(label)}</h2>{rendered}</section>', 'lxml').select_one('section')
    sections.append(section)
assert len(sections) == 20
main = old.select_one('main#manuscript')
assert main is not None
for element in list(main.select(':scope > section.chapter')):
    element.decompose()
for duplicate in list(main.find_all('footer', recursive=False))[1:]:
    duplicate.decompose()
for element in sections:
    main.append(element)
footer = main.find('footer', recursive=False)
if footer:
    paragraph = footer.find('p')
    if paragraph:
        paragraph.string = 'Frozen digital evidence · explicit assumptions · reproducible records'

hero_figure = old.select_one('.hero-figure')
if hero_figure:
    hero_figure.clear()
    hero_figure.append(BeautifulSoup('<div class="hero-evidence"><p class="eyebrow">FROZEN DIGITAL CAMPAIGN</p><img src="assets/figures/inflight_effect.svg" alt="Measured N07 to N09 VECR and B2 mean flight distances with paired confidence intervals"><p class="source-note">1,320 agent-backed attempts · 22 conditions · SITL numbers pending.</p></div>', 'lxml').select_one('.hero-evidence'))
old.select_one('.hero-summary').string = 'A tested UAV edge-execution runtime: detect sustained in-flight deviation, recover within finite bounds, and report qualified arrival and missing survey evidence separately.'
old.select_one('.status-pill').string = 'DIGITAL VALIDATION COMPLETE · SITL PENDING'
old.select_one('.sidebar-intro p').string = 'Implemented method. Frozen digital evidence. Explicit limits.'
old.select_one('.sidebar-bottom small').string = 'Revised-method SITL: TBD.'
for node in old.select('.sidebar-bottom'):
    for text_node in list(node.find_all(string=True, recursive=False)):
        if 'Revised specification' in text_node:
            text_node.replace_with(' Digital validation complete')
for text_node in old.select_one('.hero-meta').find_all(string=True):
    if 'REVISED RESEARCH SPECIFICATION' in text_node or 'DIGITAL VALIDATION V8' in text_node:
        text_node.replace_with('SWARMKIT / VECR · DIGITAL VALIDATION')
cards = old.select('.hero-stats > div')
assert len(cards) == 4
cards[2].select_one('strong').string = '100'
cards[2].select_one('span').string = 'validated client-path attempts'
cards[3].select_one('strong').string = '0'
cards[3].select_one('span').string = 'false reported complete / 1,200 honest trials*'
edition = old.select_one('.edition-note')
if edition:
    edition.clear()
    for span in BeautifulSoup('<span>READER EDITION · 29 OCT 2026</span><span>Frozen agent-backed digital campaign ✓   /   Revised-method SITL TBD</span>', 'html.parser').find_all('span'):
        edition.append(span)
old.select_one('meta[name="description"]')['content'] = 'VECR dissertation: implemented UAV edge execution, bounded in-flight recovery, and frozen agent-backed digital validation.'
old.title.string = 'VECR — Verified Edge Coverage and Recovery · Dissertation Reader'
chapter_nav = old.select_one('.chapter-nav')
chapter_nav.clear()
for number, label, ident in nav:
    link = old.new_tag('a', href='#' + ident)
    number_span = old.new_tag('span')
    number_span.string = number
    link.append(number_span)
    link.append(label)
    chapter_nav.append(link)
for link in old.select('.hero-actions .text-link'):
    link.string = 'Inspect measured results →'
top_nav = old.select_one('.top-nav')
top_nav.clear()
for label, ident in [('Researcher', 'researcher'), ('Contribution', 'innovation'),
                     ('SDK', 'swarmkit'), ('Method', 'model'),
                     ('Results', 'results'), ('Replay', 'replay')]:
    link = old.new_tag('a', href='#' + ident)
    link.string = label
    top_nav.append(link)
for notice in main.select('.edition-notice'):
    notice.decompose()
notice = BeautifulSoup('<div class="edition-notice" role="note"><strong>Evidence status:</strong> The frozen digital campaign and independent validator are complete. ArduPilot SITL numbers for this revised method are TBD. Retired datasets are excluded from the current checkout and estimates.</div>', 'lxml').select_one('.edition-notice')
main.select_one('.hero').insert_after(notice)
(SITE / 'index.htm').write_text('<!doctype html>\n' + str(old.html))
metadata = ASSETS / 'research-metadata.json'
author = json.loads(metadata.read_text())['author']
meta = {'title': 'VECR — Verified Edge Coverage and Recovery',
        'edition_date': '2026-10-29', 'author': author,
        'research_status': 'Frozen agent-backed digital campaign validated; revised-method SITL pending.',
        'digital': {'protocol': 'v8', 'attempts': 1320, 'conditions': 22,
                    'source_revision': summary['source_revision'],
                    'binary_sha256': manifest['binary_sha256'],
                    'manifest': 'assets/evidence/v8/manifest.json',
                    'raw': 'results/vecr/v8/primary/'},
        'reuse': {'attempts': 100, 'matched_pairs': 50,
                  'source_revision': json.loads((ROOT / 'results/vecr/v8/reuse-qualified/manifest.json').read_text())['source_revision'],
                  'report': 'assets/evidence/v8/REUSE_RESULTS.md'},
        'sitl_revised': 'TBD'}
metadata.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n')
print(f'Built {len(sections)} chapters, {len(attempts)} attempts, {len(replays)} recorded replay conditions')
