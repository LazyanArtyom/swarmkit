#!/usr/bin/env python3
"""Check the local reader's evidence, links and optional isolated browser behavior."""

import argparse
import csv
import hashlib
import json
import re
import zipfile
from collections import Counter
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

SITE = Path(__file__).resolve().parents[1]


class Document(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []
        self.links = []
        self.resources = []
        self.text = []

    def handle_starttag(self, tag, attrs):
        a = dict(attrs)
        if "id" in a:
            self.ids.append(a["id"])
        if tag == "a" and "href" in a:
            self.links.append(a["href"])
        if tag in ("script", "img", "link"):
            resource = a.get("src", a.get("href"))
            if resource:
                self.resources.append(resource)

    def handle_data(self, text):
        self.text.append(text)


def validate():
    doc = Document()
    doc.feed((SITE / "index.htm").read_text())
    assert not [k for k, n in Counter(doc.ids).items() if n > 1], "duplicate IDs"
    for link in doc.links + doc.resources:
        u = urlsplit(link)
        if u.scheme in ("https", "http", "mailto"):
            continue
        if not u.path and u.fragment:
            assert unquote(u.fragment) in doc.ids, link
        elif u.path:
            assert (SITE / unquote(u.path)).exists(), link
    assert all(
        not urlsplit(x).scheme for x in doc.resources
    ), "remote runtime dependency"
    text = " ".join(doc.text)
    assert "We developed and evaluated" in text
    assert not re.search(
        r"we propose|manuscript proposes|the proposed method|theorem candidate",
        text,
        re.I,
    )
    assert "Not yet executed" in text and "TBD" in text
    assert not re.search(r"\{\{[^}]+\}\}", text), "unexpanded component"
    metadata = json.loads((SITE / "assets/research-metadata.json").read_text())
    with zipfile.ZipFile(SITE / "assets/evidence/representative-traces.zip") as zipped:
        for name, info in metadata["copied_evidence"].items():
            content = (
                zipped.read(name.split("!/", 1)[1])
                if "!/" in name
                else (SITE / name).read_bytes()
            )
            assert hashlib.sha256(content).hexdigest() == info["sha256"], name
    js = (SITE / "assets/experiment-data.js").read_text()
    data = json.loads(js.removeprefix("window.VECR_DATA = ").strip().removesuffix(";"))
    assert len(data["scenarios"]) == 78 and len(data["configs"]) == 39
    assert all(set(pair) == {"B2", "P"} for pair in data["replays"].values())
    assert len(data["replays"]) == 6
    for pair in data["replays"].values():
        for run in pair.values():
            trace = run["trace"]
            assert trace and len({r["time_ms"] for r in trace}) == len(trace)
            assert all(
                int(a["time_ms"]) < int(b["time_ms"]) for a, b in zip(trace, trace[1:])
            )
    result = {
        "static": "PASS",
        "chapters": 16,
        "policy_rows": 78,
        "replayed_attempts": 12,
        "copied_artifacts_verified": len(metadata["copied_evidence"]),
        "external_runtime_resources": 0,
    }
    return data, result


def browser_checks(data, result):
    from playwright.sync_api import sync_playwright

    screenshots = SITE / "validation/screenshots"
    screenshots.mkdir(exist_ok=True)
    with sync_playwright() as w:
        browser = w.chromium.launch(
            executable_path="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
            headless=True,
            args=["--disable-background-networking", "--no-first-run"],
        )
        page = browser.new_page(
            viewport={"width": 1440, "height": 1100},
            device_scale_factor=1,
            reduced_motion="reduce",
        )
        errors = []
        requests = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("request", lambda r: requests.append(r.url))
        page.goto((SITE / "index.htm").as_uri(), wait_until="load")
        assert page.locator(".chapter").count() == 16
        assert page.evaluate("document.documentElement.scrollWidth <= innerWidth")
        page.screenshot(path=str(screenshots / "desktop.png"))
        for name in ["contract", "architecture", "states", "comparison", "evidence"]:
            diagram = page.locator(f'img[src="assets/diagrams/{name}.svg"]')
            diagram.scroll_into_view_if_needed()
            diagram.evaluate("el => el.decode()")
            diagram.screenshot(
                path=str(screenshots / f"diagram-{name}.png"),
                style=".site-header,.read-progress,.skip-link{visibility:hidden!important}",
            )
        page.locator("#scenarioGroup").select_option("controls")
        assert page.locator("#scenarioTable tbody tr:visible").count() == 2
        page.locator("#scenarioGroup").select_option("sensitivity")
        assert page.locator("#scenarioTable tbody tr:visible").count() == 18
        page.locator("#scenarioSearch").fill("X07")
        assert page.locator("#scenarioTable tbody tr:visible").count() == 1
        page.locator("[data-case=X07]").click()
        assert "0.00%" in page.locator("#caseDetails").inner_text()
        page.locator("#scenarioSearch").fill("")
        page.locator("#scenarioGroup").select_option("all")
        for case in ["M03", "M04", "M05", "M08", "M14", "M17"]:
            page.locator("#replayCase").select_option(case)
            page.locator("#replaySlider").focus()
            page.keyboard.press("End")
            cards = page.locator(".replay-card")
            for i, policy in enumerate(["B2", "P"]):
                final = data["replays"][case][policy]["trace"][-1]
                assert final["phase"] in cards.nth(i).inner_text()
                assert (
                    f"{float(final['distance_m']):,.2f} m" in cards.nth(i).inner_text()
                )
            if case == "M14":
                assert "100.0%" in cards.nth(1).inner_text()
                truth = (
                    float(data["replays"][case]["P"]["trace"][-1]["truth_covered_m"])
                    / 40
                    * 100
                )
                assert f"{truth:.1f}%" in cards.nth(1).inner_text() and truth < 100
        page.locator("#replayCase").select_option("M03")
        page.locator("#eventList button").filter(has_text="RECOVERING").first.click()
        assert "RECOVERING" in page.locator("#replayMetrics").inner_text()
        page.locator(".trace-explorer").screenshot(
            path=str(screenshots / "replay.png"),
            style=".site-header,.read-progress,.skip-link{visibility:hidden!important}",
        )
        page.locator("#resetReplay").click()
        page.locator("#playReplay").click()
        page.wait_for_function("document.querySelector('#replaySlider').value > 200")
        page.locator("#playReplay").click()
        assert page.locator("#playReplay").inner_text() == "Play replay"
        page.locator(".figure-open").first.click()
        assert page.locator("#figureDialog").is_visible()
        page.keyboard.press("Escape")
        assert not page.locator("#figureDialog").is_visible()
        assert page.locator(".figure-open").first.evaluate(
            "el => el === document.activeElement"
        )
        assert page.evaluate("document.body.style.overflow") != "hidden"
        page.locator("#chapterSearch").fill("400")
        assert page.locator("#searchResults a").count() > 0
        page.locator("#searchResults a").first.click()
        assert not page.locator("#searchResults").is_visible()
        page.locator("#themeToggle").click()
        assert page.locator("html").get_attribute("data-theme") == "dark"
        page.locator("#results").scroll_into_view_if_needed()
        page.screenshot(path=str(screenshots / "results-dark.png"))
        page.locator("#themeToggle").click()
        for width in [390, 720, 1024]:
            page.set_viewport_size({"width": width, "height": 844})
            assert page.evaluate(
                "document.documentElement.scrollWidth <= innerWidth"
            ), f"overflow at {width}"
        page.set_viewport_size({"width": 390, "height": 844})
        page.evaluate("window.scrollTo({top:0,behavior:'instant'})")
        page.screenshot(path=str(screenshots / "mobile.png"))
        page.locator("#mobileToc").click()
        assert page.locator("#sidebar").is_visible()
        page.locator('.chapter-nav a[href="#sitl"]').click()
        assert not page.locator("#sidebar").is_visible()
        assert page.locator("#mobileToc").get_attribute("aria-expanded") == "false"
        page.locator(".trace-explorer").screenshot(
            path=str(screenshots / "mobile-replay.png"),
            style=".site-header,.read-progress,.skip-link{visibility:hidden!important}",
        )
        page.emulate_media(media="print")
        assert page.locator("#sidebar").is_hidden()
        assert page.locator(".chapter").count() == 16
        assert all(
            not urlsplit(u).scheme.startswith("http") for u in requests
        ), requests
        assert not errors, errors
        result.update(
            browser="PASS",
            viewports=[390, 720, 1024, 1440],
            javascript_errors=errors,
            remote_page_requests=0,
            checks=[
                "filters",
                "condition intervals",
                "all six replay outcomes",
                "negative-control truth discrepancy",
                "event seek",
                "play/pause",
                "dialog Escape/focus",
                "search",
                "theme",
                "mobile navigation",
                "print media",
            ],
        )
        browser.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--browser", action="store_true")
    a = p.parse_args()
    data, result = validate()
    if a.browser:
        browser_checks(data, result)
    (SITE / "validation/site_validation.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    print(json.dumps(result, indent=2))
