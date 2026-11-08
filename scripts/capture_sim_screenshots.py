#!/usr/bin/env python3
"""End-to-end check of the simulator: start it, operate the real UI in headless Chrome, and assert what the page shows.

Saves the screenshots used in the docs. Needs: pip install playwright, and Google Chrome installed.
Usage: .venv/bin/python scripts/capture_sim_screenshots.py [output_dir]
"""
import pathlib
import re
import subprocess
import sys
import time

from playwright.sync_api import sync_playwright

PORT = 8097
OUT = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else pathlib.Path(__file__).resolve().parents[1] / 'docs/images'
results = []


def check(label, ok, detail=''):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'} {label}{'  [' + detail + ']' if detail and not ok else ''}", flush=True)


class Page:
    def __init__(self, page):
        self.page = page

    def status(self):
        return self.page.inner_text('#status')

    def table_row(self, n):
        return self.page.locator('#joint-table tr').nth(n).inner_text()

    def wait(self, text, timeout=8000):
        try:
            self.page.wait_for_function('(t) => document.querySelector("#status")?.innerText.includes(t)', arg=text, timeout=timeout)
            return True
        except Exception:
            return False

    def wait_rows(self, rows, text, timeout=8000):
        """Wait until each given table row contains text."""
        deadline = time.monotonic() + timeout / 1000
        while time.monotonic() < deadline:
            if all(text in self.table_row(n) for n in rows):
                return True
            time.sleep(0.1)
        return False

    def select(self, label, value):
        self.page.get_by_label(label, exact=True).click()
        self.page.get_by_role('option', name=value, exact=True).click()

    def set_number(self, label, value):
        """Type into the number box beside a visible slider, like a user would."""
        slider = self.page.locator(f'label:text-is("{label}"):visible').first
        box = slider.locator('xpath=ancestor::*[.//input[not(@type="hidden")]][1]').locator('input:visible').last
        box.fill(str(value))
        box.press('Enter')

    def click(self, name):
        self.page.get_by_role('button', name=name, exact=True).locator('visible=true').first.click()

    def shot(self, name):
        time.sleep(1.0)
        self.page.screenshot(path=OUT / name)


def gait_time(p):
    match = re.search(r'Time ([0-9.]+) s', p.status())
    return float(match.group(1)) if match else None


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    server = subprocess.Popen([sys.executable, '-m', 'hexapod_sim', '--port', str(PORT)], stdout=subprocess.DEVNULL)
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(channel='chrome', headless=True, args=['--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
            page = browser.new_page(viewport={'width': 1400, 'height': 1000})
            errors = []
            page.on('pageerror', lambda e: errors.append(str(e)))
            for _ in range(60):
                try:
                    page.goto(f'http://localhost:{PORT}')
                    break
                except Exception:
                    time.sleep(0.5)
            page.wait_for_selector('#status', timeout=60000)
            time.sleep(5)
            for button in page.locator('.mantine-Notification-closeButton').all():
                button.click()
            p = Page(page)

            print('--- Forward kinematics')
            check('starts at standby with no warnings', 'No warnings' in p.status())
            check('table shows 6 legs', page.locator('#joint-table tr').count() == 7)
            p.set_number('femur (°)', 55)
            check('femur slider updates the table', p.wait_rows([1], '55.0'), p.table_row(1))
            p.set_number('femur (°)', 46)
            p.set_number('tibia (°)', -100)
            check('tibia -100 shows a limit warning', p.wait('tibia -100.0° is below the limit -90.0°'), p.status())
            p.shot('mode_fk.png')
            p.set_number('tibia (°)', -31)
            check('warning clears', p.wait('No warnings'), p.status())

            print('--- Body pose IK')
            p.select('Mode', 'Body pose IK')
            check('body mode starts clean', p.wait('Feet stay fixed'), p.status())
            p.set_number('roll (°)', 8)
            p.set_number('pitch (°)', -6)
            p.set_number('z (mm)', 12)
            check('moderate pose stays clean', p.wait('No warnings'), p.status())
            p.shot('mode_body.png')
            p.set_number('z (mm)', 60)
            p.set_number('roll (°)', 30)
            check('extreme pose warns', p.wait('foot is unreachable') or p.wait('limit'), p.status())
            p.shot('warning_unreachable.png')
            p.click('Reset body pose')
            check('reset clears the warning', p.wait('No warnings'), p.status())

            print('--- Leg patterns')
            p.select('Mode', 'Leg patterns')
            p.click('Pose: laydown')
            check('laydown angles on all legs', p.wait_rows(range(1, 7), '77.0'), p.table_row(1))
            p.shot('mode_pattern.png')
            p.set_number('Move time (s)', 0.02)
            check('standby to laydown in 0.02 s warns about servo speed', p.wait('servo limit'), p.status())
            p.shot('warning_speed.png')
            p.set_number('Move time (s)', 0.2)
            check('the same move in 0.2 s is within servo speed', p.wait('No warnings'), p.status())

            print('--- Gait playback')
            p.select('Mode', 'Gait playback')
            check('gait mode loads', p.wait('Time 0.00 s'), p.status())
            p.click('Step')
            check('Step advances one sample', p.wait('Time 0.02 s'), p.status())
            p.click('Play')
            time.sleep(2.0)
            first = gait_time(p)
            time.sleep(1.0)
            second = gait_time(p)
            check('Play advances time', first is not None and second > first, f'{first} {second}')
            check('tripod at defaults is clean while playing', 'No warnings' in p.status(), p.status())
            p.shot('mode_gait.png')
            p.click('Pause')
            # The browser may still be drawing queued updates (slow here, without a GPU), so wait for time to settle.
            settled = False
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and not settled:
                first = gait_time(p)
                time.sleep(1.0)
                settled = gait_time(p) == first
            time.sleep(1.5)
            check('Pause stops time', settled and gait_time(p) == first, f'{first} {gait_time(p)}')
            p.select('Gait', 'wave')
            check('gait change restarts', p.wait('Time 0.00 s'), p.status())
            p.click('Play')
            time.sleep(1.5)
            check('over-fast command shows the limit note', 'limited by step length' in p.status(), p.status())
            p.click('Pause')
            check('no JavaScript errors', not errors, str(errors[:2]))
            browser.close()
    finally:
        server.terminate()
        server.wait(timeout=10)
    print(f'\n{sum(results)} of {len(results)} checks passed; screenshots in {OUT}')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
