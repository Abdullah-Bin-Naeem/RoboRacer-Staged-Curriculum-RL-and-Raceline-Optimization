"""End-to-end check of the line editor in headless Chrome.

    ../.venv-rl/bin/python line_editor/test_editor.py [--shots DIR]

Starts the server on a spare port and drives the page with Selenium through
both views: drag a point, smooth the edited region, live recompute during a
drag, undo; then in the speed view paint and remove a zone, change v_max,
draw a stroke, smooth and erase under the brush, bump a range, toggle the
budget re-sweep, save with edits, reload, clear. Asserts on the editor's state
after each step. Needs selenium in .venv-rl and a Chrome binary; Selenium
Manager fetches a matching chromedriver. Test files are written under
raceline/icra2026/ as _editor_test* and removed at the end.
"""
import argparse
import json
import socket
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.action_chains import ActionChains
from selenium.webdriver.common.by import By
from selenium.webdriver.common.keys import Keys
from selenium.webdriver.support.ui import WebDriverWait

TOOL_DIR = Path(__file__).resolve().parent
RACELINE = TOOL_DIR.parent
PORT = 8799
TRACK = "icra2026"
LINE = "raceline_a7.0zv_hard_l6.0_corners_h.csv"
TEST = "_editor_test"


def sweep(d, el, pts):
    """Press at the first (x, y) canvas point, move through the rest, release."""
    w, h = el.size["width"], el.size["height"]
    ac = ActionChains(d).move_to_element_with_offset(el, round(pts[0][0] - w / 2), round(pts[0][1] - h / 2)).click_and_hold()
    last = pts[0]
    for p in pts[1:]:
        ac = ac.move_by_offset(round(p[0] - last[0]), round(p[1] - last[1])); last = p
    ac.release().perform()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--shots", type=Path, default=None, help="directory for screenshots along the way")
    a = ap.parse_args()
    if a.shots:
        a.shots.mkdir(parents=True, exist_ok=True)
    shot = (lambda d, name: d.save_screenshot(str(a.shots / name))) if a.shots else (lambda d, name: None)

    srv = subprocess.Popen([sys.executable, "-m", "line_editor", "--track", TRACK, "--line", LINE,
                            "--port", str(PORT), "--no-browser"], cwd=RACELINE,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    for _ in range(150):
        try:
            socket.create_connection(("127.0.0.1", PORT), timeout=0.2).close()
            break
        except OSError:
            time.sleep(0.2)

    opts = Options()
    for arg in ("--headless=new", "--no-sandbox", "--disable-gpu", "--window-size=1600,1000"):
        opts.add_argument(arg)
    opts.set_capability("goog:loggingPrefs", {"browser": "ALL"})
    d = webdriver.Chrome(options=opts)
    ok = True
    try:
        d.get(f"http://127.0.0.1:{PORT}/")
        W = WebDriverWait(d, 30)
        js = d.execute_script
        W.until(lambda d: js("return !!(window.editor && editor.S.prof && editor.S.map)"))
        time.sleep(0.5)
        idle = lambda d: js("return !editor.S.busy && !editor.S.fastBusy")  # noqa: E731

        def after(fn):
            """Run fn (an action) and wait for the recompute it triggers."""
            req = js("return editor.S.req"); fn()
            W.until(lambda d: js(f"return !editor.S.busy && editor.S.req > {req}"))

        print("status:", js("return document.getElementById('status').textContent"))
        print("note:  ", js("return document.getElementById('note').textContent"))
        t0, tf = js("return [editor.S.prof.t, editor.S.t_file]")
        assert abs(t0 - tf) < 0.02, (t0, tf)
        shot(d, "v2_map.png")

        # 1. drag point 100 by (20, 10) px with the 1.5 m brush; neighbour 105 (0.5 m away) gets weight (1-1/9)^2
        m = d.find_element(By.ID, "map")
        px, py = js("return editor.screenOf(100)")
        xb, yb, x5b, scale = js("return [editor.S.x[100], editor.S.y[100], editor.S.x[105], editor.mapView.scale]")
        after(lambda: sweep(d, m, [(px, py), (px + 10, py), (px + 20, py + 10)]))
        xa, ya, x5a = js("return [editor.S.x[100], editor.S.y[100], editor.S.x[105]]")
        w5 = (1 - (0.5 / 1.5) ** 2) ** 2
        print(f"drag: dx {xa - xb:+.4f} (want {20 / scale:+.4f})  dy {ya - yb:+.4f} (want {-10 / scale:+.4f})  "
              f"neighbour dx {x5a - x5b:+.4f} (want ~{w5 * 20 / scale:+.4f})")
        assert abs((xa - xb) - 20 / scale) < 0.03 and abs((ya - yb) + 10 / scale) < 0.03
        assert abs((x5a - x5b) - w5 * 20 / scale) < 0.03
        n_region = js("return editor.S.region.filter(w => w > 0).length")
        print(f"lap after drag {js('return editor.S.prof.t'):.3f} (was {t0:.3f}); edited region {n_region} points")
        assert 20 <= n_region <= 40

        # 2. smooth the edited region: the kink around the drag softens, the line far from it does not move
        near = "return [Math.max(...editor.S.prof.kappa.slice(60, 141).map(Math.abs)), editor.S.x.filter((_, i) => i < 40 || i > 160)]"
        k_in0, x_out0 = js(near)
        after(lambda: d.find_element(By.ID, "g_smooth").click())
        k_in1, x_out1 = js(near)
        far = max(abs(a - b) for a, b in zip(x_out0, x_out1))
        print(f"region smoothing: |kappa| max around the drag {k_in0:.2f} -> {k_in1:.2f}; far points moved {far:.1e}")
        assert k_in1 < 0.8 * k_in0 and far < 1e-9
        shot(d, "v2_map_region.png")

        # 3. live recompute during a drag: fast requests happen while the mouse moves
        d.find_element(By.ID, "live").click()
        px, py = js("return editor.screenOf(300)")
        after(lambda: sweep(d, m, [(px, py), (px + 4, py), (px + 8, py), (px + 12, py)]))
        W.until(idle)
        fast = js("return editor.S.fastCount")
        print(f"live: {fast} fast recomputes during the drag")
        assert fast >= 1
        d.find_element(By.ID, "live").click()

        # 4. undo the three geometry steps: back to the loaded line, region empty
        for _ in range(3):
            js("editor.undo()"); time.sleep(0.1); W.until(idle)
        xr, nr = js("return [editor.S.x[100], editor.S.region.filter(w => w > 0).length]")
        print(f"after 3 undos: x[100] {xr:.5f} (loaded {xb:.5f}), region {nr}")
        assert abs(xr - xb) < 1e-9 and nr == 0

        # 5. speed view
        js("editor.setView('speed')"); time.sleep(0.3)
        c = d.find_element(By.ID, "speed")
        assert c.is_displayed() and js("return editor.S.view") == "speed"
        shot(d, "v2_speed.png")

        # 6. zone tool: paint a speed zone s 12-16 at 6 m/s, then shift-click it away
        js("editor.setTool('zone')")
        ax, ay = js("return editor.speedXY(12, 6, 'v')"); bx, by = js("return editor.speedXY(16, 6, 'v')")
        sweep(d, c, [(ax, ay), ((ax + bx) / 2, ay), (bx, ay)])
        W.until(lambda d: js("return !editor.S.busy && editor.S.params.v_zones.length == 2"))
        vz = js("return editor.S.params.v_zones")
        i14 = js("return editor.S.prof.s.findIndex(s => s >= 14)")
        v14 = js(f"return editor.S.prof.v[{i14}]")
        print(f"painted zone {vz[-1]}; v at s=14: {v14:.2f}")
        assert abs(vz[-1][0] - 12) < 0.5 and abs(vz[-1][1] - 16) < 0.5 and abs(vz[-1][2] - 6) < 0.15 and v14 <= 6.05
        cw, ch = c.size["width"], c.size["height"]
        (ActionChains(d).key_down(Keys.SHIFT).move_to_element_with_offset(c, round((ax + bx) / 2 - cw / 2), round(ay - ch / 2))
         .click().key_up(Keys.SHIFT).perform())
        W.until(lambda d: js("return !editor.S.busy && editor.S.params.v_zones.length == 1"))
        print("shift-click removed the zone")

        # 7. v_max 6.5 through the input, then undo the three profile steps
        js("const e = document.getElementById('v_max'); e.value = '6.5'; e.dispatchEvent(new Event('change'))")
        W.until(lambda d: js("return !editor.S.busy && editor.S.params.v_max == 6.5"))
        vmax_out = js("return Math.max(...editor.S.prof.v.filter((v, i) => editor.S.prof.s[i] < 7.5 || editor.S.prof.s[i] > 18.5))")
        vmax_in = js("return Math.max(...editor.S.prof.v)")
        print(f"v_max 6.5: max outside the zone {vmax_out:.2f}, inside {vmax_in:.2f}")
        assert vmax_out <= 6.51 and vmax_in > 8.5
        for _ in range(3):
            js("editor.undo()"); time.sleep(0.1); W.until(idle)
        assert js("return editor.S.params.v_max == 7 && editor.S.params.v_zones.length == 1")

        # 8. draw tool: a stroke from (s 30, 6.0) to (s 34, 6.5), smoothed on release
        js("editor.setTool('draw')")
        ax, ay = js("return editor.speedXY(30, 6.0, 'v')"); bx, by = js("return editor.speedXY(34, 6.5, 'v')")
        after(lambda: sweep(d, c, [(ax, ay), ((ax + bx) / 2, (ay + by) / 2), (bx, by)]))
        i32 = js("return editor.S.prof.s.findIndex(s => s >= 32)")
        v32, vp32, dv32, n_ed = js(f"return [editor.S.prof.v[{i32}], editor.S.prof.v_phys[{i32}], editor.S.dv[{i32}], "
                                   "editor.S.dv.filter(d => d != 0).length]")
        print(f"drawn: at s=32 v {v32:.2f} (profile {vp32:.2f}, edit {dv32:+.2f}); {n_ed} points edited after stroke smoothing")
        assert abs(v32 - 6.25) < 0.2 and abs(v32 - (vp32 + dv32)) < 0.01 and 35 <= n_ed <= 80
        js(f"editor.S.hover = {i32}; editor.readout({i32}); editor.draw()")
        shot(d, "v2_speed_drawn.png")

        # 8b. a stroke across the start line: from s = L-1.5 through the wrap margin to s = 1.5
        L = js("return editor.S.prof.length")
        ax, ay = js(f"return editor.speedXY({L - 1.5}, 5.0, 'v')"); bx, by = js(f"return editor.speedXY({L + 1.5}, 5.0, 'v')")
        after(lambda: sweep(d, c, [(ax, ay), ((ax + bx) / 2, ay), (bx, ay)]))
        head, tail = js("const n = editor.S.dv.length; return [editor.S.dv.slice(0, 12).filter(v => v != 0).length, editor.S.dv.slice(n - 12).filter(v => v != 0).length]")
        print(f"stroke across the start line: {tail} edited points before s=0, {head} after")
        assert head >= 8 and tail >= 8

        # 8c. the minimap: hovering a point there moves the chart cursor to it
        mm = d.find_element(By.ID, "minimap"); mw, mh = mm.size["width"], mm.size["height"]
        qx, qy = js("return editor.miniOf(200)")
        ActionChains(d).move_to_element_with_offset(mm, round(qx - mw / 2), round(qy - mh / 2)).perform()
        hov = js("return editor.S.hover")
        print(f"minimap hover -> chart cursor at index {hov} (s {js('return editor.S.prof.s[editor.S.hover]'):.1f})")
        assert abs(hov - 200) <= 2
        shot(d, "v2_speed_minimap.png")

        # 9. smooth tool: a noisy edit over s 20-24 gets smoother under the brush
        js("const p = editor.S.prof; p.s.forEach((s, i) => { if (s >= 20 && s <= 24) editor.S.dv[i] = (i % 2 ? 0.3 : -0.3); }); "
           "editor.localUpdate(); editor.draw()")
        rough = "return editor.S.dv.reduce((a, d, i) => (editor.S.prof.s[i] >= 19 && editor.S.prof.s[i] <= 25 && i) ? a + Math.abs(d - editor.S.dv[i - 1]) : a, 0)"
        r0 = js(rough)
        js("const e = document.getElementById('s_strength'); e.value = '1'; e.dispatchEvent(new Event('input'))")
        js("editor.setTool('smooth')")
        ax, ay = js("return editor.speedXY(20, 5, 'v')"); bx, by = js("return editor.speedXY(24, 5, 'v')")
        after(lambda: sweep(d, c, [(ax + k * (bx - ax) / 8, ay) for k in range(9)]))
        r1 = js(rough)
        print(f"smooth brush: roughness over s 19-25 {r0:.2f} -> {r1:.2f}")
        assert r1 < 0.5 * r0

        # 10. erase tool over the same span: the edits there go away
        m0 = js("return editor.S.dv.filter((d, i) => editor.S.prof.s[i] >= 20.5 && editor.S.prof.s[i] <= 23.5).reduce((a, d) => a + Math.abs(d), 0)")
        js("editor.setTool('erase')")
        after(lambda: sweep(d, c, [(ax + k * (bx - ax) / 8, ay) for k in range(9)]))
        m1 = js("return editor.S.dv.filter((d, i) => editor.S.prof.s[i] >= 20.5 && editor.S.prof.s[i] <= 23.5).reduce((a, d) => a + Math.abs(d), 0)")
        print(f"erase brush: sum |edit| over s 20.5-23.5 {m0:.2f} -> {m1:.2f}")
        assert m1 < 0.35 * m0

        # 11. numeric bump: +0.5 over s 40-42, an exact step; re-imposing the budgets ramps its leading edge
        for k, val in (("e_s0", "40"), ("e_s1", "42"), ("e_dv", "0.5")):
            js(f"document.getElementById('{k}').value = '{val}'")
        after(lambda: d.find_element(By.ID, "e_apply").click())
        i40 = js("return editor.S.prof.s.findIndex(s => s >= 40)")
        step = js(f"return editor.S.prof.v[{i40}] - editor.S.prof.v_phys[{i40}]")
        st = js("return document.getElementById('status').textContent")
        print(f"bump: first edited point v - profile = {step:+.3f}; status: {st[:90]}")
        assert abs(step - 0.5) < 1e-4
        after(lambda: d.find_element(By.ID, "resweep").click())
        ramp = js(f"return editor.S.prof.v[{i40}] - editor.S.prof.v_phys[{i40}]")
        after(lambda: d.find_element(By.ID, "resweep").click())
        back = js(f"return editor.S.prof.v[{i40}] - editor.S.prof.v_phys[{i40}]")
        print(f"re-imposed: leading edge {ramp:+.3f}; unchecked again: {back:+.3f}")
        assert ramp < 0.45 and abs(back - 0.5) < 1e-4

        # 12. save with edits, reload through the sidecar, clear
        js(f"document.getElementById('name').value = '{TEST}'; editor.save(false)")
        W.until(lambda d: "saved" in js("return document.getElementById('saved').textContent"))
        csv = RACELINE / TRACK / f"{TEST}.csv"; meta = json.loads(csv.with_name(f"{TEST}.edit.json").read_text())
        D = np.loadtxt(csv, delimiter=",")
        n_ed = js("return editor.S.dv.filter(d => d != 0).length")
        print(f"save: {D.shape}, {meta['edited_points']} edited points recorded ({n_ed} in the page), args {meta['args']}")
        assert D.shape == (541, 8) and meta["edited_points"] == n_ed
        t_saved = js("return editor.S.prof.t")
        js(f"editor.load('{TEST}.csv')")
        W.until(lambda d: js(f"return editor.S.name == '{TEST}.csv' && !editor.S.busy"))
        t_re, tf_re, n_re = js("return [editor.S.prof.t, editor.S.t_file, editor.S.dv.filter(d => d != 0).length]")
        print(f"reloaded: {t_re:.3f} s, file {tf_re:.3f} s, {n_re} edited points")
        assert abs(t_re - tf_re) < 0.005 and abs(t_re - t_saved) < 0.005 and n_re == n_ed
        after(lambda: d.find_element(By.ID, "e_clear").click())
        assert js("return editor.S.dv.every(d => d == 0) && editor.S.prof.v.every((v, i) => Math.abs(v - editor.S.prof.v_phys[i]) < 1e-9)")
        print("clear edits: profile back to the physics")

        sev = [entry["message"] for entry in d.get_log("browser") if entry["level"] == "SEVERE"]
        print("console SEVERE:", sev or "none")
        assert not sev
    except Exception:  # noqa: BLE001
        ok = False
        shot(d, "v2_fail.png")
        traceback.print_exc()
    finally:
        d.quit(); srv.terminate()
        out = srv.stdout.read()
        if out.strip():
            print("--- server log tail ---"); print(out[-1500:])
        for f in (RACELINE / TRACK).glob(f"{TEST}*"):
            f.unlink()
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
