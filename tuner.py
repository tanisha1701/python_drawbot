#!/usr/bin/env python3
"""
DrawBot Tuner
=============
GUI to tune the 5 key parameters in draw_bot.py without manually
editing the source file.

  TRIM_LEFT / TRIM_RIGHT   - fix straight-line drift
  SPIN_RATE_DEG_S          - calibrate corner angles
  V_MAX                    - real measured top speed of the robot
  V_CRUISE                 - normal driving speed (~65-70% of V_MAX)

The app reads current values from draw_bot.py on startup and writes
them back when you click Save.
"""

import os
import re
import subprocess
import sys
import tkinter as tk
from tkinter import messagebox

# When packaged with PyInstaller (--onefile), __file__ points to the temp
# extraction folder (AppData\Temp\MEI...), NOT the folder where the .exe lives.
# sys.executable always points to the actual .exe, so we use that when frozen.
if getattr(sys, "frozen", False):
    HERE = os.path.dirname(os.path.abspath(sys.executable))
else:
    HERE = os.path.dirname(os.path.abspath(__file__))

DRAWBOT = os.path.join(HERE, "draw_bot.py")

PARAMS = [
    dict(key="TRIM_LEFT", label="Trim LEFT motor", fmt="{:.2f}", min=0.70, max=1.30, step=0.01, default=1.0,
         desc="If the robot drifts RIGHT when driving straight,\nincrease above 1.0. If it drifts LEFT, decrease below 1.0."),
    dict(key="TRIM_RIGHT", label="Trim RIGHT motor", fmt="{:.2f}", min=0.70, max=1.30, step=0.01, default=1.0,
         desc="If the robot drifts LEFT when driving straight,\nincrease above 1.0. If it drifts RIGHT, decrease below 1.0."),
    dict(key="SPIN_RATE_DEG_S", label="Spin Rate  (deg/s)", fmt="{:.1f}", min=50.0, max=600.0, step=1.0, default=300.0,
         desc="Run --calibrate spin and observe 360 degree turn.\nOvershoots? Increase. Undershoots? Decrease."),
    dict(key="V_MAX", label="V_MAX  (m/s)", fmt="{:.3f}", min=0.02, max=0.80, step=0.005, default=0.20,
         desc="Measured top speed. Run: --test-pwm 255 --seconds 3\nMeasure distance traveled divided by 3 = V_MAX."),
    dict(key="V_CRUISE", label="V_CRUISE  (m/s)", fmt="{:.3f}", min=0.01, max=0.75, step=0.005, default=0.15,
         desc="Normal driving speed. Keep at 65-70% of V_MAX\nso outer wheel has headroom to speed up on curves."),
]

ACCENT = "#4A90D9"
BG     = "#1A1A2E"
CARD   = "#252540"
FG     = "#E8E8F8"
MUTED  = "#7878A8"
GREEN  = "#4CAF50"
ORANGE = "#FF8C00"


def _read_value(src, key):
    m = re.search(rf"^{key}\s*=\s*([0-9.]+)", src, re.MULTILINE)
    if m:
        return float(m.group(1))
    raise KeyError(key)


def load_params():
    with open(DRAWBOT, "r", encoding="utf-8") as f:
        src = f.read()
    return {p["key"]: _read_value(src, p["key"]) for p in PARAMS}


def save_params(values):
    with open(DRAWBOT, "r", encoding="utf-8") as f:
        src = f.read()
    for key, val in values.items():
        src = re.sub(
            rf"^({key}\s*=\s*)[0-9.]+",
            lambda m, v=val: m.group(1) + str(v),
            src, flags=re.MULTILINE,
        )
    with open(DRAWBOT, "w", encoding="utf-8") as f:
        f.write(src)


def _find_python() -> str:
    """
    In a frozen PyInstaller EXE, sys.executable is the .exe itself — not Python.
    This function finds the real python.exe by checking PATH and common locations.
    """
    if not getattr(sys, "frozen", False):
        return sys.executable          # running as a normal .py file, it's correct

    import shutil
    # Try PATH first (works if user installed Python with "Add to PATH" ticked)
    for name in ("python", "python3"):
        found = shutil.which(name)
        if found:
            return found

    # Common Windows install locations as a last resort
    common = [
        os.path.expandvars(r"%LOCALAPPDATA%\Programs\Python"),
        r"C:\Python3",
        r"C:\Python",
    ]
    for base in common:
        if os.path.isdir(base):
            for entry in sorted(os.listdir(base), reverse=True):  # newest first
                candidate = os.path.join(base, entry, "python.exe")
                if os.path.isfile(candidate):
                    return candidate

    raise RuntimeError(
        "Cannot find python.exe.\n\n"
        "Make sure Python is installed with 'Add Python to PATH' enabled,\n"
        "then re-open the tuner."
    )


def run_calibrate(mode):
    try:
        python = _find_python()
        subprocess.Popen(
            [python, DRAWBOT, "--calibrate", mode],
            creationflags=subprocess.CREATE_NEW_CONSOLE,
            cwd=HERE,
        )
    except Exception as e:
        messagebox.showerror("Launch Error", str(e))


class TunerApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DrawBot Tuner")
        self.configure(bg=BG)
        self.resizable(False, False)
        self._vars = {}
        self._evars = {}
        self._build()
        self._load()

    def _build(self):
        # ── Header ──────────────────────────────────────────────────────────
        hdr = tk.Frame(self, bg=ACCENT, pady=10)
        hdr.pack(fill="x")
        tk.Label(hdr, text="  DrawBot Tuner", font=("Segoe UI", 15, "bold"),
                 bg=ACCENT, fg="white").pack(side="left", padx=4)
        tk.Label(hdr, text="edit draw_bot.py parameters visually",
                 font=("Segoe UI", 9), bg=ACCENT, fg="#CCE4FF").pack(side="left")

        # ── Status ───────────────────────────────────────────────────────────
        self._status = tk.StringVar(value="Loading...")
        tk.Label(self, textvariable=self._status, font=("Segoe UI", 8),
                 bg=BG, fg=MUTED, anchor="w", padx=16, pady=4).pack(fill="x")

        # ── Parameter cards ──────────────────────────────────────────────────
        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=12)

        for p in PARAMS:
            card = tk.Frame(body, bg=CARD, pady=10, padx=14)
            card.pack(fill="x", pady=4)

            row1 = tk.Frame(card, bg=CARD)
            row1.pack(fill="x")
            tk.Label(row1, text=p["label"], font=("Segoe UI", 10, "bold"),
                     bg=CARD, fg=FG).pack(side="left")

            evar = tk.StringVar()
            self._evars[p["key"]] = evar
            ent = tk.Entry(row1, textvariable=evar, width=9,
                           font=("Consolas", 11, "bold"),
                           bg="#0D0D1E", fg=ACCENT, insertbackground=FG,
                           relief="flat", justify="center", bd=4)
            ent.pack(side="right")
            ent.bind("<Return>",   lambda e, k=p["key"]: self._sync_from_entry(k))
            ent.bind("<FocusOut>", lambda e, k=p["key"]: self._sync_from_entry(k))

            tk.Label(card, text=p["desc"], font=("Segoe UI", 8),
                     bg=CARD, fg=MUTED, justify="left", anchor="w").pack(
                fill="x", pady=(3, 6))

            dvar = tk.DoubleVar(value=p["default"])
            self._vars[p["key"]] = dvar
            sl = tk.Scale(card, variable=dvar,
                          from_=p["min"], to=p["max"], resolution=p["step"],
                          orient="horizontal", showvalue=False,
                          bg=CARD, troughcolor="#0D0D1E", highlightthickness=0, bd=0,
                          activebackground=ACCENT, length=520,
                          command=lambda v, k=p["key"], f=p["fmt"]: self._sync_from_slider(k, v, f))
            sl.pack()

            mm = tk.Frame(card, bg=CARD)
            mm.pack(fill="x")
            tk.Label(mm, text=p["fmt"].format(p["min"]),
                     font=("Segoe UI", 7), bg=CARD, fg=MUTED).pack(side="left")
            tk.Label(mm, text=p["fmt"].format(p["max"]),
                     font=("Segoe UI", 7), bg=CARD, fg=MUTED).pack(side="right")

        # ── Save button ──────────────────────────────────────────────────────
        tk.Frame(self, bg=MUTED, height=1).pack(fill="x", padx=12, pady=6)
        tk.Button(self, text="  Save to draw_bot.py",
                  font=("Segoe UI", 11, "bold"), bg=GREEN, fg="white",
                  relief="flat", cursor="hand2", pady=10,
                  activebackground="#388E3C", command=self._save
                  ).pack(fill="x", padx=12, pady=(0, 6))

        # ── Calibration launcher ─────────────────────────────────────────────
        cf = tk.LabelFrame(self, text="  Quick Calibration Tests  ",
                           bg=BG, fg=MUTED, font=("Segoe UI", 9),
                           padx=10, pady=8, bd=1, relief="groove")
        cf.pack(fill="x", padx=12, pady=(0, 12))

        tests = [
            ("Straight 1m",    ACCENT,  "straight"),
            ("360 Spin",        ORANGE, "spin"),
            ("Square 0.5m",    ACCENT,  "square"),
            ("Circle r=0.3m",  ACCENT,  "circle"),
        ]
        for i, (lbl, col, mode) in enumerate(tests):
            tk.Button(cf, text=lbl, font=("Segoe UI", 9, "bold"),
                      bg=col, fg="white", relief="flat", cursor="hand2",
                      padx=6, pady=6, command=lambda m=mode: run_calibrate(m)
                      ).grid(row=0, column=i, padx=4, sticky="ew")
            cf.columnconfigure(i, weight=1)

        tk.Label(cf, text="Each button opens a new terminal and runs the robot.",
                 font=("Segoe UI", 8), bg=BG, fg=MUTED).grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(4, 0))

    def _sync_from_slider(self, key, val, fmt):
        self._evars[key].set(fmt.format(float(val)))

    def _sync_from_entry(self, key):
        p = next(x for x in PARAMS if x["key"] == key)
        try:
            v = float(self._evars[key].get())
            v = max(p["min"], min(p["max"], v))
            self._vars[key].set(v)
            self._evars[key].set(p["fmt"].format(v))
        except ValueError:
            self._evars[key].set(p["fmt"].format(self._vars[key].get()))

    def _load(self):
        if not os.path.isfile(DRAWBOT):
            messagebox.showerror("Not found",
                f"draw_bot.py not found at:\n{DRAWBOT}\n\n"
                "Place tuner.exe in the same folder as draw_bot.py.")
            return
        try:
            vals = load_params()
            for p in PARAMS:
                k, f = p["key"], p["fmt"]
                v = vals.get(k, p["default"])
                self._vars[k].set(v)
                self._evars[k].set(f.format(v))
            self._status.set(f"Loaded from  {DRAWBOT}")
        except Exception as ex:
            messagebox.showerror("Load Error", str(ex))

    def _save(self):
        try:
            vals = {}
            for p in PARAMS:
                k = p["key"]
                try:
                    v = float(self._evars[k].get())
                    v = max(p["min"], min(p["max"], v))
                except ValueError:
                    v = self._vars[k].get()
                vals[k] = float(p["fmt"].format(v))
            save_params(vals)
            self._status.set("Saved!  " + "  |  ".join(f"{k}={v}" for k, v in vals.items()))
        except Exception as ex:
            messagebox.showerror("Save Error", str(ex))


if __name__ == "__main__":
    app = TunerApp()
    app.mainloop()
