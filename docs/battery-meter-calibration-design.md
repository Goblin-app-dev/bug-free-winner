# Battery Meter Accuracy Improvement Design (TENSTAR T-Display ESP32)

## 1) Goal and scope
Improve displayed battery percentage so it **tracks real-world remaining runtime** for this device under representative load, while preserving low-power behavior and current UI cadence.

> Important framing: with voltage-only data (no current integration / coulomb counting), this design targets **runtime-percent under a known usage profile**, not true electrochemical SOC.

This design assumes:
- No dedicated charger-status GPIO is available.
- Charging detection remains voltage-heuristic based.
- Calibration data available per sample:
  - `raw_adc` (averaged)
  - `adc_mv` from `esp_adc_cal_raw_to_voltage`
  - `battery_volts = adc_mv * 0.001 * BAT_DIVIDER`
  - timestamp
  - charging / not-charging flag

---

## 2) Data collection plan (tomorrow)
Minimum viable: 5–8 points spread across discharge.
Recommended: continuous logging over a full discharge run, then downsample to anchors.

## 2.1 Required fields per sample
For each measurement point, capture:
1. Raw ADC reading (average)
2. ADC millivolts from `esp_adc_cal_raw_to_voltage()`
3. Calculated battery volts after divider (`mv * 0.001 * BAT_DIVIDER`)
4. Timestamp
5. Charging/not-charging state

## 2.2 Practical capture guidance
- Prefer non-charging rows for discharge map generation.
- Keep load profile representative of normal use.
- Capture denser points near low-voltage knee (approx ~3.74V to ~3.40V).
- If operationally possible, for a few points capture both:
  - **loaded voltage** (normal running state)
  - **settled voltage** (~30–60s reduced activity)

Use one convention for the final table (recommended: loaded voltage, because that is what users experience during operation).

---

## 3) Two-stage model (kept)

## Stage A — Measurement correctness (ADC -> volts)
Objective: ensure measured voltage is stable and sane.

What we can reliably do with available fields:
- Validate stability/noise (`raw_adc` spread and `adc_mv` consistency).
- Verify transfer consistency (`raw_adc -> adc_mv -> battery_volts`).
- Catch obvious configuration issues (divider constant mismatch, attenuation misuse, drift patterns).

What we should **not** do with only self-derived data:
- Do **not** fit `V_corrected = a*V+b` unless validated against an external meter at multiple points.

Reason: without independent reference, linear correction risks “calibrating math to itself.”

## Stage B — Runtime mapping (volts -> percent)
Objective: build a monotonic, stable percent mapping that reflects discharge progression.

Because true SOC is not identifiable from voltage-only points, define percent as:
- **percent of representative discharge runtime remaining**.

---

## 4) How to assign percent using timestamps (critical)
Given non-charging rows from a discharge run:
- `t0` = timestamp at discharge segment start
- `t_end` = timestamp at chosen “empty” cutoff event
  - e.g., shutdown/brownout, or chosen loaded-voltage cutoff (e.g., 3.35V)

For any sample at time `t`, assign runtime-percent:

`pct = clamp(100 * (t_end - t) / (t_end - t0), 0, 100)`

This yields a defensible target variable with your available data.

---

## 5) Table construction workflow

1. Filter to non-charging discharge rows.
2. Compute runtime-percent from timestamps (Section 4).
3. Sort by voltage descending.
4. Bin across voltage range (denser bins near low-voltage knee).
5. For each bin, compute robust representatives:
   - voltage = median voltage
   - percent = median runtime-percent
6. Enforce monotonicity:
   - as voltage decreases, percent must not increase
7. Emit anchor table and use piecewise-linear interpolation.

Example target format:

```c
struct SocPoint { float v; uint8_t pct; };
static const SocPoint SOC_TABLE[] = {
  // Placeholder only. Replace with calibrated anchors.
  {4.20f, 100},
  {4.05f,  90},
  {3.92f,  75},
  {3.80f,  60},
  {3.70f,  45},
  {3.62f,  30},
  {3.52f,  15},
  {3.40f,   0}
};
```

---

## 6) Runtime guardrails for stable UX
Even with monotonic table, voltage relaxation/noise can cause bounce.

Recommended low-cost guardrails:
1. **Deadband**: ignore sub-1% changes before UI update.
2. **Rate limit**: cap max % drop per minute unless hard voltage threshold crossed.
3. **Update-on-drop preference** when not charging: avoid brief upward flicker from relaxation.

These preserve smoothness without increasing sample rate.

---

## 7) Charging-mode behavior (no charger-status GPIO)
Voltage during charging can be high long before battery is truly “full,” so direct table mapping while charging is misleading.

Recommended policy when charging heuristic is active:
- Do not trust direct volts->percent mapping for immediate jumps.
- Either:
  - freeze displayed percent + show charging icon, or
  - allow slow cosmetic upward ramp.
- Resume normal discharge mapping after not-charging confirmation window.

This avoids “plug in at 20%, jump to 90%” artifacts.

---

## 8) Validation criteria

## 8.1 Functional
- Percent always clamped `[0,100]`.
- Anchor table monotonic.
- No large step jumps between adjacent samples unless real voltage shift.

## 8.2 Accuracy (correctly framed)
Evaluate against runtime progression, not chemistry SOC:
- MAE vs runtime-percent targets at anchors/midpoints.
- Trend sanity: during representative discharge, percent should decrease approximately with elapsed runtime.

## 8.3 Stability
- No extra redraw cadence beyond existing battery sample ticks.
- No regressions in charging icon behavior, low-battery trigger, deep sleep flow.

---

## 9) ESP32-specific caveats to keep in mind
- ADC2/Wi-Fi interactions can affect readings if future code enables Wi-Fi sampling paths.
- Attenuation/calibration scheme must match voltage range assumptions.
- Divider impedance can impact ADC sampling behavior; averaging helps but verify in data.

---

## 10) Deliverables after tomorrow’s run
1. Filtered discharge dataset (non-charging segment).
2. Runtime-percent computation method and cutoff definition (`t0`, `t_end`).
3. Final monotonic `SOC_TABLE` anchors.
4. Before/after comparison:
   - legacy threshold mapping vs calibrated table,
   - MAE vs runtime-percent target,
   - observed UI stability notes.
5. Minimal firmware patch updating only mapping/guardrail logic.


---

## 11) Automation helper (added)
To avoid manual spreadsheet steps, use:

`tools/generate_runtime_soc_table.py`

What it does:
- Parses the CSV log produced by firmware (`timestamp_ms,...,battery_volts,...,charging`).
- Finds the longest contiguous non-charging segment.
- Computes runtime-percent from timestamps.
- Generates monotonic voltage anchors for selected percent targets.
- Prints a ready-to-paste C++ table.

Example:

```bash
python3 tools/generate_runtime_soc_table.py /path/to/battery_cal.csv \
  --targets "100,90,80,70,60,50,40,30,20,10,5,0" \
  --window 1.5
```

This keeps the mapping workflow reproducible for each new calibration run.

## 12) Notes for the pasted 3494-line run
From the supplied dump:
- The run includes both charging (`charging=1`) and non-charging (`charging=0`) periods.
- Voltage spans approximately `4.186V` down to `2.608V`.
- Existing firmware `battery_pct` remains threshold-bucketed during logging, so treat it as legacy output, not calibration truth.

Therefore, when fitting a new table, use only:
1. `timestamp_ms`
2. `battery_volts`
3. `charging` (for discharge filtering)

and derive percent from runtime as defined in Section 4.
