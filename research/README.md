# Comparative Performance Evaluation of Multi-Method PID Auto-Tuning for Resource-Constrained Embedded Control Systems

Research paper evaluating [AutoTunePID](https://github.com/lily-osp/AutoTunePID)
in three stages: implementation verification, ablation-backed simulation
audit, and a frozen hardware benchmark protocol for a fan-driven
air-levitation chamber. Revised through four external review rounds.

**Canonical location:** the `research/` directory of the AutoTunePID
repository. Everything is self-contained there: `./build.sh --sim` finds the
library source at `../src` automatically. (A standalone working copy also
lives at `~/project/research/AutoTunePID-Comparative-Evaluation`; the repo
copy is the single source of truth.)

## Layout

```
paper.tex            manuscript (two-column article, 13 pages)
references.bib       bibliography (23 verified entries)
build.sh             PDF build helper (--sim regenerates data, --clean resets)
Makefile             thin wrapper over build.sh
sim/Arduino.h        host stub for Arduino.h (millis, constrain)
sim/harness.cpp      benchmark harness, compiles the unmodified library source
sim/mcu_footprint.sh cross-compiles minimal sketches for ATmega328P and ESP32;
                     flash/RAM delta + on-target sizeof (run separately)
results/             generated CSVs and LaTeX table fragments (one --sim run regenerates all;
                     includes per-plant relay traces and per-seed noise statistics)
figures/             generated pgfplots CSV data (setpoint steps)
SUBMISSION-GUIDE.md  venue strategy, checklist, cover letter skeleton
paper.pdf            generated manuscript
```

## Build

```sh
./build.sh                  # compile paper.pdf
./build.sh --sim            # recompile harness, regenerate results + figures, compile
./build.sh --clean          # remove artifacts and generated data
./sim/mcu_footprint.sh      # (optional) AVR/ESP32 cross-compilation footprint
```

Requires: `pdflatex`, `bibtex`, `g++` (C++14), the AutoTunePID library source
(override with `ATP_LIB_SRC`); the footprint script additionally needs
`arduino-cli` with the `arduino:avr` and `esp32:esp32` cores.

## Paper structure (three stages, three verifiable results)

1. **Verification (RQ1).** Fidelity within 0.13 ppm; method-audit table with
   an explicit canonical definition: five options -> four gain mappings -> two
   canonical (CC is a Ku-based variant, IMC == Lambda in v1.2.0, process gain
   omitted from model-based rules).
2. **Simulation audit (RQ2, RQ3), ablation-backed.**
   - Factorial relay audit with main effects: Ku error, estimator -78.3 pp vs
     centering +4.0 pp (centering instead repairs Tu by -19.5 pp); raw traces
     released.
   - Track A (end-to-end) vs Track B (ground-truth inputs, factor table):
     pipeline explains the failures; canonical CC* is the rule-level exception
     on lag-dominant plants.
   - Anti-windup audit under explicit assumptions: engagement condition
     Ki*Imax < u_eq, predictions within 0.4 units, disable ablation and
     harness-side contribution-clamp repair both remove the offsets.
   - Predeclared composite success criterion + failure classification
     (offset/oscillatory/band); 1/25 clean success, 0/500 at 1% noise.
   - Multi-band settling analysis: the 2%-band headline is largely metric
     design; settled fractions 4/24/40% (clean) and 0/34/51% (1% noise) at
     2/5/10% bands; dither effect documented; noise-floored protocol band
     validated in simulation.
   - 20-seed noise campaigns (1%, 3%), plant-gain sweep (K-blind rules
     degrade as 1/K), resource footprint measured by cross-compilation:
     161 B object on ATmega328P (+5.7 KB flash), 172 B on ESP32 (+2.2 KB).
3. **Hardware benchmark protocol (RQ4).** Sequenced air-levitation campaign:
   characterization before any tuning data, validity criteria with failure
   retention, five standardized conditions, ITAE/control-effort/total-variation
   metrics, noise-floored right-censored settling, composite success criterion,
   blocked randomized 250-trial statistical plan, fixed CSV schema, on-target
   resource measurement, safety requirements.

## Reproducibility

Verified from a clean copy: all CSVs, table fragments, and figures regenerate
byte-identically, and the PDF text is identical; the only run-varying quantity
is the host timing indicator (reported as a range, never enters a claim).
Seeds and noise distributions are fixed in the harness source.
