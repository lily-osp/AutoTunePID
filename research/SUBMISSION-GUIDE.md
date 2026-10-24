# Submission Guide: Scopus / SINTA Targeting

Paper: *Comparative Performance Evaluation of Multi-Method PID Auto-Tuning for
Resource-Constrained Embedded Control Systems*
Current state: 13 pages, 23 references, 17 tables, 2 figures, zero compile
errors; every number regenerates from `./build.sh --sim`; clean-copy
reproduction verified byte-identical (CSVs, table fragments, figures) with
identical PDF text; MCU static footprint measured by cross-compilation
(ATmega328P 161 B / 5.7 KB flash, ESP32 172 B / 2.2 KB flash).

## Revision status after three external review rounds

Round 1 (structure): three-stage framework, RQ1-RQ4, hardware protocol.
Round 2 (evidence): factorial relay audit (with self-corrected attribution),
Track A/B separation, analytic anti-windup audit, 20-seed noise, gain sweep.
Round 3 (rigor): predeclared composite success criterion with failure
classification, multi-band settling analysis (metric-design effect separated
from controller behavior), factorial main effects, AW repair ablation,
method-audit table with an explicit "canonical" definition, Track B
factor-change table, conservative selection-guide language, measured AVR/ESP32
footprints, clean-room reproduction statement in Declarations.

## Two-track strategy (unchanged, now with stronger Track A evidence)

- **Track A (submit now).** Verification + ablation-backed simulation audit +
  frozen hardware protocol + measured static footprints. Framed throughout as
  verification and pre-validation; on-target timing explicitly future work.
- **Track B (after the levitation campaign).** Adds physical validation,
  on-target WCET/stack/heap, statistics, and failure analysis; unlocks the
  application tier.

## Venue ranking (Track A)

| Rank | Venue | Type | Why it fits | Notes |
|------|-------|------|-------------|-------|
| 1 | IFAC symposium via IFAC-PapersOnLine (ACE / LC / TEAC cycle 2027) | Conference, Scopus + EI | Benchmark/audit papers with ablation rigor are core IFAC material | 6-8 pp format; condense Sections 5-6 |
| 2 | IEEE Access | Journal, Scopus, OA | Rigorous engineering evaluation + methodology; fast review | APC applies |
| 3 | Results in Control and Optimization (Elsevier) | Journal, Scopus | Reproducible benchmark studies welcomed | No page charges |
| 4 | SN Computer Science | Journal, Scopus | Software-audit angle | Backup |
| 5 | JESTECH | Journal, Scopus, OA | Applied control + embedded experiments | Backup |

For SINTA 1 specifically: Jurnal Teknik ITS (JUTS), Jurnal Rekayasa
Elektronika, or JOIV/TELKOMNIKA accept this profile, but the honest scope
(software audit + pre-validation) fits the Scopus-indexed international
venues above better; a SINTA 1 target becomes defensible after the hardware
campaign lands (Track B tier: ISA Transactions, Control Engineering Practice).

## Venue ranking (Track B, after hardware data)

ISA Transactions > Control Engineering Practice > Journal of Process Control
> IEEE Transactions on Education (curriculum framing).

## Pre-submission checklist

- [ ] Abstract ~300 words; trim per venue cap, keep the three results
- [ ] Cover letter leads with the three verifiable results and the
      self-correction episode (reviewers reward it)
- [ ] Zenodo DOI for the artifact repository once public
- [ ] Keep: AUTOSAR "guideline-informed" wording; provisional selection
      guide; scope statements on every causal claim
- [ ] Run `sim/mcu_footprint.sh` fresh if the toolchain changes; record
      arduino-cli/core versions in the submission letter
- [ ] Disclose author-owns-the-library (Declarations)

## Cover letter skeleton

> Dear Editors,
>
> I submit "[title]" for consideration in [venue]. The paper audits a
> multi-method embedded PID auto-tuning library and reports three
> ablation-backed results: a factorial attribution of relay-identification
> error (estimation procedure, not relay placement, dominates, with main
> effects of -78.3 vs +4.0 percentage points), a two-track separation showing
> the same tuning rules that fail end-to-end mostly succeed with correct
> inputs and a repaired protection mechanism, and a source-derived
> anti-windup engagement condition whose closed-form predictions match
> observation within 0.4 units and whose repair direction is validated in
> ablation. A predeclared success criterion classifies every failure, a
> multi-band analysis separates controller behavior from metric design,
> static footprint is measured by cross-compilation for AVR and ESP32, and a
> frozen hardware protocol is specified. All artifacts regenerate from a
> single command and were verified from a clean copy.
>
> Sincerely,
> Azzar Budiyanto, Independent Researcher
