# Why JPDA, MHT and GM-PHD underperformed, and what was changed

Scenarios: the existing napl.track.scenarios only (no new threat profiles). Exploration: 40 paired seeds (1..40) with
`tools/track_tune.py`; final numbers: the 100-seed grid in `tracking-bench.md` / `.json`, same seeds as the pre-tuning run
(every cell of the untouched variants is bit-identical before and after, so the comparison is paired).
Tuned options live in `variants.KW`; `tools/track_tune.py imm2-mht:kmax=4,nscan=3,decay=1.0,hit_llr=-1e9` reproduces any old setting.

## Root causes (each confirmed by an experiment, not by reading code alone)

1. **Loose confirmation hits (all of JPDA, MHT, and also GNN).** A scan counted as a hit when `W = pd*L/lam > pd-miss weight`
   (JPDA: `beta0 < 0.5`, i.e. W > 0.1; MHT: any gated measurement; GNN: llr > 0, same W > 0.1). A newborn track has a gate of
   ~350 m^3 (velocity prior 10 m/s), so about 7 % of scans have a clutter point inside it [estimate from clutter density x gate
   volume] with W of 0.1 to 2 (per-scan trace of JPDA false tracks: hits at W = 0.1 to 0.8, beta0 0.03 to 0.47), which counts as
   a hit. At 15 clutter/scan swarm5 makes about 15 clutter births per scan; 3-of-5 confirmation lets ~15 (GNN) to ~26 (JPDA)
   of them through per 150-frame run. Inference, not tested: GNN suffers less because its hard update shrinks the covariance
   after a weak hit (the gate no longer catches the next clutter point) while JPDA and MHT keep honest, larger covariances.
   Diagnosis: false confirmed
   ids per run on swarm5 (8 seeds): GNN 15.5, JPDA 25.9, MHT 16.0; life 6.7 / 9.0 / 11.5 frames; "far" (> 6 m from any truth)
   false tracks were 0.68 / 1.54 / 1.27 per frame. A confirmed false track needs 3 consecutive weak hits; requiring real
   evidence (W above ~33x the missed-detection weight: `hit_beta0=0.01` for JPDA, `hit_llr=3.5` for MHT, `min_llr=3.5` for
   GNN) removes them while real targets (W in the hundreds) are untouched. Confirmation stays 3-of-5, so t_confirm does not move.
   (4-of-5 gives the same false-track removal but delays every confirmation by a scan: GNN missed 0.223 -> 0.272 on cross and
   0.408 -> 0.489 on swarm5, 40 seeds.)
2. **JPDA `_dedupe` deleted valid tracks.** The coalescence guard (delete a younger track within d^2 < 9 of an older one) fired on
   legitimate close pairs in swarm5: 1.98 breaks/run with it, 0.12 without (m=4 runs, 40 seeds), and it was the single most
   expensive call (8.1 ms vs 4.7 ms per update). Off by default now (`dedupe=0`); the option remains.
3. **MHT pruned the leaf the global hypothesis had just chosen.** The kmax/gap pruning ran after selection and could drop the selected
   leaf; the tree then fell out of the output for that scan (`sel` dropped, track unselected, birth of a duplicate tree). Always keeping
   the selected leaf alone took swarm5 breaks 9.9 -> 5.8 and cross breaks 1.57 -> 0.65 (m=4, 40 seeds).
4. **MHT absolute scores favoured old trees, and kmax=4 / N=3 was too narrow.** Greedy selection orders trees by best score; a long-lived
   tree with a score of 400 beats a young tree sitting on the target. Keeping more hypotheses (`kmax=8`, `nscan=2`) took swarm5
   breaks 5.8 -> 1.5 and missed 0.73 -> 0.54 (m=4 runs, decay 0.9); `nscan=1` -> 0.8 breaks; `decay=0.8` (fading the score) cut
   ID switches 11.3 -> 6.7 on top of the old kmax/nscan.
5. **GM-PHD labels are not exclusive.** (a) Components were merged only within a label, so two labels sat on one target (dup ~0.18
   per frame); (b) a label spawns a component for every gated measurement, so when two targets come within a gate one label ends up
   with mass ~1.6 (two targets) and extraction (heaviest component per label) shows only one: the other target "breaks"; (c) a label
   whose weight fell under 0.05 (two consecutive misses at Pd 0.9) was dropped from the track list and had to re-confirm (a 3-scan hole;
   12 of the 43 swarm5 breaks per run had no track within 5 m). Fixes: `xmerge` (merge across labels, heaviest label wins),
   `split_w=0.7` (a second distinct component of weight >= 0.7 gets a new label), `keep` (the track survives while its label has
   components and fewer than max_miss low-weight scans), `merge_d2=9`. Each alone is neutral or worse on swarm5 (xmerge alone 1.87,
   split alone 1.96); together 1.82 -> 1.41.
6. **Not causes (measured, no effect or worse).** Clutter density passed to the tracker (`lamx=3` on top of 4-of-5: swarm5
   GOSPA 1.206 -> 1.178 for JPDA, nothing for GNN), `pdx` (assumed Pd) for PHD (0.7 helps turn and
   cross misses, hurts swarm5 and ID switches once the label fixes are in), `w_birth` 0.05 to 0.3, `ps`, `jmax`, `kmax` 12 vs 8, JPDA
   `exclusive_hit` (a hit only for the measurement the track owns), MHT `coast_keep` (always keep a coasting leaf), MHT `max_unsel`
   (drop persistently unselected trees), `gate_all` (births gated by all confirmed leaves), `cwin=1,2` (conflicts through only the
   newest scans: much worse, swarm5 GOSPA 2.3), exact per-cluster global hypothesis (fewer misses, but more duplicate tracks and ID switches:
   swarm5 GOSPA 1.31 vs 1.24 greedy at that point). Those code paths were removed again.

## Before / after (100 seeds, final grid; before = the committed table from main)

GOSPA | ID switches | track breaks | false part of GOSPA | ms/update (see timing note)

| scenario | variant    | GOSPA          | ID sw          | breaks        | false          | ms/update     |
| -------- | ---------- | -------------- | -------------- | ------------- | -------------- | ------------- |
| cv       | imm2-jpda  | 0.418 -> 0.411 | 0.00 -> 0.00   | 0.01 -> 0.01  | 0.049 -> 0.003 | 2.13 -> 0.95  |
| cv       | imm2-mht   | 0.416 -> 0.411 | 0.00 -> 0.00   | 0.01 -> 0.01  | 0.034 -> 0.001 | 3.63 -> 1.10  |
| cv       | phd-singer | 0.521 -> 0.504 | 0.15 -> 0.01   | 0.97 -> 0.01  | 0.009 -> 0.001 | 1.52 -> 0.82  |
| turn     | imm2-jpda  | 0.563 -> 0.480 | 0.06 -> 0.00   | 0.00 -> 0.00  | 0.374 -> 0.052 | 3.48 -> 1.40  |
| turn     | imm2-mht   | 0.557 -> 0.476 | 0.02 -> 0.00   | 0.00 -> 0.00  | 0.350 -> 0.026 | 5.53 -> 2.40  |
| turn     | phd-singer | 0.557 -> 0.528 | 0.01 -> 0.03   | 1.97 -> 0.05  | 0.004 -> 0.008 | 2.57 -> 1.43  |
| ca       | imm2-jpda  | 0.505 -> 0.495 | 0.00 -> 0.00   | 0.00 -> 0.00  | 0.071 -> 0.015 | 3.29 -> 1.37  |
| ca       | imm2-mht   | 0.510 -> 0.496 | 0.00 -> 0.00   | 0.00 -> 0.00  | 0.092 -> 0.015 | 5.46 -> 2.39  |
| ca       | phd-singer | 0.554 -> 0.527 | 0.00 -> 0.00   | 1.33 -> 0.03  | 0.001 -> 0.001 | 2.46 -> 1.35  |
| cross    | imm2-jpda  | 0.700 -> 0.637 | 1.80 -> 1.80   | 0.02 -> 0.01  | 0.344 -> 0.053 | 6.17 -> 2.26  |
| cross    | imm2-mht   | 0.735 -> 0.638 | 2.19 -> 2.22   | 1.50 -> 0.03  | 0.347 -> 0.041 | 9.59 -> 4.03  |
| cross    | phd-singer | 0.825 -> 0.775 | 2.16 -> 1.87   | 4.03 -> 0.10  | 0.018 -> 0.016 | 4.19 -> 2.41  |
| swarm5   | imm2-jpda  | 1.995 -> 1.097 | 1.83 -> 1.06   | 2.11 -> 0.31  | 1.719 -> 0.279 | 16.80 -> 4.53 |
| swarm5   | imm2-mht   | 2.031 -> 1.112 | 11.88 -> 7.34  | 10.99 -> 0.65 | 1.630 -> 0.183 | 21.72 -> 8.66 |
| swarm5   | phd-singer | 1.833 -> 1.393 | 28.68 -> 12.56 | 43.60 -> 6.36 | 0.479 -> 0.341 | 11.71 -> 7.23 |
| cam      | imm2-jpda  | 0.922 -> 0.896 | 0.00 -> 0.00   | 0.00 -> 0.00  | 0.188 -> 0.016 | 1.60 -> 0.78  |
| cam      | imm2-mht   | 0.898 -> 0.894 | 0.00 -> 0.00   | 0.00 -> 0.00  | 0.027 -> 0.003 | 2.54 -> 1.22  |
| cam      | phd-singer | 1.359 -> 1.264 | 0.00 -> 0.00   | 2.27 -> 0.04  | 0.020 -> 0.021 | 1.17 -> 0.62  |

Timing note: the old table ran 12 processes on 16 cores, the new one 2 on a quiet box (untouched imm2-gnn on swarm5: 8.5 -> 5.7 ms),
so absolute ms are not comparable. As a ratio to imm2-gnn on swarm5: JPDA 1.98 -> 0.80 (no dedupe), MHT 2.55 -> 1.53 (kmax 8 but
nscan 1), PHD 1.38 -> 1.28.

Swarm5 against the same-strictness GNN (`imm2-gnn-s`, 1.153 GOSPA, 0.70 ID sw, 0.19 breaks): JPDA 1.097 is now better than GNN
(and GNN with the original loose rule, 1.520, was also paying for root cause 1; that rule is unchanged in `imm2-gnn`).

## What is still weaker than GNN (not fixed)

- **MHT ID switches on swarm5: 7.3 vs 0.7 for GNN/JPDA.** Breaks are gone but identities flip between neighbouring trees when two
  targets come within ~2 m, and young trees steal an identity from a long-lived tree that drifted off by ~2 m. Greedy global
  hypothesis plus score-based tree order is the limit; an exact search did not fix it (it admitted duplicates). Needs a
  proper multi-frame assignment with a track-existence term, or ID-stable output (hold the displayed id of a truth-sized cluster).
- **GM-PHD swarm5: 6.4 breaks and 12.6 ID switches**, ~0.1 s slower to confirm; labels follow components, not targets, so close pairs
  swap. A CPHD/LMB or a label-association step against the previous output would be the next step.
- **Crossing ID switches (~2 per run) are the same for every associator** (GNN 2.2, JPDA 1.8, MHT 2.2, PHD 1.9): two CV targets crossing
  within sensor noise; this is the information limit of position-only measurements, not a tracker defect.
- The thresholds (3.5 / 0.01) were tuned on these scenarios only (40 seeds exploring, 100 final); a different clutter density
  or sensor needs them re-derived: the rule of thumb is hit if the association weight exceeds ~30x the missed-detection weight.
