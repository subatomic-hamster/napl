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

# TUNE-2: MHT identity stability on the threat-suite swarms

Scenes: the threat suite (`threat-bench.md`), 50 seeds (1..50) x 17 scenes, `tools/threat_bench.py`. Problem: `imm2-mht` ID switches
swarm_v 17.8 / swarm_split 23.9 / busy 15.8 against JPDA 4.7 / 4.6 / 8.0 at similar GOSPA. Exploration used 12 to 16 seeds (1..12 / 1..16)
with a scratch runner (not committed); only the final table below is 50 seeds. Result: MHT 10.3 / 7.7 / 11.7, GOSPA better on every
swarm scene, unchanged elsewhere. JPDA's level is NOT reached (see the end).

## Root cause (traced, then ablated)

Trace of swarm_v seed 3 (67 switches): the swarm ends in a line with 0.6 to 1 m between neighbours against 0.5 m measurement noise. Two
trees (ids 7 and 1703) sit on two such neighbours and swap truths for 20 consecutive scans. Per scan the older tree (score 26 to 30, no
misses) takes whichever of the two nearby measurements fits it best, so its output state hops between the neighbours; the other tree gets the
leftover, weakly associated measurement (score 6 to 21, 1 to 3 misses, velocity estimate off by 2 m/s). The cause is the greedy global
hypothesis: trees are ordered by their own best score and each takes its best free leaf, so the order, not the joint likelihood, decides
which neighbour gets which measurement. GNN solves the same ambiguity with one optimal assignment per scan.

Ablations on swarm_v / swarm_split / busy, 12 seeds, ID switches (GOSPA in brackets for swarm_v), tested one at a time. The two joint
rows are the first version (total-score assignment, no `drop`/`margin`), whose side effect on one-object scenes is described below:

| change                                                                 | swarm_v      | swarm_split | busy        | reading                                                          |
| ---------------------------------------------------------------------- | ------------ | ----------- | ----------- | ---------------------------------------------------------------- |
| as committed (greedy, kmax 8, nscan 1, decay 0.8)                      | 20.8 (1.130) | 15.3        | 14.8        |                                                                  |
| kmax 1 (no multiple hypotheses)                                        | 19.8         | 17.3        | n/a         | multi-hypothesis tracking is not the cause                       |
| kmax 16 (more hypotheses)                                              | 20.8         | 15.3        | 14.8        | identical: kmax never binds, the score gap prunes                |
| nscan 3 / decay 1.0                                                    | 28.4 / 31.8  | 23.7 / 31.0 | 22.8 / 21.1 | worse (the old settings)                                         |
| keep only strong associations as leaves                                | 16.4 (1.290) | 12.8        | 16.9        | fewer switches but GOSPA 1.13 -> 1.29: rejected                  |
| hysteresis alone (prefer the leaf continuing last scan's exposed leaf) | 20.3         | 16.6        | 14.5        | no effect: the lineage breaks it removes (0 left) were a symptom |
| joint assignment alone                                                 | 11.4 (1.109) | 10.4        | 12.4        | halves the switches                                              |
| joint + hysteresis (stick 5)                                           | 6.3 (1.087)  | 2.9         | 11.3        | best on swarms, but see below                                    |

Lineage breaks (the selected leaf is not a child of the leaf exposed last scan) were 1.3 % of the selections and coincided with 43 % of
the switches under greedy selection; under joint selection alone they rise to 7.6 % of selections and 80 % of switches, which is why the
hysteresis only helps together with the joint assignment.

## Fix (python/napl/track/trackers.py, `MHT`, options of `variants.KW["imm2-mht"]`)

- `joint=1`: per scan, one linear assignment of the trees to the current measurements (or coast), cost = regret of the chosen leaf below the
  tree's own best leaf; a tree without a coasting leaf may be left out at cost `drop` (4). The greedy loop afterwards repairs leaves whose
  older scans clash. The assignment replaces the greedy choice only if it is better by `margin` (3), see the failure below.
- `stick=4`: regret bonus (log-likelihood units) for a leaf that continues the leaf the tree exposed last scan.
- Not changed: scores, pruning, births, hit rule (`hit_llr`), `kmax`/`nscan`/`decay`.

Two first versions failed and were corrected, both seen on the one-object scenes:

1. Maximising the summed total scores (no regret, no drop option) forced every tree to take a leaf: a young duplicate tree with a high
   score then took the measurement and the old tree coasted (survey seed 16: tree 7, score 19.0 on measurement 3, coasted at 11.8 while
   duplicate 778, score 10.6, took it). Result (50 seeds): up to 0.2 spurious ID switches per run on the one-object scenes (survey 0.20,
   weave 0.18) where the committed MHT has 0, and GOSPA +0.006 to +0.012 on every one-object scene. Regret plus the drop option fixes the forcing but a near tie still went to the young tree.
2. `margin`: the greedy choice (old tree first) stays unless the assignment is better by 3 log-likelihood units. margin 6 loses the swarm
   gain (swarm_v 14.6), margin 1.5 gives swarm_v 6.8 but hanging false 0.071 -> 0.096, margin 3 leaves every one-object scene at the
   committed value (16 seeds: survey, weave 0 switches, same GOSPA) with swarm_v 10.9 and swarm_split 5.4 (16 seeds). `stick` above 5
   makes false tracks (swarm_v false 0.65 at stick 7, a cliff, so 4 is kept with margin).

## Before / after, 50 seeds, all 17 scenes (`imm2-mht`; JPDA and GNN-s are unchanged, shown for reference)

| scene           | GOSPA before -> after | +-95 % (after) | breaks       | ID sw          | false          | JPDA GOSPA / ID sw | GNN-s GOSPA / ID sw |
| --------------- | --------------------- | -------------- | ------------ | -------------- | -------------- | ------------------ | ------------------- |
| route_hover     | 0.391 -> 0.391        | 0.008          | 0.00 -> 0.00 | 0.00 -> 0.00   | 0.072 -> 0.072 | 0.399 / 0.00       | 0.436 / 0.12        |
| route_fast      | 0.414 -> 0.414        | 0.008          | 0.00 -> 0.00 | 0.00 -> 0.00   | 0.078 -> 0.078 | 0.421 / 0.02       | 0.454 / 0.14        |
| orbit           | 0.409 -> 0.409        | 0.010          | 0.00 -> 0.00 | 0.00 -> 0.00   | 0.078 -> 0.074 | 0.425 / 0.00       | 0.493 / 0.22        |
| survey          | 0.394 -> 0.394        | 0.007          | 0.00 -> 0.00 | 0.00 -> 0.00   | 0.041 -> 0.041 | 0.411 / 0.00       | 0.486 / 0.30        |
| weave           | 0.449 -> 0.450        | 0.009          | 0.00 -> 0.00 | 0.00 -> 0.00   | 0.047 -> 0.051 | 0.451 / 0.00       | 0.464 / 0.20        |
| swarm_v         | 1.117 -> 1.085        | 0.016          | 2.04 -> 0.94 | 17.82 -> 10.30 | 0.220 -> 0.185 | 1.064 / 4.68       | 1.131 / 4.54        |
| swarm_split     | 1.339 -> 1.250        | 0.025          | 7.72 -> 1.74 | 23.90 -> 7.68  | 0.298 -> 0.217 | 1.249 / 4.64       | 1.281 / 1.66        |
| balloons        | 0.639 -> 0.638        | 0.006          | 0.00 -> 0.00 | 0.56 -> 0.24   | 0.039 -> 0.039 | 0.639 / 0.00       | 0.648 / 0.14        |
| hanging         | 0.541 -> 0.546        | 0.026          | 0.48 -> 0.78 | 1.60 -> 0.68   | 0.071 -> 0.068 | 0.554 / 2.12       | 0.663 / 1.30        |
| birds           | 1.023 -> 1.022        | 0.016          | 1.00 -> 0.90 | 1.40 -> 1.24   | 0.161 -> 0.163 | 1.021 / 1.18       | 1.059 / 3.44        |
| clutter_mix     | 1.265 -> 1.264        | 0.033          | 4.44 -> 4.44 | 4.64 -> 4.56   | 0.430 -> 0.431 | 1.257 / 4.30       | 1.274 / 5.26        |
| busy            | 1.708 -> 1.692        | 0.034          | 8.40 -> 7.56 | 15.84 -> 11.66 | 0.535 -> 0.531 | 1.669 / 7.98       | 1.694 / 8.90        |
| cam_mix         | 2.489 -> 2.491        | 0.110          | 8.00 -> 8.04 | 4.30 -> 4.52   | 0.975 -> 0.979 | 2.488 / 4.50       | 2.487 / 5.12        |
| close_cam       | 0.415 -> 0.415        | 0.013          | 0.02 -> 0.02 | 0.26 -> 0.26   | 0.004 -> 0.004 | 0.417 / 0.30       | 0.416 / 0.26        |
| random          | 0.837 -> 0.834        | 0.088          | 1.50 -> 1.42 | 3.60 -> 2.96   | 0.195 -> 0.193 | 0.833 / 1.58       | 0.894 / 2.34        |
| random_cam      | 2.572 -> 2.573        | 0.345          | 7.48 -> 7.58 | 9.54 -> 9.96   | 0.836 -> 0.839 | 2.560 / 8.88       | 2.617 / 11.54       |
| close_cam_lidar | 0.410 -> 0.410        | 0.016          | 0.02 -> 0.02 | 0.02 -> 0.02   | 0.004 -> 0.004 | 0.412 / 0.06       | 0.411 / 0.02        |

GOSPA is not worse on any scene beyond noise: the five cells that moved up (weave +0.0005, hanging +0.004, cam_mix +0.001, random_cam
+0.001, route_hover +0.00002) are all under 20 % of their 95 % band. Hanging has more breaks (0.48 -> 0.78) at fewer ID switches (1.60 -> 0.68);
not explained. Paired seeds (same seeds before and after), 4 processes; ms/update with the joint assignment was +0.1 ms (7.3 -> 7.5 on swarm_v).
PHD and the other variants are untouched (no code path of theirs changed).

## Not done

- **MHT is still above JPDA/GNN-s on the swarms**: swarm_v 10.3 vs 4.7, busy 11.7 vs 8.0 (swarm_split 7.7 vs 4.6). Closing it needs margin
  below 3 (swarm_v 6.8 at margin 1.5 on 16 seeds), which costs duplicate tracks on `hanging` and `birds` (not run at 50 seeds). The remaining
  switches in swarm_v are mostly neighbours within the noise (0.6 to 1 m against 0.5 m), which no position-only tracker separates reliably.
  In `busy` most remaining switches are young trees taking over a target (birth after a gap), not neighbour swaps.
- **PHD (24 to 40 switches on the swarms) is unchanged.** One attempt, tried and removed: carry identity by position continuity
  (match the extracted label states to the previous tracks by linear assignment, gate 1.5 m) instead of the mixture label. 12 seeds:
  swarm_v switches 26.4 -> 20.4, breaks 15.5 -> 8.1, swarm_split 27.3 -> 19.8, GOSPA unchanged; gate 3 m was worse (30), 5 m much worse (53).
  A 25 % cut is not the JPDA level, so it was not shipped; the label problem is in the mixture (components shared between labels), not in the output.
- The margin / stick / drop values were tuned on these scenes (12 to 16 seeds exploring, 50 final); the cliff at stick > 5 means they should be
  re-checked for other clutter densities or sensors.
