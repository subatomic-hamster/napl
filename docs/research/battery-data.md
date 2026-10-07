# Battery data: LiPo OCV(SoC) and pack resistance (stage P1d)

Why: the only OCV curve in the parts DB (`parts/drone/lipo-cell-ocv.toml`, Modelica example data, `estimated`) reads 3.507 V/cell at 50 % SoC; a real LiPo rests near 3.80 V there. Result in `docs/validation/thrust-stands.md`: simulated X500 V2 endurance 8.9 min to a 3.5 V/cell loaded cutoff against Holybro's ~18 min.

## New parts

| file | what | fidelity | notes |
|---|---|---|---|
| `parts/common/lipo-cell-ocv-chen2006.toml` | OCV(SoC) 41 points 0..1 plus two-RC parameters vs SoC | `literature` | Chen & Rincon-Mora, IEEE Trans. Energy Conv. 21(2), 2006; fit to measured TCL PL-383562 850 mAh polymer Li-ion pouch cells, room temperature, 80-640 mA pulses. Equation evaluated by us, no digitising. |
| `parts/common/lipo-cell-ocv-bauersfeld2022.toml` | effective U0 vs consumed energy (28 points, SoC 1..0) and per-cell R0 law | coefficients `literature`, table `derived` | Bauersfeld & Scaramuzza, IEEE RA-L 2022 (arXiv 2109.04741); identified on 10 hobby LiPo packs, 1.55-5.2 Ah, 5-70 C. |

## Data quality and caveats

- Chen curve: 3.675 V at 10 %, 3.803 V at 50 %, 3.946 V at 80 %, top 4.103 V because the cells were charged to 4.1 V, not 4.2 V (quote in the part). A 4.2 V LiPo reads up to ~0.1 V higher in the top third. Below SoC 0.05 the exponential is a fit artefact. The Thevenin fit is for an 850 mAh energy cell: R_series 74.5 mOhm, R_ts 46.7 mOhm / 703.6 F (tau 33 s), R_tl 49.8 mOhm / 4475 F (tau 223 s), flat over 20-100 % SoC and rising below 20 %. Normalised: 0.0633 ohm*Ah, i.e. ~12 mOhm per 5.2 Ah cell of the same design; a 35C power pack is lower, so treat as an upper bound. Two RC pairs (the napl model has one).
- Bauersfeld curve: 4.200 V full, 3.80 V at 50 %, 3.53 V when a nominal-capacity cell has delivered 13.32 kJ/Ah. It is fitted to loaded data, so it includes some polarisation and stops at their 3.5 V end criterion. Unit ambiguity RESOLVED BY INFERENCE: the paper gives no unit for E_cell (Eq. 14, Table I). E in kJ per Ah of cell capacity is the only reading consistent with the paper's 3.5 V cutoff and its ~20-26 Wh effective-capacity axis for a 4S 1.8 Ah pack (reasoning in the part header). The R0 law is read as capacity-normalised (ohm*Ah per cell, because R0 multiplies W/Ah in Eq. 12); R_cell = 7.1 mOhm (28 mOhm per 4S pack) at the X500 hover point, but 5.2 Ah single cells are probably outside the identified range. Their tau_RC = 3.3 s (already in the Tattu part) is a lumped time constant, not a physical RC.
- Not usable: Kokam SLPB533459 (740 mAh pouch) polynomial of arXiv 1604.01293 Table I. It is a degree-10 polynomial printed with 3 significant digits; evaluating the printed coefficients gives 7.7 V at SoC 0.8 and -6.7 V at SoC 1, so the rounding destroys it. Not recorded.
- Extra measured pack point (not recorded as a part): Satoh et al., ICAS 2020 paper 0769, 6S enRoute Zion 99 Wh LiPo: "Rcell [Ω] 0.0083" per cell (estimated from averaged terminal voltage over 0.4 <= D <= 0.8 in constant-current discharges). Same order as Bauersfeld's 7 mOhm per cell, i.e. ~6-8 mOhm per cell is the measured range for hobby LiPo.

## Recommendation (no other file edited)

Replace the generic Modelica curve in the X500 build by `lipo-cell-ocv-chen2006` `ocv_vs_soc` (measured pouch cell, rest-like OCV, covers the whole 0..1 range including the knee). Because it tops out at 4.103 V, either accept the ~0.1 V top-end error or, for a 4.2 V pack, scale/shift only the top end explicitly as a `derived` entry (not done here). Use `lipo-cell-ocv-bauersfeld2022` as a cross-check: it agrees with Chen within 0.01-0.03 V over SoC 0.25-0.75 (3.80 vs 3.80 at 0.5) and is the only source with a 4.2 V full point from RC-class packs. Do not use both OCV curves together with Bauersfeld R0/tau, which already absorb polarisation. For pack resistance keep the Tattu `r_internal_pack` = 20 mOhm (consistent with 4 x 5-7 mOhm per cell from Bauersfeld / Satoh); it is still `estimated`, now bracketed by two measured sources of 6-8 mOhm per cell. Chen's R scales to ~49 mOhm per 5.2 Ah 4S pack and should only be an upper bound.

## Expected effect on endurance (rough, not run in napl)

Constant-power check, 209 W pack load (12.69 A at 16.47 V, the thrust-stands hover point), 20 mOhm pack resistance, no RC, 5.2 Ah coulomb counting, stop at 14.0 V loaded (3.5 V/cell). Script: scratchpad `endur.py`, one-off.

| OCV curve | 50 % SoC OCV (V/cell) | endurance to 3.5 V/cell loaded | charge used |
|---|---|---|---|
| Modelica (current) | 3.507 | 9.5 min (napl with RC and its R: 8.9 min) | 2.17 Ah |
| Chen 2006 | 3.803 | 21.2 min | 4.89 Ah |
| Bauersfeld 2022 (SoC mapped, 13.32 kJ/Ah) | 3.795 | 21.3 min | 4.93 Ah |

So the loaded 3.5 V/cell cutoff stops binding until ~94 % of nominal charge is drawn; the endurance becomes capacity limited (~21 min at 100 % depth of discharge, ~17 min at 80 %), which brackets Holybro's ~18 min claim for a 5000 mAh pack. Expect the napl X500 estimate to move from 8.9 to ~17-21 min depending on the usable-depth criterion. Constant power ignores motor efficiency changes with sag and RC polarisation (a few percent).

## Gaps (searched, not found)

- Thevenin R0, R1, C1 for an actual Tattu or Gens Ace pack from a measurement paper: none found. The Tattu manufacturer page for the 5200 mAh 35C pack lists no internal resistance or discharge curve (checked genstattu.com product page); drone-zone.de review URLs found by search returned 404; the Tattu R-Line review on genstattu.com gives only a reviewer's "2-3" per-cell remark without equipment (not recorded). The intofpv thread quotes "a per cell rating of between 0-6 mΩ is as good as it gets" for generic packs, no Tattu 4S 5000 mAh data.
- Task 3 (rated 1C/10C discharge curves for Tattu 4S 5200 or similar 4S 5000 mAh): no tabular or accessible data found; no `tables.discharge_<rate>` written.
- A relaxed (hours-rest) OCV table for a 4.2 V full, RC-class LiPo from a primary measurement: not found. Vendor "LiPo voltage charts" (Grepow, HobbyKing, Ampow blogs) exist but their pages carry no stated method and one (Grepow) held no table, so they were not used.

## Sources

- https://rincon-mora.gatech.edu/publicat/jrnls/tec05_batt_mdl.pdf (Chen & Rincon-Mora 2006)
- https://arxiv.org/pdf/2109.04741 (Bauersfeld & Scaramuzza 2022)
- https://www.icas.org/icas_archive/ICAS2020/data/papers/ICAS2020_0769_paper.pdf (Satoh et al., ICAS 2020; Table 3 "Rcell [Ω] 0.0083")
- https://arxiv.org/pdf/1604.01293 (Kokam SLPB533459 OCV polynomial; rejected)
- https://intofpv.com/t-lipo-internal-resistance-testing, https://genstattu.com/tattu-5200mah-14-8v-35c-4s1p-lipo-battery-pack-with-xt60-plug.html (checked, no usable data)
