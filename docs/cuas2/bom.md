# cuas2 bill of materials (hardware + filament, ammo separate)

Prices in USD, prices looked up 2026-10 (v1 lines carried over from `docs/cuas-lite/bom.md`, same sources). Fidelity: **datasheet** = a price read from a vendor page, **derived** = converted from another currency or a multi-pack
(1.15 USD/EUR, 1.33 USD/GBP, 7.8 HKD/USD, none looked up) or pack price divided by pack size, **estimated** = a search snippet, an unfetchable page or an allowance with no listing.
Part ids are in `parts/` (cuas2 printed parts `parts/cuas2/`); the assembly is `assemblies/cuas2.toml`. Excluded from the totals (user's rule): the Jetson Orin Nano, the Logitech Brio, the foam rounds, the 12 V brick.
Every line has a link or is marked `[est]` (allowance with no listing).

| #   | Part                                                                                                 | Qty | Unit  | Line  | Source                                                                                                                                           | Fidelity  |
| --- | ---------------------------------------------------------------------------------------------------- | --- | ----- | ----- | ------------------------------------------------------------------------------------------------------------------------------------------------ | --------- |
| 1   | NEMA 17 17HS4401S 42x40 (pan, tilt)                                                                  | 2   | 8.11  | 16.22 | [Handson Tech sheet](https://www.handsontec.com/dataspecs/17HS4401S.pdf); EUR 7.05 (bitsandparts.nl)                                             | derived   |
| 2   | BTT TMC2209 V1.3 driver                                                                              | 2   | 7.89  | 15.78 | [BIQU](https://biqu.equipment/collections/driver-board/products/bigtreetech-tmc2209-stepper-motor-driver-for-3d-printer-board-vs-tmc2208)        | datasheet |
| 3   | Raspberry Pi Pico H                                                                                  | 1   | 5.00  | 5.00  | [Adafruit 5525](https://www.adafruit.com/product/5525)                                                                                           | datasheet |
| 4   | GT2 20T pulley, 5 mm bore, 6 mm belt (pan 100T and tilt 80T are printed)                             | 2   | 0.92  | 1.84  | [Olimex GT2-5T20-6MM](https://olimex.com/Products/Robot-CNC-Parts/GT2/GT2-5T20-6MM), 0.80 EUR                                                    | derived   |
| 5   | GT2 6 mm closed belt loops (pan, tilt)                                                               | 2   | 3.00  | 6.00  | [est] no loop of these lengths has a fetched price (as v1)                                                                                       | estimated |
| 6   | 2205 2300 KV brushless motor, 5 mm shaft (flywheels; new; was 2207 1800 KV at 13.12)                 | 2   | 9.99  | 19.98 | [GetFPV Racerstar BR2205S 2300KV](https://www.getfpv.com/racerstar-br2205s-2300kv-brushless-motor.html), search snippet 2026-10-27, out of stock | estimated |
| 7   | 30 A BLHeli_S / Bluejay-capable ESC, DShot (flywheels; new)                                          | 2   | 14.99 | 29.98 | [Tiger Motor F 30A at GetFPV](https://www.getfpv.com/tiger-motor-f-30a-2-4s-esc-blheli-s-w-dshot.html)                                           | datasheet |
| 8   | TowerPro SG92R servo (gate plug, pusher; v1 had one)                                                 | 2   | 5.95  | 11.90 | [Adafruit 169](https://www.adafruit.com/product/169)                                                                                             | estimated |
| 9   | A3144 hall switch (pan, tilt, gate home, push home)                                                  | 4   | 0.52  | 2.08  | [Tinytronics](https://www.tinytronics.nl/en/sensors/magnetic-field/3144-hall-effect-switch), 0.45 EUR                                            | derived   |
| 10  | Neodymium magnet disc 6x2 mm                                                                         | 4   | 0.50  | 2.00  | [est] allowance, [Adafruit search](https://www.adafruit.com/search?q=neodymium+magnet)                                                           | estimated |
| 11  | 608ZZ bearing (both trunnions)                                                                       | 2   | 0.80  | 1.60  | [Olimex](https://www.olimex.com/Products/Robot-CNC-Parts/Bearings/608ZZ/), 0.70 EUR                                                              | derived   |
| 12  | 6 in square lazy susan                                                                               | 1   | 4.95  | 4.95  | [RF Parts](https://www.rfparts.com/lazy-susan.html)                                                                                              | datasheet |
| 13  | E-stop, red mushroom 22 mm, 1NO-1NC (NC used)                                                        | 1   | 5.90  | 5.90  | [Maplin Pro](https://pro.maplin.co.uk/products/red-mushroom-22mm-emergency-stop-switch-no-nc), GBP 4.44                                          | derived   |
| 14  | Key switch E-Switch KO106A102 (SPST, 1 A)                                                            | 1   | 3.94  | 3.94  | [Digi-Key](https://www.digikey.com/en/products/detail/e-switch/KO106A102/EG2625-ND/502089)                                                       | datasheet |
| 15  | 40 A 12 V automotive relay + socket ($3.98 + $4.98; replaces the Songle)                             | 1   | 8.96  | 8.96  | [Parts Express relays and sockets](https://www.parts-express.com/electronic-parts/relays/12v-relays-sockets/reviewrating/1to5)                   | derived   |
| 16  | Inline ATC fuse holder 12 AWG + 20 A blade fuse (spec 9: 20 A)                                       | 1   | 4.00  | 4.00  | [Amazon holder](https://www.amazon.com/Quelong-Inline-Waterproof-Lighting-Circuits/dp/B0GDYJ3TG8) $3.00 unreadable page + fuse $1.00 [est]       | estimated |
| 17  | XT60 pair (inlet)                                                                                    | 1   | 2.00  | 2.00  | [CMU MRSD project sheet](https://mrsdprojects.ri.cmu.edu/2025teamb/wp-content/uploads/sites/84/2025/05/xt60.pdf), KiwiQuads 3.49 NZD             | derived   |
| 18  | LM2596 buck module (5 V: gate servo, pusher rail)                                                    | 1   | 1.90  | 1.90  | [ebuildelectronics](https://ebuildelectronics.odoo.com/shop/dc-dc-buck-converter-lm2596-adjustable-69), 109 PHP                                  | derived   |
| 19  | L7805CV regulator, TO-220 (5 V logic for the supervisor, limiter, counter, head halls; new)          | 1   | 1.00  | 1.00  | [Digi-Key L7805CV](https://www.digikey.ie/en/products/detail/umw/L7805CV/17635308)                                                               | derived   |
| 20  | IRLB8721 MOSFET (relay coil low-side switch; v1 had two for the flywheels)                           | 1   | 2.25  | 2.25  | [Adafruit 355](https://www.adafruit.com/product/355)                                                                                             | datasheet |
| 21  | IRF9540N P-MOSFET (pusher power window, limiter; new)                                                | 1   | 0.73  | 0.73  | [Tayda IRF9540N](https://www.taydaelectronics.com/transistors-semiconductors/fets-mosfets/irf9540-irf9540n-power-mosfet-p-channel-23a-100v.html) | datasheet |
| 22  | 2N3904 NPN (counter clamp; new)                                                                      | 1   | 0.15  | 0.15  | [Digi-Key 2N3904](https://www.digikey.es/en/products/detail/dcomponents/2N3904/13575042)                                                         | estimated |
| 23  | 1N5822 Schottky (relay coil flyback)                                                                 | 1   | 0.34  | 0.34  | [TME](https://www.tme.com/us/en-us/details/1n5822-e3_54/tht-schottky-diodes/vishay/)                                                             | datasheet |
| 24  | 74HCT123 dual retriggerable monostable (supervisor, limiter; new)                                    | 2   | 0.40  | 0.80  | [Futurlec 74HCT123](https://futurlec.com/74HCT/74HCT123pr.shtml)                                                                                 | datasheet |
| 25  | 74HCT4017 decade counter (shot counter units and tens; new). Price of the CD74HC4017E proxy          | 2   | 1.03  | 2.06  | [Digi-Key CD74HC4017E](https://www.digikey.com/en/products/detail/texas-instruments/CD74HC4017E/480317)                                          | estimated |
| 26  | 940 nm 5 mm IR LED (muzzle and breech beams; new)                                                    | 2   | 0.30  | 0.60  | [Core Electronics](https://core-electronics.com.au/super-bright-5mm-ir-led-940nm.html.plain.md)                                                  | estimated |
| 27  | 940 nm 5 mm IR phototransistor (beams; new)                                                          | 2   | 0.30  | 0.60  | [ProtoSupplies](https://protosupplies.com/product/phototransistor-940nm/)                                                                        | estimated |
| 28  | WS2812B 5050 status LED (new)                                                                        | 1   | 0.75  | 0.75  | [SparkFun 24837](https://www.sparkfun.com/products/24837)                                                                                        | datasheet |
| 29  | Resistors 1/4 W (30 in the wiring model)                                                             | 30  | 0.05  | 1.50  | [Adafruit 2784](https://www.adafruit.com/product/2784) in quantity                                                                               | estimated |
| 30  | Radial electrolytic capacitors 16 V (3.3 u, 4.7 u, 10 u, 1 u; 4 in the wiring model)                 | 4   | 0.10  | 0.40  | [Adafruit 2784 listing, assortment](https://www.adafruit.com/product/2784) [est]                                                                 | estimated |
| 31  | M3 fastener kit, 522 pcs                                                                             | 1   | 8.33  | 8.33  | [SCC Ltd](https://scccltd.com/products/522pcs-m3-m4-hex-socket-screw-nut-kit), HK$65 / 7.8, stock unverified                                     | derived   |
| 32  | M3 brass heat-set inserts, pack of 50 (CAD needs about 36: see "Fasteners")                          | 1   | 5.00  | 5.00  | [est] TME about $0.10 each (search snippet)                                                                                                      | estimated |
| 33  | M4 brass heat-set inserts (about 14: Y1 yoke docks 8, Brio 2, lazy-susan posts 4)                    | 14  | 0.375 | 5.25  | [est] allowance, no listing fetched                                                                                                              | estimated |
| 34  | USB cable (Pico to Jetson) and DC cables: one allowance line                                         | 1   | 4.00  | 4.00  | [est] allowance, no listing                                                                                                                      | estimated |
| 35  | Hookup wire (12 to 28 AWG) and connectors                                                            | 1   | 6.00  | 6.00  | [est] allowance, no listing                                                                                                                      | estimated |
| 36  | Buzzer (GP17, unwired) and spare LEDs                                                                | 1   | 2.00  | 2.00  | [est] allowance                                                                                                                                  | estimated |
| 37  | Filament: 940.8 g PETG/TPU at $20/kg (CAD build `build/cad/cuas2/manifest.json`, 34.7 h of printing) | 1   | 18.81 | 18.81 | [est] $20/kg price assumption, as v1                                                                                                             | estimated |

## Totals (target: hardware + filament <= $190, excludes the Jetson, Brio, rounds and the 12 V brick)

- Hardware, lines 1-36: **$185.79**.
- Filament, line 37: **$18.81**.
- **Hardware + filament = 185.79 + 18.81 = $204.60, which is $14.60 over the $190 target** (explained below; script output: `37 lines, total 204.6`).

- Rounds (consumable, outside the total): Nerf Rival 100-round pack, $14.99 ([Nerf wiki](https://nerf.fandom.com/wiki/Rival_Round)).
- Optional 12 V 5 A brick: $24.95 ([Adafruit 352](https://www.adafruit.com/product/352)), not counted.

The sum is recomputed from the table by the script in "Check" below; the total above is that script's output.

## Why the total is over $190

Spec section 9 estimated hardware at $167.13 and filament at $19.99 (total about $187.12, all [est]). With fetched prices the hardware was $192.05 (+$24.92; $185.79 after the 2205 motor lever below) and filament $18.81 (-$1.18). The biggest lines are the new
ones: two 30 A ESCs $29.98 and two 2205 motors $19.98 ($49.96 together; 2207 motors were $26.24), the 40 A relay with socket $8.96, the second SG92R $5.95, and the added safety electronics (supervisor and limiter ICs, counter ICs, L7805, P-FET,
clamp, beams, resistors and capacitors: lines 19, 21, 22, 24-27, 29, 30 = $7.84, including the hardware shot counter added by the orchestrator, spec 14). Filament is $18.81 for a 941 g build (v1: 1099.7 g).

Levers (looked up 2026-10-27; the first three were proposed to reach $184.55):

- **Applied: 2205 2300 KV motors** (line 6), saving 2 x (13.12 - 9.99) = **6.26**. Launcher check (kv 2300, 12 V, droop 0.10, python/napl/cuas2/launcher.py model): 18144 rpm (the 38 m/s cap) needs u = 0.73 and rpm_max 17500 needs u = 0.70, both under throttle_max 0.90; the 2207 1800 KV needed u = 0.93 for the cap, so the 2205 has more headroom. Price is a search snippet for an out-of-stock listing (fallback: RS2205 4-pack USD 28.59 = 7.15 each, eBay snippet); mount pattern is listed 16 x 19 mm by [thingbits](https://www.thingbits.in/products/rs2205-brushless-motor-cw-ccw) against the CAD's 16 x 16: verify before printing. The model run (hfbox) gives 28.6 m/s mean at 17500 rpm for either KV; 2207 1800 KV needs u = 0.90 for rpm_max, i.e. no throttle headroom under throttle_max 0.90 at droop 0.10, which the spec's 8-14 % claim did not show. No-load runaway is 29.0k rpm (spec section 1 failure table, row 17), still caught by the overspeed fault.
- **Not applied: $8-class ESC.** No single 30 A ESC at about $8 was found that is in stock, 2-4S/3S capable, DShot600 and with a stated Bluejay target. Found: [RDQ 30A 3-6S BLHeli_S DShot600](https://www.racedayquads.com/products/rdq-esc-30a-blheli_s-dshot600) $13.49 in stock (would save 2 x 1.50 = 3.00) but the page names no Bluejay layout; [HSKRC 30A 2-4S](https://www.banggood.com/HSKRC-Brushless-20A-or-30A-or-35A-or-45A-2-4S-BLHeli-S-ESC-Speed-Controller-Oneshot-DSHOT600-for-FPV-Racing-RC-Drone-reviews-p1824526.html) $12.34 for the 20/30/35/45 A listing (30 A price unclear, Bluejay not stated); [Evelta 30A solder version](https://evelta.com/30a-dshot-brushless-blheli_s-esc-for-fpv-drone-solder-version/) INR 1417.5 (about $16), 2-5S, EFM8BB21, BLHeli_S. The Bluejay README says only that it flashes "BLHeli_S compatible ESCs" (EFM8 BB21/BB51) with bidirectional DShot and RPM telemetry; it publishes no per-product list, so every candidate needs its layout checked at flash time. The Tiger Motor line stays (its GetFPV page was not fetchable, 403; the part file records it as out of stock).
- **Not applied: drop the M3 kit.** Single-size packs cost about as much as the kit: Zoro M3x10 x100 $6.19, Global Industrial M3x8 x100 $9.80 (search snippets), plus M4 screws and nuts the kit also provides; no cheaper priced listing, and the CAD does not count screws (only inserts). Needed from the kit: roughly 36 M3 screws for the inserts plus 8 for the NEMA mounts and 8 for the motors, 14 M4 screws, and nuts [est, not counted by the CAD].
- Drop the WS2812: **0.75** (status can use the Pico LED), not applied.
- A 4-in-1 ESC board for the pair (spec 9 lever: $2 to $4 less) changes the two-ESC wiring (not applied); the DarwinFPV Bluejay 4-in-1 was $35.99 in a search snippet, more than the two singles.

Net: 210.86 - 6.26 = $204.60, still $14.60 over $190.

## Fasteners

Heat-set inserts, counted from the CAD by instrumenting `hole()` for 4.2 mm (M3) and 5.6 mm (M4) holes over every printed part and multiplying by the part count: **22 M3 and 4 M4** from direct holes
(base plate 8 M3 + 4 M4, base legs 4 x 1, lower deck 4, cartridge cage 2, power box 4). The dock interfaces add pilot holes (D1 dovetail 2 per site, F1 flange 4, H1 power-box 2 and head 2, D2 camera 2 M4, Y1 yoke 8 M4);
that is about 14 M3 and 10 M4 on top. Total about **36 M3 and 14 M4**: the 50-piece M3 pack covers it.

## Not in the BOM

Jetson Orin Nano devkit ($399), Logitech Brio, foam rounds, 12 V supply, the TPU tyre filament is inside the filament line (18 g), optional TF-Luna rangefinder (pocket in the sensor bar only), the GT2 belt teeth tooling.

## Check

```
python3 - <<'PY'
import re
rows=[l for l in open("docs/cuas2/bom.md") if re.match(r"\| \d+ +\|", l)]
tot=0.0
for l in rows:
    c=[x.strip() for x in l.strip().strip("|").split("|")]
    q,u,line=float(c[2]),float(c[3]),float(c[4])
    assert abs(q*u-line)<0.006, (c[0], q*u, line)
    assert "http" in c[5] or "[est]" in c[5], c[0]
    tot+=line
print(len(rows),"lines, total",round(tot,2))
PY
```
