# cuas-lite bill of materials (hardware + filament, ammo separate)

Prices in USD, looked up 2026-10-11. Fidelity: **datasheet** = a price read from the vendor page, **derived** = converted from another currency at an assumed rate (1.15 USD/EUR, 1.33 USD/GBP, 7.8 HKD/USD, 120 BDT/USD, none looked up),
**estimated** = a search snippet, an unfetchable page, or an allowance with no listing. Part ids are in `parts/`; the assembly is `assemblies/cuas-lite.toml`.
Excluded from the totals (user's rule): the Jetson Orin Nano, the Logitech Brio, the foam rounds (consumable, shown separately).

| # | Part | Qty | Unit | Line | Source | Fidelity |
|---|------|-----|------|------|--------|----------|
| 1 | NEMA 17 17HS4401S 42x40 (pan, tilt) | 2 | 8.11 | 16.22 | [Handson Tech sheet](https://www.handsontec.com/dataspecs/17HS4401S.pdf); EUR 7.05 (bitsandparts.nl) | derived |
| 2 | BTT TMC2209 V1.3 driver | 2 | 7.89 | 15.78 | [BIQU](https://biqu.equipment/collections/driver-board/products/bigtreetech-tmc2209-stepper-motor-driver-for-3d-printer-board-vs-tmc2208) | datasheet |
| 3 | Raspberry Pi Pico H | 1 | 5.00 | 5.00 | [Adafruit 5525](https://www.adafruit.com/product/5525) | datasheet |
| 4 | GT2 20T pulley, 5 mm bore, 6 mm belt (the 60T is dropped: pan 100T and tilt 80T pulleys are printed) | 2 | 0.92 | 1.84 | [Olimex GT2-5T20-6MM](https://olimex.com/Products/Robot-CNC-Parts/GT2/GT2-5T20-6MM), 0.80 EUR, "In Stock" (Adafruit 1251 is no longer stocked, $7.95) | derived |
| 5 | GT2 6 mm closed belt loops: 252 mm (pan), 204 mm (tilt) | 2 | 3.00 | 6.00 | no loop of these lengths has a fetched price (VXB lists 200 mm at an inconsistent $99.99/$150; Studica and an eBay UK listing could not be read) | estimated |
| 6 | 180 brushed motor, 6 V (flywheels) | 2 | 1.58 | 3.16 | [othoba.com](https://othoba.com/180-high-speed-motor-strong-magnetic-carbon-brushed-180-engine-high-torque-180-motor-with-strong-magnetic-force-for-various-applications-noor-electronics-1033075), Tk 190 | derived |
| 7 | 35x7 mm rubber wheel (flywheel) | 2 | 0.69 | 1.38 | [Sigmanortec](https://sigmanortec.ro/en/35mm-x-7mm-wheel-n20-motor), 0.60 EUR | derived |
| 8 | TowerPro SG92R servo (pusher) | 1 | 5.95 | 5.95 | [Adafruit 169](https://www.adafruit.com/product/169) | estimated |
| 9 | A3144 hall switch (pan, tilt) | 2 | 0.52 | 1.04 | [Tinytronics](https://www.tinytronics.nl/en/sensors/magnetic-field/3144-hall-effect-switch), 0.45 EUR | derived |
| 10 | Neodymium magnet disc 6x2 mm | 2 | 0.50 | 1.00 | allowance (no listing fetched) | estimated |
| 11 | 608ZZ bearing (left trunnion) | 1 | 0.80 | 0.80 | [Olimex](https://www.olimex.com/Products/Robot-CNC-Parts/Bearings/608ZZ/), 0.70 EUR | derived |
| 12 | 6807ZZ bearing 35x47x7 (right trunnion, hollow) | 1 | 7.49 | 7.49 | Newegg 6807-2RS from a search snippet (page 403); an eBay seller at about 3.09 | estimated |
| 13 | 8 mm x 100 mm stainless rod (left 608 stub) | 1 | 3.49 | 3.49 | [goBILDA 2100-0008-0100](https://www.gobilda.com/stainless-steel-round-shafting/) (the right rod is gone with the 6807ZZ) | datasheet |
| 14 | 6 in square lazy susan (see "Lazy susan") | 1 | 4.95 | 4.95 | [RF Parts](https://www.rfparts.com/lazy-susan.html), "buy 6 for 4.50", limited quantity (Hardware Tree XLAZSUSN6Z is $12.95) | datasheet |
| 15 | E-stop, red mushroom 22 mm, 1NO-1NC (NC used) | 1 | 5.90 | 5.90 | [Maplin Pro](https://pro.maplin.co.uk/products/red-mushroom-22mm-emergency-stop-switch-no-nc), GBP 4.44, 128 in stock (VAT status not stated) | derived |
| 16 | Key switch E-Switch KO106A102 (SPST, 1 A) | 1 | 3.94 | 3.94 | [Digi-Key](https://www.digikey.com/en/products/detail/e-switch/KO106A102/EG2625-ND/502089) | datasheet |
| 17 | Songle SRD-12VDC-SL-C relay (weapon rail) | 1 | 0.44 | 0.44 | [LCSC C30431](https://www.lcsc.com/product-detail/C30431.html), $0.4421 | datasheet |
| 18 | Inline ATC fuse holder 12 AWG + 5 A fuse | 1 | 4.00 | 4.00 | holder $3.00 on an unreadable Amazon page, fuse $1.00 allowance | estimated |
| 19 | XT60 pair (inlet) | 1 | 2.00 | 2.00 | KiwiQuads 3.49 NZD | derived |
| 20 | 12 V 5 A brick, 5.5x2.1 mm | 1 | 24.95 | 24.95 | [Adafruit 352](https://www.adafruit.com/product/352), 7 in stock (library part `facmogu-12v-5a-psu`, $23 estimated, is $1.95 cheaper but not verified in stock) | datasheet |
| 21 | LM2596 buck module (5 V weapon rail) | 1 | 1.90 | 1.90 | [ebuildelectronics](https://ebuildelectronics.odoo.com/shop/dc-dc-buck-converter-lm2596-adjustable-69), 109 PHP | derived |
| 22 | IRLB8721 MOSFET (one per flywheel) | 2 | 2.25 | 4.50 | [Adafruit 355](https://www.adafruit.com/product/355) | datasheet |
| 23 | 1N5822 Schottky diode (flyback) | 2 | 0.34 | 0.68 | [TME](https://www.tme.com/us/en-us/details/1n5822-e3_54/tht-schottky-diodes/vishay/) | datasheet |
| 24 | Resistors 1/4 W (13 in the wiring: key / E-stop sense dividers, EN pull-ups, gate pull-down) | 13 | 0.05 | 0.65 | [Adafruit 2784](https://www.adafruit.com/product/2784) in quantity | estimated |
| 25 | M3 fastener kit, 765 pcs (US seller) | 1 | 14.00 | 14.00 | eBay search snippet; the in-stock 522-pc SCC kit (HK$65, $8.33) is flagged both sold out and in stock | estimated |
| 26 | M3 brass heat-set inserts, 100 pcs (about 40 used: see "Fasteners") | 1 | 10.00 | 10.00 | TME about $0.10 each at 100 (search snippet); Adafruit 4256 (50 pack, $5.95) is out of stock | estimated |
| 27 | USB cable, Pico micro-USB to Jetson (the Brio brings its own) | 1 | 2.50 | 2.50 | allowance | estimated |
| 28 | DC cables: barrel plug to the Jetson (from the 12 V bus), brick pigtail to the XT60 (2 pcs) | 2 | 2.00 | 4.00 | allowance | estimated |
| 29 | Status LEDs (3) and buzzer (GP15-17, not in the wiring model) | 1 | 2.00 | 2.00 | allowance | estimated |
| 30 | Hookup wire (silicone 18/22/24/26/28 AWG, about 17 m in the wiring model) and connectors | 1 | 10.00 | 10.00 | allowance (spec.md budget line "wire/connectors $10") | estimated |

**Hardware total: $165.56** (items 1-30).
**Filament: $21.97** (CAD build 8fa7ecc: 29 printed parts + 1, 1098.6 g at $20/kg).
**Hardware + filament: $187.53.** This is **$37.53 over the $150 target.**
**Rounds (consumable, outside the hardware total):** Nerf Rival 100-round pack, $14.99 ([Nerf wiki / retailer listings](https://nerf.fandom.com/wiki/Rival_Round)). With rounds: **$202.52.**

## Where the money is, and the levers (not applied)

Lean alternatives, all already researched: 12 V brick at the facmogu $23 (-1.95), M3 kit at the SCC $8.33 (-5.67), a 50-piece insert pack at about $5 (-5), wire and connectors at $6 (-4), USB and DC cable allowances at $4 (-2.50)
brings the total to about $168. The $150 target is not reachable with this parts list without dropping a function (the relay weapon rail is $0.44 and stays; the Pico is $5). The two largest single lines are the brick ($24.95)
and the stepper pair + drivers ($32.00).
## Added since the last BOM

Relay (17), 13 resistors (24), USB and DC cables (27, 28), LED and buzzer allowance (29), the 6807ZZ (12) with one 608 and one rod fewer, the 20T pulleys at the cheaper Olimex price and no 60T pulley (4), the E-stop at the Maplin price (15),
the pan stop parts (below).

## Fasteners for the pan hard stop

`pan_stop_stud` (M3 x 14 threaded stud in a heat-set insert in the stator) and `pan_stop_tab` (printed, 1.2 g, two countersunk M3 screws into two inserts in the rotor underside): the SCC 522-piece kit lists M3 screws in 8, 10, 12 and 16 mm,
flat-head and round-head, so the countersunk screws are covered; **a 14 mm length is not in that kit**, so the stud is an M3 x 16 screw (kit) cut to length, or an M3 x 14 from the 765-piece kit (item 25, contents not read). The three
inserts (one stator, two rotor) come from item 26. Insert count for the whole turret is a CAD hole count estimate (about 40), the pack of 100 covers it.

## Lazy susan

The 6 in square lazy susan has **no published fixing-hole pattern**: RF Parts gives size, thickness (5/16 in), "INSIDE HOLE DIAMETER 4 5/8 in" (117.5 mm) and 350 lb; Simply Bearings LAZY/6 gives pitch circle 121 mm,
inner diameter 119.1 mm and 0.21825 kg; Hardware Tree nothing. The 121 mm "pitch circle" cannot be a bolt circle (radius 60.5 mm would cut into the 119.1 mm opening). **CAD parameter: `LS_BOLT_R` stays an estimate**;
measure the bolt-hole radius on the delivered bearing and set `LS_BOLT_R` in `cad/cuas-lite/params.py` (the printed rotor plate and stator posts carry the holes). In the design the bearing is the part
`generic-lazy-susan-6in-square`, collided as a four-slab square ring with the 117.5 mm opening, mass 218 g (CAD assumes 330 g), mounted in the pan group at z = 18 mm.

## CAD mismatches found while building the assembly (for the CAD worker)

These four envelope clashes are the only DRC findings left; they are waived in `assemblies/cuas-lite.toml` by `waived--*` closure mates (through `clash_*` ports on the beam, E-stop and hall mount) and are real:

1. **Trunnion beam against the flywheel motors:** the CAD models the 180 motor 25 mm long, the listing says 32 mm (can 32 mm, 20 x 15.3 mm), so the can runs 1.7 mm (-y) and 4.0 mm (+y) into the beam.
2. **E-stop against the pan NEMA 17:** the 35 mm contact block (estimated) behind the front-panel hole at y = -28 mm reaches 3.6 mm into the motor body (half-width 21 mm at x 40-81 mm).
   Move the E-stop and key holes outward (or shorten the block).
3. **Lazy susan against the pan hall mount:** the 6 in plate edge (x = -76.2 mm) against the sensor pocket (x -85..-75 mm, z up to 24 mm), 1.2 mm.
4. The relay was moved (x 78, y -27 mm, turned 90 deg) so that it is clear of the tray rim; no CAD change needed.

Other mismatches (no DRC finding): Jetson devkit 103 x 90.5 x 34.8 mm against the CAD block 100 x 79 x 21 mm; 17HS4401S body 40 mm (listing 40) against CAD 38 mm; lazy-susan mass 218 g against 330 g and its
opening 117.5 mm against the CAD aperture 92 mm; the hall magnet pocket size is unknown; the key and E-stop hole diameters are catalogue values; the CAD pulley hub z.

## Not in the totals

Jetson Orin Nano devkit ($399), Logitech Brio, foam rounds ($14.99 per 100, shown above), tools, soldering consumables, printer time.
