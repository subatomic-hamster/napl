"""S3: a complete X500-class quad built FROM AN EMPTY DESIGN through the editing API alone (Design.new + the same ops as MCP
design_edit): no assembly TOML is read or written by hand. Parts come from library queries, mechanics from add-to-port snapping
(port names, never poses), power wiring from autowire_all, the four PWM channels (ambiguous by design) from explicit autowire
calls, sensors / ESC / rotor / battery devices from the device ops. Checks: DRC 0 errors, hover passes, hover current and mass
agree with assemblies/x500v2.toml (the reference, loaded only for the comparison), save + reload gives the identical DRC."""

import asyncio
from pathlib import Path

import napl
import pytest
from mcp import Client
from napl import design
from napl.design import Design, DesignError
from napl.mcp_server import NaplServer
from test_design_mcp import accept, call, ok, root  # noqa: F401  (root is a fixture)

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
X500 = REPO / "assemblies" / "x500v2.toml"
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")

# quad-X, PX4 numbering: motor k sits on arm port ARM[k] of the bottom plate; k = 1, 2 spin CCW, 3, 4 CW
ARM = {1: "arm4", 2: "arm2", 3: "arm1", 4: "arm3"}
REVERSE = {1: False, 2: False, 3: True, 4: True}


def est(v, unit, why):
    return {"value": v, "unit": unit, "fidelity": "estimated", "rationale": why}


def derived(v, unit, how):
    return {"value": v, "unit": unit, "fidelity": "derived", "derivation": how}


def ids(rows):
    return [r["part"] for r in rows]


def pick(rows, want, why):
    """Choose from a library query / suggest() result; the choice must be one of the candidates the API offered."""
    assert want in ids(rows), f"{want} not offered for {why}: {ids(rows)[:8]}"
    return want


def build_scratch(parts=PARTS) -> Design:
    d = Design.new(
        "scratch-quad", parts, description="X500-class quad built from an empty design"
    )
    lib = d.lib
    # ---- 1. frame: the root plate, then everything snaps to its named open ports ------------------------------------
    plate = pick(
        [
            {"part": p.id}
            for p in lib.by_category("frame.plate")
            if p.has_port("bolt-pattern.x500v2-arm-clamp.m3")
        ],
        "holybro-x500v2-bottom-plate",
        "bottom plate (the plate with six arm seats)",
    )
    d.add(plate, id="bottom", root=True, group="frame")
    clamp = {
        "part": "iso4762-m3x38",
        "count": 4,
        "nut": "m3-flange-locknut",
    }  # through the plate + connector + top plate
    con = pick(
        d.suggest("bottom.arm4", category="frame.connector"),
        "holybro-x500v2-arm-connectors",
        "bottom.arm4",
    )
    for k, arm in ARM.items():
        d.add(
            con,
            id=f"con{k}",
            to=f"bottom.{arm}",
            port="plate_lo",
            group="frame",
            mate=f"clamp{k}",
            fasteners=clamp,
        )
    top = pick(
        d.suggest("con1.plate_hi", category="frame.plate"),
        "holybro-x500v2-top-plate",
        "con1.plate_hi",
    )
    # the top plate is clamped by the same M3x38 bolts as the bottom one (the clamp mates above): no fasteners of its own
    d.add(
        top,
        id="top",
        to="con1.plate_hi",
        port="arm4",
        group="frame",
        mate="top-seat1",
        fasteners=None,
    )
    for k in (
        2,
        3,
        4,
    ):  # the plate is already placed: the other three seats close loops
        d.mate(
            f"con{k}.plate_hi",
            f"top.{ARM[k]}",
            closure=True,
            name=f"top-seat{k}",
            fasteners=None,
        )
    tube = pick(
        d.suggest("con1.tube_body", category="frame.tube"),
        "holybro-x500v2-arm-tube",
        "con1.tube_body",
    )
    for k in ARM:
        d.add(
            tube,
            id=f"tube{k}",
            to=f"con{k}.tube_body",
            port="body_clamp",
            group="frame",
            mate=f"tube-body{k}",
        )
        d.mate(
            f"con{k}.tube_end",
            f"tube{k}.end_clamp",
            closure=True,
            name=f"tube-end{k}",
        )
    # ---- 2. propulsion: motor, prop (spin joint), ESC on the tube ------------------------------------------------------
    motor = pick(
        d.suggest("con1.motor", category="motor.bldc"),
        "holybro-2216-kv920",
        "con1.motor",
    )
    esc = pick(
        d.suggest("tube1.esc", category="esc"),
        "holybro-x500v2-esc-blheli-s-20a",
        "tube1.esc",
    )
    for k in ARM:
        d.add(
            motor,
            id=f"mot{k}",
            to=f"con{k}.motor",
            group="propulsion",
            mate=f"motor{k}",
        )
    prop = pick(
        d.suggest("mot1.shaft", category="prop"), "holybro-1045-prop", "mot1.shaft"
    )
    for k in ARM:
        d.add(
            prop,
            id=f"prop{k}",
            to=f"mot{k}.shaft",
            group="propulsion",
            mate=f"spin{k}",
            joint={
                "type": "hinge",
                "spin": True,
                **({"reverse": True} if REVERSE[k] else {}),
            },
        )
        d.add(
            esc,
            id=f"esc{k}",
            to=f"tube{k}.esc",
            group="propulsion",
            mate=f"esc-seat{k}",
        )
    # ---- 3. power + avionics on the plates ------------------------------------------------------------------------------
    pdb = pick(
        d.suggest("bottom.pdb", category="pcb.pdb"), "holybro-pdb-60a", "bottom.pdb"
    )
    d.add(pdb, id="pdb", to="bottom.pdb", group="power", mate="pdb-mount")
    pm = pick(
        d.suggest("bottom.pm02", category="pcb.pm"), "holybro-pm02-v3", "bottom.pm02"
    )
    d.add(pm, id="pm02", to="bottom.pm02", group="power", mate="pm02-seat")
    fc = pick(d.suggest("top.fc", category="pcb.fc"), "holybro-pixhawk-6c", "top.fc")
    d.add(fc, id="fc", to="top.fc", group="avionics", mate="fc-seat")
    d.add(
        "holybro-sik-telemetry-radio-v3",
        id="sik",
        to="top.sik",
        group="avionics",
        mate="sik-seat",
    )
    # ---- 4. the rest of the frame -----------------------------------------------------------------------------------------
    d.add(
        "holybro-x500v2-platform-board",
        id="platform",
        to="bottom.platform",
        group="frame",
        mate="platform-seat",
    )
    d.add(
        "holybro-x500v2-gps-mast",
        id="mast",
        to="platform.mast",
        group="frame",
        mate="mast-seat",
    )
    gps = pick(
        d.suggest("mast.tray", category="pcb.gps"), "holybro-m8n-gps", "mast.tray"
    )
    d.add(gps, id="gps", to="mast.tray", group="avionics", mate="gps-seat")
    d.add(
        "holybro-x500v2-battery-mount",
        id="batmount",
        to="bottom.battery_mount",
        group="frame",
        mate="batmount-seat",
    )
    d.add(
        "holybro-x500v2-hardware-kit",
        id="hw",
        to="bottom.hardware",
        group="frame",
        mate="hardware-seat",
    )
    for side, port in (("f", "leg_front"), ("r", "leg_rear")):
        d.add(
            "holybro-x500v2-landing-gear",
            id=f"leg_{side}",
            to=f"bottom.{port}",
            group="frame",
            mate=f"leg-{'front' if side == 'f' else 'rear'}",
        )
    for i in range(1, 9):
        d.add(
            "holybro-x500v2-hanger",
            id=f"hanger{i}",
            to=f"bottom.hanger{i}",
            group="frame",
            mate=f"hanger{i}-seat",
        )
    for s, rail in (("l", "rail_l"), ("r", "rail_r")):
        d.add(
            "holybro-x500v2-rail-rod",
            id=f"rod_{s}",
            to=f"bottom.{rail}",
            group="frame",
            mate=f"rod-{s}",
        )
    bat = pick(
        _batteries(d),
        "tattu-4s-5200mah-35c",
        "4S pack with an XT60 lead on the rail pair",
    )
    d.add(
        bat, id="bat", to="rod_l.slide", port="rail_l", group="power", mate="battery-l"
    )
    d.mate("rod_r.slide", "bat.rail_r", closure=True, name="battery-r")
    # sensor chips live on the boards (no body, no mass of their own): offboard instances the device bindings refer to
    for iid, pid in CHIPS.items():
        d.add(pid, id=iid, offboard=True)
    d.add("matek-elrs-r24-s", id="rx", offboard=True)
    # ---- 5. devices (before the wires: the DRC's current model then sizes every wire) ---------------------------------------
    bind_devices(d)
    # ---- 6. wiring: power by autowire_all; the motor leads and the four PWM channels are ambiguous by design (which motor
    # is which channel is the caller's call), so autowire_all reports them as two decisions and does not guess -----------------
    # datasheet masses that already contain a lead (pack: 120 mm XT60 lead, PM02: pre-soldered leads + BEC cable, ESC: input
    # cable + signal lead, motor: 150 mm phase leads, GPS: 26 cm cable, radio: its cable): those wires add no mass
    inc = {
        "bat.power_out": "bat",
        "pm02.power_out": "pm02",
        "pm02.fc_power": "pm02",
        "gps.gps_out": "gps",
        "sik.telem": "sik",
        **{f"esc{k}.power_in": f"esc{k}" for k in ARM},
    }
    r = d.autowire_all(included_in=inc).changes[0]
    for k in ARM:
        d.autowire(f"mot{k}.phase", f"esc{k}.motor_out", included_in=f"mot{k}")
        d.autowire(f"fc.pwm{k}", f"esc{k}.signal", included_in=f"esc{k}")
    return d, r


CHIPS = {
    "icm": "tdk-icm-42688-p",
    "baro_chip": "te-ms5611-01ba03",
    "mag_chip": "isentek-ist8310",
    "gnss_chip": "ublox-neo-m8n",
}
REF = "same value as assemblies/x500v2.toml (the reference this build is compared with)"


def bind_devices(d):
    maps = {
        "esc2216": {
            "motor.kv": "motor:kv",
            "motor.r20": "motor:r_winding",
            "motor.i0": "motor:i0",
            "motor.rotor_inertia": "motor:j_rotor_bell",
            "i_limit": "current_continuous",
            "r_on": "r_on_total",
            "sw_loss": "switching_loss_fraction",
            "i_q": "quiescent_current",
        },
        "prop1045": {
            "diameter": "diameter",
            "j": "@ct_cq_j.j",
            "ct": "@ct_cq_j.ct",
            "cq": "@ct_cq_j.cq",
        },
        "tattu": {
            "capacity_ah": "capacity_ah",
            "r0": "r_internal_pack",
            "r1": "thevenin_r1",
        },
        "pm02_board": {"resistance": "r_series"},
        "pdb_path": {"resistance": "r_path"},
        "icm42688": {
            "gyro.noise_density": "gyro_noise_density",
            "gyro.full_scale": "gyro_fs_max",
            "gyro.bits": "gyro_adc_bits",
            "accel.noise_density": "accel_noise_density_xy",
            "accel.full_scale": "accel_fs_max",
            "accel.bits": "accel_adc_bits",
        },
        "ms5611": {"noise_std": "pressure_rms_noise_osr4096"},
        "ist8310": {"noise_std": "rms_noise"},
        "m8n": {"rate_hz": "max_nav_rate_gps_glonass", "vel_std": "velocity_accuracy"},
    }
    for name, m in maps.items():
        d.device_map(name, m)
    bind = lambda **dev: d.bind(**dev)
    bind(
        type="battery",
        name="battery",
        instance="bat",
        port="power_out",
        ocv="lipo-cell-ocv-chen2006",
        map="tattu",
        set={
            "c1": est(
                412.5,
                "F",
                "tau_RC / r1 = 3.3 s / 0.008 ohm; estimated because r1 is. " + REF,
            )
        },
    )
    bind(
        type="wire",
        name="pm02_board",
        instance="pm02",
        a="power_in",
        b="power_out",
        map="pm02_board",
    )
    for k in ARM:
        bind(
            type="wire",
            name=f"pdb_path{k}",
            instance="pdb",
            a="power_in",
            b=f"out{k}",
            map="pdb_path",
        )
    for name, i0, who in (
        ("load_fc", -0.1083, "Pixhawk 6C"),
        ("load_gps", -0.0124, "M8N GPS"),
        ("load_sik", -0.0258, "SiK radio"),
    ):
        bind(
            type="load",
            name=name,
            instance="pm02",
            port="power_in",
            set={
                "i0": derived(
                    i0,
                    "A",
                    f"{who}: supply current x 5.2 V BEC / (0.85 efficiency x 14.8 V); {REF}",
                )
            },
        )
    for k in ARM:
        bind(
            type="esc",
            name=f"esc{k}",
            instance=f"esc{k}",
            port="power_in",
            motor=f"mot{k}",
            joint=f"spin{k}",
            map="esc2216",
            set={
                "motor.c_th": est(
                    25.6,
                    "J/K",
                    "whole 64 g motor at ~0.4 J/(g K); no vendor value. " + REF,
                ),
                "motor.r_th": derived(
                    0.962,
                    "K/W",
                    "(80 degC vendor surface - 25 degC ambient) / (I_m^2 R(80 degC)) of the napl bench fit; "
                    + REF,
                ),
            },
        )
        bind(
            type="rotor",
            name=f"rotor{k}",
            instance=f"prop{k}",
            body=f"prop{k}",
            joint=f"spin{k}",
            map="prop1045",
            dir=-1 if REVERSE[k] else 1,
            ground_effect=True,
            reynolds=True,
            cq_ref=derived(
                0.005794,
                "1",
                "C_Q(J=0) of the T1045II from the 2216 thrust-stand fit (napl bench); "
                + REF,
            ),
            set={
                "k_h": est(
                    0.0, "kg", "no H-force data for the T1045II: term off. " + REF
                )
            },
        )
    bind(
        type="imu",
        name="imu",
        instance="icm",
        site="fc",
        map="icm42688",
        set={"rate_hz": est(500.0, "Hz", "every 2nd tick of the 1 ms step")},
    )
    bind(
        type="baro",
        name="baro",
        instance="baro_chip",
        site="fc",
        map="ms5611",
        set={
            "rate_hz": est(
                50.0, "Hz", "every 20th tick; MS5611 OSR 4096 converts in 8.2 ms"
            )
        },
    )
    bind(
        type="mag",
        name="mag",
        instance="mag_chip",
        site="gps",
        map="ist8310",
        set={
            "rate_hz": est(100.0, "Hz", "IST8310 ODR 1-200 Hz; mid value"),
            "earth_field": derived(
                [1.294e-6, 2.1756e-5, -4.3027e-5],
                "T",
                "WMM field at 47.3769 N, 8.5417 E (the origin PX4 assumes): H = 21.795 uT, decl 3.40 deg, incl 63.14 deg; "
                + REF,
            ),
        },
    )
    bind(
        type="gnss",
        name="gnss",
        instance="gnss_chip",
        site="gps",
        map="m8n",
        set={
            "latency_s": est(0.2, "s", "no published output latency; one 5 Hz sample"),
            "lat0": est(
                47.3769,
                "deg",
                "arbitrary simulation origin (Zurich, as tests/data/devices/all.xml)",
            ),
            "lon0": est(8.5417, "deg", "as lat0"),
            "alt0": est(408.0, "m", "as lat0"),
            "horiz_std": derived(
                2.1233, "m", "2.5 m CEP / 1.17741 = 2.1233 m per coordinate"
            ),
            "vert_std": derived(3.185, "m", "1.5 x horiz_std"),
            "corr_time_s": est(
                600.0, "s", "slowly varying GNSS error, minutes (literature range)"
            ),
            "corr_frac": est(
                0.9,
                "1",
                "share of the variance that is the Gauss-Markov part; estimate",
            ),
        },
    )
    bind(
        type="environment",
        name="environment",
        instance="gnss_chip",
        set={
            "origin_alt": est(408.0, "m", "same site as the GNSS origin"),
            "w20": est(0.0, "m/s", "calm air: turbulence off"),
        },
    )


def _batteries(d):
    """4S packs whose power lead is an XT60 and that slide on the 10 mm rails."""
    out = []
    for p in d.lib.by_category("battery.lipo"):
        if (
            p.has_port("xt60", "male")
            and p.has_port("tube.10", "female")
            and p.params.get("cells_series", {}).get("value") == 4
        ):
            out.append({"part": p.id})
    return out


# ---- checks ---------------------------------------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def scratch():
    return build_scratch()


def _codes(findings, sev):
    return sorted((f.code, tuple(f.where)) for f in findings if f.severity == sev)


def test_built_from_empty_with_no_toml_and_drc_has_no_errors(scratch):
    d, _ = scratch
    assert d.path is None  # never opened from a file
    errors = [f for f in d.check() if f.severity == "error"]
    assert errors == [], [(f.code, f.where, f.message[:120]) for f in errors]
    # peak (full throttle on all four) above a continuous rating is a warning by design; the grip warnings are the legs'
    # M3x8 in a 2 mm plate (same as the reference)
    assert {f.code for f in d.check() if f.severity == "warning"} <= {
        "CURRENT_PEAK_OVER_RATING",
        "FASTENER_GRIP",
    }
    n = {
        p: sum(1 for i in d.doc["instances"] if i["part"] == p)
        for p in (
            "holybro-2216-kv920",
            "holybro-1045-prop",
            "holybro-x500v2-esc-blheli-s-20a",
        )
    }
    assert set(n.values()) == {4}
    assert (
        len(d.doc["wires"]) == 9 + 8
    )  # autowire_all's nine + four motor leads + four PWM channels


def test_autowire_all_reports_each_ambiguous_decision_once(scratch):
    _, r = scratch
    assert [(x["ports"], x["candidates"]) for x in r["ambiguous"]] == [
        ([f"esc{k}.motor_out" for k in ARM], [f"mot{k}.phase" for k in ARM]),
        ([f"esc{k}.signal" for k in ARM], [f"fc.pwm{k}" for k in ARM]),
    ]
    # the receiver has no 5 V pad source anywhere in the library (the PM02 BEC is a JST-GH power cable): reported, not guessed
    assert {x["port"] for x in r["unmatched"]} <= {"rx.power_in"}
    # the PM02 -> PDB link joins two pass-through ports: wired because the battery already feeds the PM02
    assert {w["wire"]["name"] for w in r["wired"]} >= {"pm02.power_out--pdb.power_in"}


def test_empty_design_needs_a_root_first():
    d = Design.new("e", PARTS)
    assert d.check() == [] and d.doc["instances"] == []
    with pytest.raises(DesignError, match="no root yet"):
        d.add("holybro-pdb-60a")
    r = d.add("holybro-x500v2-bottom-plate", id="bottom", root=True)
    assert r.changes[0]["note"].startswith("root")
    with pytest.raises(DesignError, match="already has a root"):
        d.add("holybro-x500v2-top-plate", root=True)
    assert [i["id"] for i in d.doc["instances"]] == ["bottom"]  # nothing changed
    d.undo()  # the empty design is a valid undo target
    assert d.doc["instances"] == []


def test_suggest_filters_by_category_before_the_limit(scratch):
    d, _ = scratch
    # a flat seat is offered every board in the library; the filter asks for "an ESC for this seat"
    assert len({r["category"] for r in d.suggest("tube1.esc", limit=40)}) > 1
    rows = d.suggest("tube1.esc", limit=3, category="esc")
    assert 0 < len(rows) <= 3 and {r["category"] for r in rows} == {"esc"}


def test_mcp_builds_from_an_empty_design(root):
    async def agent():
        async with Client(NaplServer(root).server, elicitation_callback=accept) as c:
            o = await ok(c, "design_open", new="mcp-quad")
            assert o["instances"] == [] and o["path"] is None
            err, text = await call(c, "design_open")  # neither path nor new
            assert err
            r = await ok(
                c,
                "design_edit",
                ops=[
                    {
                        "op": "add",
                        "part": "holybro-x500v2-bottom-plate",
                        "id": "bottom",
                        "root": True,
                    },
                    {
                        "op": "add",
                        "part": "holybro-x500v2-arm-connectors",
                        "id": "con1",
                        "to": "bottom.arm4",
                    },
                ],
            )
            assert r["changes"][1]["mate"]["a"].startswith("bottom.arm4")
            s = await ok(
                c, "design_suggest", target="bottom.pm02", category="pcb.pm", limit=3
            )
            assert s["suggestions"] and all(
                x["category"] == "pcb.pm" for x in s["suggestions"]
            )

    asyncio.run(agent())


def test_save_and_reload_give_the_identical_drc(scratch, tmp_path):
    d, _ = scratch
    p = d.save(tmp_path / "scratch-quad.toml")
    d2 = Design.open(p)
    key = lambda fs: sorted((f.code, f.severity, tuple(f.where), f.message) for f in fs)
    assert key(d2.check()) == key(d.check())
    assert design.load(p) == d.asm


@pytest.fixture(scope="module")
def numbers(scratch, tmp_path_factory):
    d, _ = scratch
    ref = Design.open(X500)
    out = {}
    for name, des in (("scratch", d), ("x500v2", ref)):
        r = des.simulate("hover", out=tmp_path_factory.mktemp(name))
        rep = des.build(tmp_path_factory.mktemp(name + "-b"))["report"]
        wires = sum(w["mass"] or 0.0 for w in rep["wires"] if not w["included_in"])
        out[name] = {"sim": r, "mass": rep["mass"]["total"], "wire_mass": wires}
    return out


@needs_kernel
def test_hover_passes_and_matches_the_reference_assembly(numbers):
    s, x = numbers["scratch"], numbers["x500v2"]
    print()
    for k in ("scratch", "x500v2"):
        n = numbers[k]
        print(
            f"{k:8s} mass {n['mass']:.4f} kg (wires counted {n['wire_mass'] * 1000:.1f} g)  hover current "
            f"{n['sim']['hover_current_A']:.3f} A  hover throttle {n['sim']['hover_throttle']:.3f}  T/W {n['sim']['thrust_to_weight']:.2f}"
        )
    assert s["sim"]["pass"], s["sim"]
    assert x["sim"]["pass"], x["sim"]
    # what differs: the scratch build auto-picks the PDB screws (M3x6 + plain nut, the reference pins M3x14 + locknut), sizes
    # every wire from the DRC current (AWG 8 pack lead, the reference's is AWG 10) and carries a receiver (3 g, offboard here)
    assert abs(s["mass"] / x["mass"] - 1) < 0.01
    assert abs(s["sim"]["hover_current_A"] / x["sim"]["hover_current_A"] - 1) < 0.01
