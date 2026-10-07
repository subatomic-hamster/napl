"""Design-rule checks (napl.design.drc): the reference assemblies, and one seeded fault per rule code. A fault is injected
in memory (assembly document or an in-memory copy of the library), never into a file under assemblies/ or parts/."""

import copy
import json
import re
from pathlib import Path

import pytest
from napl import design
from napl.cli import main
from napl.design import Assembly, Library, check, drc, dumps

REPO = Path(__file__).resolve().parents[2]
PARTS = REPO / "parts"
X500 = REPO / "assemblies" / "x500v2.toml"
SO101 = REPO / "assemblies" / "so101.toml"


@pytest.fixture(scope="module")
def lib():
    return Library(PARTS)


@pytest.fixture(scope="module")
def x500():
    return design.load(X500)


@pytest.fixture(scope="module")
def so101():
    return design.load(SO101)


@pytest.fixture(scope="module")
def x500_findings(x500, lib):
    return check(x500, lib)


@pytest.fixture(scope="module")
def so101_findings(so101, lib):
    return check(so101, lib)


# ---- helpers ------------------------------------------------------------------------------------------------------------------
def prov(value, unit):
    return {
        "value": value,
        "unit": unit,
        "fidelity": "estimated",
        "rationale": "seeded fault for tests/python/test_drc.py",
    }


def mutated(asm, fn):
    d = copy.deepcopy(asm.doc)
    fn(d)
    return Assembly(d)


def by(d, section, key, value):
    return next(x for x in d[section] if x.get(key) == value)


def codes(fs, severity=None):
    return {f.code for f in fs if severity is None or f.severity == severity}


def fires(fs, code, also=()):
    """`code` is among the findings and no other error code is (except those in `also`, which the fault legitimately causes)."""
    assert code in codes(fs), f"{code} not found in {sorted(codes(fs))}"
    assert codes(fs, "error") <= {code, *also}, sorted(codes(fs, "error"))
    return [f for f in fs if f.code == code]


def lib_copy(lib):
    return copy.deepcopy(lib)


# ---- reference designs ----------------------------------------------------------------------------------------------------------
def test_x500_reference_has_no_errors(x500_findings):
    errs = [f for f in x500_findings if f.severity == "error"]
    assert errs == []
    assert "DRC_INTERNAL_ERROR" not in codes(x500_findings)
    # numbers: all-up 1.37 kg on four 2216 KV920 at 14.8 V; the vendor quotes ~18 min hover (no avionics load here)
    tw = next(f for f in x500_findings if f.code == "THRUST_TO_WEIGHT")
    assert (
        tw.severity == "info"
        and re.search(r"= (\d+\.\d+)$", tw.message)
        and 2.0 < float(re.search(r"= (\d+\.\d+)$", tw.message).group(1)) < 5.0
    )
    ft = next(f for f in x500_findings if f.code == "FLIGHT_TIME")
    assert 10 < float(re.search(r"= ([\d.]+) min", ft.message).group(1)) < 40
    assert {"HOVER_THROTTLE", "WIRE_RATING_UNKNOWN"} <= codes(x500_findings, "info")
    # worst case (all four ESCs at full throttle, 72 A) is above the continuous XT60 / 12 AWG ratings: warnings, not errors
    assert "CURRENT_PEAK_OVER_RATING" in codes(x500_findings, "warning")


def test_so101_reference_findings(so101_findings):
    # The one error class: the Waveshare board (an ESTIMATED 42 x 33 x 12 mm envelope) overlaps the base and the base motor holder
    # meshes by a few mm. It is a property of the placeholder envelope, reported rather than hidden.
    errs = {(f.code, tuple(f.where)) for f in so101_findings if f.severity == "error"}
    assert errs == {
        ("INTERFERENCE", ("adapter", "base")),
        ("INTERFERENCE", ("adapter", "base_motor_holder")),
    }
    w = codes(so101_findings, "warning")
    assert {
        "FASTENER_SHORT",
        "VOLTAGE_BELOW_MIN",
        "PSU_OVERLOAD",
        "SERVO_TORQUE",
        "CURRENT_PEAK_OVER_RATING",
    } <= w
    pl = next(f for f in so101_findings if f.code == "PAYLOAD_LIMIT")
    assert pl.severity == "info" and "g at moving_jaw" in pl.message
    assert "DRC_INTERNAL_ERROR" not in codes(so101_findings)


def test_findings_are_complete_records(x500_findings, so101_findings):
    for f in x500_findings + so101_findings:
        assert (
            f.code in drc.RULES
            and f.severity in ("error", "warning", "info")
            and f.message
        )
        assert set(f.as_dict()) == {"code", "severity", "message", "where", "fix"}
        if f.severity != "info":
            assert f.where, f.code


# ---- mechanical ----------------------------------------------------------------------------------------------------------------
def test_mate_invalid_wrong_port_type(x500, lib):
    fs = check(
        mutated(
            x500, lambda d: by(d, "mates", "name", "motor1").update(b="mot1.shaft")
        ),
        lib,
    )
    f = fires(fs, "MATE_INVALID", also=("FLOATING_INSTANCE",))
    assert "motor1" in f[0].where and f[0].fix


def test_floating_instance_missing_mate(x500, lib):
    fs = check(
        mutated(x500, lambda d: d["mates"].remove(by(d, "mates", "name", "esc-seat1"))),
        lib,
    )
    f = fires(fs, "FLOATING_INSTANCE")
    assert f[0].where == ["esc1"] and "esc1.base" in f[0].fix
    assert "CHECKS_SKIPPED" in codes(
        fs, "info"
    )  # the model cannot be built, so geometry / mass rules are skipped


def test_port_unmated_required(x500, lib):
    lib2 = lib_copy(lib)
    for p in lib2.get("holybro-2216-kv920").ports:
        if p["name"] == "base_19":
            p["required"] = True
    fs = check(x500, lib2)
    f = fires(fs, "PORT_UNMATED_REQUIRED")
    assert {w for x in f for w in x.where} == {f"mot{i}.base_19" for i in (1, 2, 3, 4)}


def test_fastener_thread_m2_screw_on_m3_pattern(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "mates", "name", "motor1").update(
                fasteners={"part": "iso4762-m2x6", "count": 4}
            ),
        ),
        lib,
    )
    f = fires(fs, "FASTENER_THREAD")
    assert "iso4762-m2x6" in f[0].message and "iso4762-m3x" in f[0].fix


def test_fastener_short(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "mates", "name", "clamp1").update(
                fasteners={
                    "part": "iso4762-m3x12",
                    "count": 4,
                    "nut": "m3-flange-locknut",
                }
            ),
        ),
        lib,
    )
    f = fires(fs, "FASTENER_SHORT")
    assert (
        f[0].severity == "warning"
        and "clamp1" in f[0].where
        and "iso4762-m3x" in f[0].fix
    )


def test_fastener_unavailable_too_many_screws(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "mates", "name", "motor1").update(
                fasteners={"part": "iso4762-m3x6", "count": 5}
            ),
        ),
        lib,
    )
    fires(fs, "FASTENER_UNAVAILABLE")


def test_interference_duplicate_part_on_the_same_seat(x500, lib):
    def f(d):
        d["instances"].append(
            {"id": "rod3", "part": "holybro-x500v2-rail-rod", "group": "frame"}
        )
        d["mates"].append(
            {"name": "rod3-seat", "a": "bottom.rail_l", "b": "rod3.hanger"}
        )

    fs = check(mutated(x500, f), lib)
    hits = fires(fs, "INTERFERENCE")
    assert {tuple(sorted(h.where)) for h in hits} >= {("rod3", "rod_l")}


def test_prop_overlap_with_oversize_prop(x500, lib):
    # adjacent motors are 0.354 m apart; a 0.5 m prop next to a 0.254 m prop reaches 0.377 m
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "prop1").update(
                params={"diameter": prov(0.5, "m")}
            ),
        ),
        lib,
    )
    f = fires(fs, "PROP_OVERLAP", also=("PROP_FRAME_CLASH",))
    assert any("prop1" in x.where for x in f) and "overlap" in f[0].message


def test_prop_frame_clash_with_huge_prop(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "prop1").update(
                params={"diameter": prov(0.9, "m")}
            ),
        ),
        lib,
    )
    hits = fires(fs, "PROP_FRAME_CLASH", also=("PROP_OVERLAP",))
    assert all("prop1" in h.where for h in hits)


def test_cg_offset_with_heavy_gps_mast(x500, lib):
    # a 1 kg GPS on its mast puts the CG 41 mm off the thrust centre: trimming takes 29 % of the thrust headroom (> 20 %)
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "gps").update(
                params={"mass": prov(1.0, "kg")}
            ),
        ),
        lib,
    )
    f = fires(fs, "CG_OFFSET", also=("CURRENT_OVER_RATING",))
    assert f[0].severity == "warning" and "mm from the thrust centre" in f[0].message


# ---- electrical ----------------------------------------------------------------------------------------------------------------
def test_wire_invalid_unknown_port(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "wires", "name", "pdb-to-esc1").update(
                **{"from": "pdb.nope"}
            ),
        ),
        lib,
    )
    fires(fs, "WIRE_INVALID", also=("FLOATING_PORT",))  # esc1 is left unpowered too


def test_wire_connector_mismatch_xt60_connector_between_xt30_ports(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "wires", "name", "pdb-to-esc1").update(part="amass-xt60"),
        ),
        lib,
    )
    f = fires(fs, "WIRE_CONNECTOR_MISMATCH")
    assert "xt60" in f[0].message


def test_wire_connector_mismatch_cable_end_family(so101, lib):
    # the STS3215 cable ends in 5264-3p male; the PSU port is a DC barrel
    fs = check(
        mutated(
            so101,
            lambda d: by(d, "wires", "name", "psu_cord").update(
                part="sts3215-servo-cable-3pin"
            ),
        ),
        lib,
    )
    assert "WIRE_CONNECTOR_MISMATCH" in codes(fs, "error")


def test_voltage_over_max_12s_pack(x500, lib):
    lib2 = lib_copy(lib)
    lib2.get("tattu-4s-5200mah-35c").ports[0]["v_max"] = {
        "value": 50.4,
        "unit": "V",
        "fidelity": "estimated",
        "rationale": "12S seeded fault",
    }
    fs = check(x500, lib2)
    # the 12S pack also scales the worst-case motor currents by (50.4 / 16)^2: BATTERY_C_RATING follows legitimately
    f = fires(fs, "VOLTAGE_OVER_MAX", also=("BATTERY_C_RATING",))
    assert {w.split(".")[0] for x in f for w in x.where[:1]} == {
        "esc1",
        "esc2",
        "esc3",
        "esc4",
    }
    assert "50.4" in f[0].message and f[0].fix


def test_voltage_below_min(x500, lib):
    lib2 = lib_copy(lib)
    for p in lib2.get("holybro-x500v2-esc-blheli-s-20a").ports:
        if p["name"] == "power_in":
            p["v_min"] = {
                "value": 20.0,
                "unit": "V",
                "fidelity": "estimated",
                "rationale": "seeded",
            }
    fs = check(x500, lib2)
    assert "VOLTAGE_BELOW_MIN" in codes(fs, "warning") and not codes(fs, "error")


def test_current_over_rating_22awg_on_the_battery_lead(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "wires", "name", "bat-to-pm02").update(
                part="awg-22-silicone-wire"
            ),
        ),
        lib,
    )
    f = fires(fs, "CURRENT_OVER_RATING")
    assert "awg-22" in f[0].message and "sustained" in f[0].message


def test_current_peak_over_rating_is_a_warning(x500_findings):
    f = [x for x in x500_findings if x.code == "CURRENT_PEAK_OVER_RATING"]
    assert f and all(x.severity == "warning" for x in f)


def test_battery_c_rating(x500, lib):
    # 35 C x 0.2 Ah = 7 A continuous, below the 10.7 A hover current
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "bat").update(
                params={"capacity_ah": prov(0.2, "Ah")}
            ),
        ),
        lib,
    )
    f = fires(fs, "BATTERY_C_RATING", also=("CURRENT_OVER_RATING",))
    assert f[0].severity == "error" and "35 C x 0.2 Ah" in f[0].message


def test_psu_overload(so101, lib):
    fs = check(
        mutated(
            so101,
            lambda d: by(d, "instances", "id", "psu").update(
                params={"i_out_rated": prov(0.5, "A")}
            ),
        ),
        lib,
    )
    f = [x for x in fs if x.code == "PSU_OVERLOAD"]
    assert f and f[0].severity == "error" and "rated 0.5 A" in f[0].message


def test_floating_port_unwired_esc(x500, lib):
    fs = check(
        mutated(
            x500, lambda d: d["wires"].remove(by(d, "wires", "name", "pdb-to-esc1"))
        ),
        lib,
    )
    f = fires(fs, "FLOATING_PORT")
    assert f[0].where == ["esc1.power_in"]


def test_net_no_source_when_the_battery_is_unplugged(x500, lib):
    fs = check(
        mutated(
            x500, lambda d: d["wires"].remove(by(d, "wires", "name", "bat-to-pm02"))
        ),
        lib,
    )
    f = fires(fs, "NET_NO_SOURCE")
    assert "pm02.power_in" in f[0].message


def test_wire_rating_unknown_is_info(x500_findings):
    f = [x for x in x500_findings if x.code == "WIRE_RATING_UNKNOWN"]
    assert f and all(x.severity == "info" for x in f)


# ---- performance ----------------------------------------------------------------------------------------------------------------
def test_thrust_to_weight_overweight_drone(x500, lib):
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "bat").update(
                params={"mass": prov(4.0, "kg")}
            ),
        ),
        lib,
    )
    f = fires(fs, "THRUST_TO_WEIGHT", also=("CG_OFFSET",))
    assert (
        f[0].severity == "error"
        and "< 1.3" in f[0].message
        and "motors with" in f[0].fix
    )


def test_thrust_to_weight_warning_band(x500, lib):
    # 2.4 kg: T/W = 44.7 / (2.4 x 9.81) = 1.9, between 1.3 and 2.0
    fs = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "bat").update(
                params={"mass": prov(1.45, "kg")}
            ),
        ),
        lib,
    )
    f = [x for x in fs if x.code == "THRUST_TO_WEIGHT"]
    # the hover current of the heavier drone also passes the 20 A rating of the 12 AWG lead: CURRENT_OVER_RATING is right
    assert f and f[0].severity == "warning"
    assert codes(fs, "error") <= {"CURRENT_OVER_RATING"}


def test_hover_throttle_and_flight_time_follow_mass(x500, lib, x500_findings):
    heavy = check(
        mutated(
            x500,
            lambda d: by(d, "instances", "id", "bat").update(
                params={"mass": prov(1.0, "kg")}
            ),
        ),
        lib,
    )
    t0 = float(
        re.search(
            r"= ([\d.]+) min",
            next(f for f in x500_findings if f.code == "FLIGHT_TIME").message,
        ).group(1)
    )
    t1 = float(
        re.search(
            r"= ([\d.]+) min", next(f for f in heavy if f.code == "FLIGHT_TIME").message
        ).group(1)
    )
    assert t1 < t0
    h0 = next(f for f in x500_findings if f.code == "HOVER_THROTTLE").message
    assert "throttle" in h0 and " A" in h0


def test_servo_torque_with_1kg_on_the_arm(so101, lib):
    fs = check(
        mutated(
            so101,
            lambda d: by(d, "instances", "id", "moving_jaw").update(
                params={"mass": prov(1.0, "kg")}
            ),
        ),
        lib,
    )
    # holding 1 kg also draws more than the PSU / cord can give
    f = fires(
        fs,
        "SERVO_TORQUE",
        also=("INTERFERENCE", "PSU_OVERLOAD", "CURRENT_OVER_RATING"),
    )
    assert any(x.severity == "error" and "cannot hold" in x.message for x in f)
    pl = next(x for x in fs if x.code == "PAYLOAD_LIMIT")
    assert pl.severity == "warning"


def test_payload_limit_is_what_the_servos_can_still_hold(so101, lib, so101_findings):
    pl = next(f for f in so101_findings if f.code == "PAYLOAD_LIMIT")
    grams = float(re.search(r"mass ([\d.]+) g", pl.message).group(1))
    assert 20 < grams < 400
    # extra mass on the jaw (it is the payload point): 80 % of the limit keeps every servo within stall, 150 % does not
    jaw = next(
        r["mass"]
        for r in design.compute(so101, PARTS, ".", lib)[1]["instances"]
        if r["id"] == "moving_jaw"
    )
    for k, expect_error in ((0.8, False), (1.5, True)):
        fs = check(
            mutated(
                so101,
                lambda d, k=k: by(d, "instances", "id", "moving_jaw").update(
                    params={"mass": prov(jaw + k * grams / 1000, "kg")}
                ),
            ),
            lib,
        )
        assert ("SERVO_TORQUE" in codes(fs, "error")) == expect_error


# ---- build failure and interface -------------------------------------------------------------------------------------------------------
def test_unknown_part_is_a_build_failed_finding(x500, lib):
    fs = check(
        mutated(
            x500, lambda d: by(d, "instances", "id", "gps").update(part="no-such-part")
        ),
        lib,
    )
    assert [f.code for f in fs] == ["BUILD_FAILED"]


def test_every_code_is_documented_in_the_readme():
    text = (REPO / "assemblies" / "README.md").read_text()
    section = text[text.index("## Design rules") :]
    for code in drc.RULES:
        assert f"`{code}`" in section, code


def test_build_report_carries_findings_and_never_fails_on_them(tmp_path):
    r = design.build(SO101, PARTS, tmp_path)
    rep = json.loads(Path(r["report_path"]).read_text())
    assert rep["findings"] == r["report"]["findings"] and rep["findings"]
    # so101 has DRC errors and still builds; DRC findings are not in the builder's own warning list
    assert any(f["severity"] == "error" for f in rep["findings"])
    assert all(
        w["code"] not in drc.RULES or w["code"] == "FASTENER_SHORT"
        for w in rep["warnings"]
    )


def test_cli_check_exit_codes_and_json(tmp_path, capsys, x500, so101):
    ok = tmp_path / "ok.toml"  # one plate, nothing to violate
    ok.write_text(
        dumps(
            Assembly(
                {
                    "assembly": {"name": "plate"},
                    "instances": [
                        {
                            "id": "p",
                            "part": "holybro-x500v2-bottom-plate",
                            "root": True,
                        }
                    ],
                }
            )
        )
    )
    assert main(["design", "check", str(ok)]) == 0
    capsys.readouterr()
    rc = main(["design", "check", str(X500), "--json"])
    out = json.loads(capsys.readouterr().out)
    assert out and set(out[0]) == {"code", "severity", "message", "where", "fix"}
    assert rc == (1 if any(f["severity"] == "error" for f in out) else 0)
    assert (
        main(["design", "check", str(SO101)]) == 1
    )  # the adapter / base INTERFERENCE errors
    assert "INTERFERENCE" in capsys.readouterr().out
    bad = tmp_path / "bad.toml"
    bad.write_text(
        dumps(
            mutated(
                x500, lambda d: d["mates"].remove(by(d, "mates", "name", "esc-seat1"))
            )
        )
    )
    assert main(["design", "check", str(bad)]) == 1
    assert main(["design", "check", str(tmp_path / "missing.toml")]) == 2
