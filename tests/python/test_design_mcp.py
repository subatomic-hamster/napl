"""E1 exit check: an agent edits designs through the MCP tools only: open -> suggest -> swap -> DRC delta -> build -> hover test,
the battery swap that trips the battery rules, the arm payload case, undo, save with approval, the sandbox, schema errors.

Variant parts (a heavier 2216 motor with the same ports; a tiny 4S pack with the same ports; both generated from library part
files by text substitution) live in a tmp parts directory that mirrors parts/ with symlinks. parts/ and assemblies/ are not touched."""

import asyncio
import json
import tomllib
from pathlib import Path

import napl
import pytest
from mcp import Client
from mcp.shared.exceptions import MCPError
from mcp_types import ElicitResult
from napl.mcp_server import NaplServer
from test_design_edit import MOTOR_VARIANT, PACK_VARIANT, variant_texts

REPO = Path(__file__).resolve().parents[2]
needs_kernel = pytest.mark.skipif(napl._napl is None, reason="kernel not built")


@pytest.fixture(scope="module")
def root(tmp_path_factory):
    r = tmp_path_factory.mktemp("design-root")
    (r / "parts").mkdir()
    for f in (REPO / "parts").rglob("*.toml"):
        dest = r / "parts" / f.relative_to(REPO / "parts")
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.symlink_to(f)
    for pid, text in variant_texts().items():
        (r / "parts" / "test").mkdir(exist_ok=True)
        (r / "parts" / "test" / f"{pid}.toml").write_text(text)
    (r / "meshes").symlink_to(
        REPO / "meshes"
    )  # mesh paths are relative to the parts dir's parent
    (r / "assemblies").mkdir()
    for n in ("x500v2", "so101"):
        (r / "assemblies" / f"{n}.toml").write_bytes(
            (REPO / "assemblies" / f"{n}.toml").read_bytes()
        )
    return r


async def accept(context, params):
    return ElicitResult(action="accept", content={"confirm": True})


async def decline(context, params):
    return ElicitResult(action="decline")


async def call(c, name, **args):
    try:
        r = await c.call_tool(name, args)
    except MCPError as e:
        return True, str(e)
    text = r.content[0].text
    return r.is_error, (text if r.is_error else json.loads(text))


async def ok(c, name, **args):
    err, out = await call(c, name, **args)
    assert not err, out
    return out


def brief(f):
    return f"{f['severity']} {f['code']} {f['where'][:2]}: {f['message'][:90]}"


@needs_kernel
def test_agent_swaps_motors_then_battery_and_checks_hover(root):
    async def agent():
        srv = NaplServer(root)
        log = []
        async with Client(srv.server, elicitation_callback=accept) as c:
            o = await ok(c, "design_open", path="assemblies/x500v2.toml")
            assert (
                o["name"] == "x500v2"
                and o["findings"]["error"] == 0
                and o["library_parts"] > 280
            )
            log.append(f"open: {len(o['instances'])} instances, DRC {o['findings']}")
            base_hover = await ok(c, "design_simulate", kind="hover")
            assert base_hover["pass"]
            log.append(
                f"stock hover: throttle {base_hover['hover_throttle']:.4f} current {base_hover['hover_current_A']:.2f} A "
                f"alt err max {base_hover['altitude_error_max_m']:.3f} m endurance {base_hover['endurance_s']:.0f} s"
            )

            # 1. which motors fit the arm connector's 16x16 M3 pattern? (stock part first)
            fit = (await ok(c, "design_suggest", target="con1.motor", limit=10))[
                "suggestions"
            ]
            assert fit[0]["part"] == "holybro-2216-kv920" and MOTOR_VARIANT in [
                r["part"] for r in fit
            ]
            log.append(
                "suggest con1.motor: "
                + ", ".join(f"{r['part']}({r['score']:g})" for r in fit[:4])
            )
            # 2. replacement candidates for the motor instance, ranked by fit; the agent takes the best that keeps everything
            cands = (await ok(c, "design_suggest", target="mot1", limit=6))[
                "suggestions"
            ]
            pick = next(r for r in cands if not r["mates_lost"] and not r["dangling"])
            assert pick["part"] == MOTOR_VARIANT
            log.append(
                "suggest mot1: "
                + ", ".join(
                    f"{r['part']}(keeps {r['mates_kept']}, loses {len(r['mates_lost'])})"
                    for r in cands[:4]
                )
            )

            # 3. swap all four motors in one atomic edit
            r = await ok(
                c,
                "design_edit",
                ops=[
                    {
                        "op": "swap",
                        "id": ["mot1", "mot2", "mot3", "mot4"],
                        "part": pick["part"],
                    }
                ],
            )
            assert len(r["changes"]) == 4 and all(
                not ch["mates"]["lost"] for ch in r["changes"]
            )
            assert r["drc"]["after"]["error"] == 0 and r["undo_depth"] == 1
            delta = {c_["after"]["code"] for c_ in r["drc"]["changed"]}
            assert (
                "THRUST_TO_WEIGHT" in delta
                and "HOVER_THROTTLE" in delta
                or "FLIGHT_TIME" in delta
            )
            tw = next(
                c_
                for c_ in r["drc"]["changed"]
                if c_["after"]["code"] == "THRUST_TO_WEIGHT"
            )
            log.append(
                f"swap 4 motors -> {pick['part']}: DRC {r['drc']['before']} -> {r['drc']['after']}"
            )
            log.append(
                "  T/W before: "
                + tw["before"]["message"][-60:]
                + " | after: "
                + tw["after"]["message"][-60:]
            )

            chk = await ok(c, "design_check", severity="warning")
            assert chk["counts"]["error"] == 0
            b = await ok(c, "design_build")
            assert (root / b["xml"]).is_file() and b["mass_kg"] > base_hover[
                "mass_kg"
            ] + 0.03
            log.append(
                f"build: {b['xml']} mass {b['mass_kg']:.4f} kg (stock {base_hover['mass_kg']:.4f}) findings {b['findings']}"
            )
            hv = await ok(
                c, "design_simulate", kind="hover", altitude=2.0, hold_s=10.0, tol=0.3
            )
            assert (
                hv["pass"] and hv["altitude_error_max_m"] <= 0.3 and hv["sim_s"] >= 14.0
            )
            assert (
                abs(hv["hover_throttle"] - base_hover["hover_throttle"]) > 0.005
            )  # the edit reached the simulation
            log.append(
                f"edited hover: pass={hv['pass']} alt err max {hv['altitude_error_max_m']:.3f} m rms {hv['altitude_error_rms_m']:.3f} "
                f"throttle {hv['hover_throttle']:.4f} current {hv['hover_current_A']:.2f} A endurance {hv['endurance_s']:.0f} s T/W {hv['thrust_to_weight']:.2f}"
            )

            # 4. a tiny pack: the battery rules fire
            r = await ok(
                c,
                "design_edit",
                ops=[{"op": "swap", "id": "bat", "part": PACK_VARIANT}],
            )
            new = {f["code"]: f for f in r["drc"]["new"]}
            assert (
                new["BATTERY_C_RATING"]["severity"] == "error"
                and r["drc"]["after"]["error"] > 0
            )
            ft = next(
                c_ for c_ in r["drc"]["changed"] if c_["after"]["code"] == "FLIGHT_TIME"
            )
            log.append(
                "swap bat -> 0.3 Ah: new "
                + "; ".join(brief(f) for f in r["drc"]["new"])
            )
            log.append(
                "  FLIGHT_TIME: "
                + ft["before"]["message"][:55]
                + " -> "
                + ft["after"]["message"][:55]
            )
            chk = await ok(c, "design_check", severity="error")
            assert {f["code"] for f in chk["findings"]} == {"BATTERY_C_RATING"}
            tiny = await ok(c, "design_simulate", kind="hover")
            assert (
                tiny["endurance_s"] < 0.1 * hv["endurance_s"]
                and tiny["hover_current_A"] > 5
            )
            log.append(
                f"tiny pack hover: pass={tiny['pass']} current {tiny['hover_current_A']:.2f} A endurance {tiny['endurance_s']:.0f} s (DRC FLIGHT_TIME above)"
            )

            # 5. undo both swaps; the design is the stock one again
            u1, u2 = await ok(c, "design_undo"), await ok(c, "design_undo")
            assert "BATTERY_C_RATING" in {f["code"] for f in u1["drc"]["resolved"]}
            assert u2["drc"]["after"] == o["findings"]
            err, text = await call(c, "design_undo")
            assert err and "nothing to undo" in text
            redo = await ok(c, "design_undo", redo=True)
            assert redo["changes"] == [{"op": "redo"}]
            return log

    for line in asyncio.run(agent()):
        print(line)


@needs_kernel
def test_agent_arm_payload(root):
    async def agent():
        log = []
        async with Client(NaplServer(root).server, elicitation_callback=accept) as c:
            await ok(c, "design_open", path="assemblies/so101.toml")
            h0 = await ok(c, "design_simulate", kind="hold")
            chk0 = {
                f["code"]: f
                for f in (await ok(c, "design_check", severity="info"))["findings"]
            }
            b = await ok(c, "design_build")
            jaw = json.loads((root / b["report"]).read_text())
            jaw_m = next(i["mass"] for i in jaw["instances"] if i["id"] == "moving_jaw")
            log.append(
                f"stock arm: sag {h0['sag_max_rad']:.4f} rad current {h0['current_max_A']:.3f} A, "
                f"{brief(chk0['PAYLOAD_LIMIT'])}"
            )
            assert h0["pass"] and chk0["PAYLOAD_LIMIT"]["severity"] == "info"

            # 0.3 kg payload in the gripper = the moving jaw's mass + 0.3 kg, with provenance (numbers rule)
            err, text = await call(
                c,
                "design_edit",
                ops=[
                    {
                        "op": "set",
                        "id": "moving_jaw",
                        "param": "mass",
                        "value": jaw_m + 0.3,
                    }
                ],
            )
            assert (
                err and "numbers rule" in text
            )  # a bare number is refused over MCP too
            r = await ok(
                c,
                "design_edit",
                ops=[
                    {
                        "op": "set",
                        "id": "moving_jaw",
                        "param": "mass",
                        "value": round(jaw_m + 0.3, 6),
                        "unit": "kg",
                        "fidelity": "estimated",
                        "rationale": "0.3 kg payload held in the gripper (test)",
                    }
                ],
            )
            pairs = [(c_["before"], c_["after"]) for c_ in r["drc"]["changed"]]
            pl = next(p for p in pairs if p[1]["code"] == "PAYLOAD_LIMIT")
            st = [p for p in pairs if p[1]["code"] == "SERVO_TORQUE"]
            assert (pl[0]["severity"], pl[1]["severity"]) == ("info", "warning")
            assert st and all(b["message"] != a["message"] for b, a in st)
            log.append(
                "payload +0.3 kg: PAYLOAD_LIMIT "
                + pl[0]["message"][:60]
                + " -> "
                + pl[1]["message"][:60]
                + " | "
                + " | ".join(
                    f"SERVO_TORQUE {a['where'][0]} {a['severity']}: {a['message'][:70]}"
                    for _, a in st
                )
            )
            h1 = await ok(c, "design_simulate", kind="hold")
            log.append(
                f"loaded arm: sag {h1['sag_max_rad']:.4f} rad current {h1['current_max_A']:.3f} A pass={h1['pass']}"
            )
            assert (
                h1["sag_max_rad"] > 2 * h0["sag_max_rad"]
                and h1["current_max_A"] > h0["current_max_A"]
            )
            return log

    for line in asyncio.run(agent()):
        print(line)


def test_save_needs_approval_and_the_sandbox_holds(root, tmp_path_factory):
    outside = tmp_path_factory.mktemp("outside")

    async def agent():
        srv = NaplServer(root)
        async with Client(srv.server) as c:  # no elicitation support
            err, text = await call(
                c, "design_edit", ops=[{"op": "remove", "id": "gps"}]
            )
            assert err and "no design open" in text
            await ok(c, "design_open", path="assemblies/x500v2.toml")
            await ok(
                c, "design_edit", ops=[{"op": "remove", "id": "gps"}]
            )  # control: no approval needed
            err, text = await call(c, "design_save", path="edited.toml")
            assert err and "elicitation" in text
        async with Client(srv.server, elicitation_callback=decline) as c:
            err, text = await call(c, "design_save", path="edited.toml")
            assert err and "refused" in text and not (root / "edited.toml").exists()
        async with Client(srv.server, elicitation_callback=accept) as c:
            for bad in ("../x.toml", str(outside / "x.toml"), "/etc/x.toml"):
                err, text = await call(c, "design_save", path=bad)
                assert err and "outside the server root" in text, (bad, text)
            err, text = await call(c, "design_open", path=str(outside / "x.toml"))
            assert err and "outside the server root" in text
            err, text = await call(
                c, "design_open", path="assemblies/x500v2.toml", parts="../elsewhere"
            )
            assert err and "outside the server root" in text
            err, text = await call(c, "design_save", path="notes.txt")
            assert err and ".toml" in text
            s = await ok(c, "design_save", path="out/edited.toml")
            assert s["saved"] == "out/edited.toml" and s["bytes"] > 1000
            # the saved design reopens with the edit in it
            o = await ok(c, "design_open", path="out/edited.toml")
            assert "gps" not in {i["id"] for i in o["instances"]}
            assert (tomllib.loads((root / "out" / "edited.toml").read_text()))[
                "assembly"
            ]["name"] == "x500v2"
        async with Client(
            NaplServer(root, allow_destructive=True).server
        ) as c:  # --allow-destructive: no prompt
            await ok(c, "design_open", path="assemblies/so101.toml")
            await ok(c, "design_save", path="out/so101-copy.toml")

    asyncio.run(agent())


def test_schema_and_design_errors_are_clean(root):
    async def agent():
        async with Client(NaplServer(root).server, elicitation_callback=accept) as c:
            await ok(c, "design_open", path="assemblies/x500v2.toml")
            for ops in (
                [{"op": "explode"}],  # unknown op
                [{"op": "remove", "id": "gps", "force": True}],  # unknown key
                [],  # minItems
                [{"op": "swap", "id": "mot1"}],  # missing part
            ):
                err, text = await call(c, "design_edit", ops=ops)
                assert err, ops
            for ops, msg in (
                ([{"op": "remove", "id": "nope"}], "unknown instance"),
                (
                    [{"op": "swap", "id": "mot1", "part": "holybro-2216-kv92"}],
                    "did you mean holybro-2216-kv920",
                ),
                (
                    [
                        {
                            "op": "wire",
                            "from": "gps.gps_out",
                            "to": "fc.gps1",
                            "part": "sts3215-servo-cable-3pin",
                            "length": 0.1,
                        }
                    ],
                    "numbers rule",
                ),
            ):
                err, text = await call(c, "design_edit", ops=ops)
                assert err and msg in text, (ops, text)
            assert (await ok(c, "design_check"))["counts"][
                "error"
            ] == 0  # none of it changed the design
            err, text = await call(c, "design_suggest", target="nope.port")
            assert err and "unknown instance" in text
            err, text = await call(c, "design_simulate", kind="hold", altitude=2.0)
            assert (
                err
            )  # `altitude` is not a hold option (schema allows it, the tool refuses)

    asyncio.run(agent())
