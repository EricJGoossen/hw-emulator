#!/usr/bin/env python3
"""Turns devices.yaml's MuJoCo elements into Achilles attachments.

    tools/elements_to_arow.py --devices devices.yaml [--sim sim.yaml]
                              --robot ROBOT.arow --out PARTS_DIR
                              [--archetype NAME] [--gaps FILE]

A URDF has no motors or sensors, so devices.yaml declares each binding's
MuJoCo element under `bind.mujoco` (see the README's table). For the Achilles
backend the same names have to be attachments in the .arow. This writes one
.arow part per element into PARTS_DIR, and adds them to ROBOT.arow's
`includes`, so both backends serve the same devices from one devices.yaml.

ROBOT.arow comes from Achilles's tools/urdf_to_arow.py, and its
ROBOT.frames.json, written beside it, says which .arow joint carries each
URDF link and joint, and where the link sits in that joint's frame. A
`body:` element's pos/quat (in the URDF link's frame) become its `mount` in
the joint's frame; a `joint:` element attaches to the joint's .arow joint.

Element -> Achilles kind (MuJoCo key -> Achilles setting):
  motor    -> dc_motor (axis [1, 0, 0], the joint's own x; the DC-motor keys as
              is, except the friction: the MuJoCo backend takes
              viscous_friction and coulomb_friction at the joint, Achilles's
              DcMotor at the rotor, so viscous_friction * gear_ratio^2 and
              coulomb_friction * gear_ratio)
  encoder  -> encoder, multi-turn like MuJoCo's jointpos (resolution ->
              position_resolution, noise -> position_noise_sigma,
              velocity_noise -> velocity_noise_sigma)
  gyro / accel / mag -> gyroscope / accelerometer / magnetometer (noise -> noise_sigma)
  attitude -> attitude (noise -> noise_sigma)
  switch   -> switch (press_above, press_below, hysteresis, normally_closed;
              noise -> position_noise_sigma)
  camera   -> camera (width, height, fx, fy, cx, cy, near, far, period;
              fovy -> fx = fy = height / 2 / tan(fovy / 2) when fx/fy aren't given)
  lidar    -> lidar (samples -> horizontal_samples, fov -> horizontal_fov,
              min_range, max_range, period, noise -> range_noise)

sim.yaml's `mujoco.elements` are model edits rather than attachments:
  exclude -> nothing to do when both bodies are in the robot (urdf_to_arow
             puts a robot's bodies in one collision group that never
             collides with itself); otherwise a gap
  joint   -> armature is added to each matching joint's inertia about its
             axis (exact for the joint's own acceleration, approximate for
             its coupling to the parent: a gap); damping and frictionloss
             aren't modelled by Achilles (a gap)

Every key that has no Achilles counterpart, and every approximation, is a sim
gap: printed as `[hwemu GAP] elements_to_arow: ...` and, with --gaps, appended
to that JSONL file in hw-emulator's gap format (process "model").
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import math
import os
import re
import sys

import yaml


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qrot(q, v):
    w = qmul(qmul(q, (0.0, *v)), (q[0], -q[1], -q[2], -q[3]))
    return w[1:]


def qnorm(q):
    n = math.sqrt(sum(c * c for c in q))
    q = tuple(c / n for c in q)
    return q if q[0] >= 0 else tuple(-c for c in q)


def compose(a, b):
    """Pose a * pose b; poses are (translation, w-first quaternion)."""
    r = qrot(a[1], b[0])
    return (tuple(x + y for x, y in zip(a[0], r)), qnorm(qmul(a[1], b[1])))


def fmt(values) -> str:
    out = []
    for v in values:
        v = float(v)
        out.append("0" if abs(v) < 1e-12 else f"{v:.9g}")
    return "[" + ", ".join(out) + "]"


class Gaps:
    def __init__(self, path):
        self.path, self.seen = path, set()

    def __call__(self, key, message):
        if key in self.seen:
            return
        self.seen.add(key)
        print(f"[hwemu GAP] elements_to_arow: {message}", file=sys.stderr)
        if self.path:
            os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
            with open(self.path, "a") as f:
                f.write(json.dumps({"process": "model", "source": "elements_to_arow", "key": key,
                                    "message": message, "time": 0}) + "\n")


MOTOR_KEYS = ["torque_constant", "back_emf_constant", "resistance", "bus_voltage", "max_current", "gear_ratio",
              "gearbox_efficiency", "viscous_friction", "coulomb_friction", "ambient_temperature",
              "thermal_resistance", "thermal_time_constant"]
VEC_KIND = {"gyro": "gyroscope", "accel": "accelerometer", "mag": "magnetometer", "attitude": "attitude"}


class Converter:
    def __init__(self, frames, archetype, gaps):
        self.links = frames["links"]
        self.joints = frames["joints"]
        self.archetype = archetype
        self.gaps = gaps

    def joint(self, name, el_name):
        if name not in self.joints:
            raise SystemExit(f"elements_to_arow: {el_name}: the URDF has no joint '{name}'")
        return self.joints[name]

    def mount(self, el, el_name):
        body = el.get("body")
        if body not in self.links:
            raise SystemExit(f"elements_to_arow: {el_name}: the URDF has no link '{body}'")
        f = self.links[body]
        link_in_joint = (tuple(f["pose"]["translation"]), tuple(f["pose"]["rotation"]))
        local = (tuple(float(v) for v in el.get("pos", [0, 0, 0])),
                 qnorm(tuple(float(v) for v in el.get("quat", [1, 0, 0, 0]))))
        t, q = compose(link_in_joint, local)
        return f["joint"], f"{{translation: {fmt(t)}, rotation: {fmt(q)}}}"

    def unused(self, el_name, el, used):
        for k in el:
            if k not in used and k != "type":
                self.gaps(f"key:{el['type']}:{k}",
                          f"{el['type']} '{k}' has no Achilles counterpart; ignored (first on {el_name})")

    def part(self, el_name, el) -> str | None:
        t = el.get("type")
        cfg: list[tuple[str, str]] = []

        def num(key, as_key=None, value=None):
            if key in el or value is not None:
                v = el[key] if value is None else value
                cfg.append((as_key or key, fmt(v) if isinstance(v, list) else f"{float(v):.9g}"))

        used = set()
        if t == "motor":
            joint = self.joint(el["joint"], el_name)
            cfg.append(("axis", "[1, 0, 0]"))
            g = float(el.get("gear_ratio", 1.0))
            for k in MOTOR_KEYS:
                if k == "viscous_friction" and k in el:
                    num(k, value=float(el[k]) * g * g)  # N*m*s at the joint -> at the rotor
                elif k == "coulomb_friction" and k in el:
                    num(k, value=float(el[k]) * g)      # N*m at the joint -> at the rotor
                else:
                    num(k)
            used = {"joint", *MOTOR_KEYS}
            head = f"actuator: {el_name}\nkind: dc_motor"
        elif t == "encoder":
            joint = self.joint(el["joint"], el_name)
            cfg.append(("multi_turn", "1"))
            num("resolution", "position_resolution")
            num("noise", "position_noise_sigma")
            num("velocity_noise", "velocity_noise_sigma")
            used = {"joint", "resolution", "noise", "velocity_noise"}
            head = f"sensor: {el_name}\nkind: encoder"
        elif t in VEC_KIND:
            joint, mount = self.mount(el, el_name)
            cfg.append(("mount", mount))
            num("noise", "noise_sigma")
            used = {"body", "pos", "quat", "noise"}
            head = f"sensor: {el_name}\nkind: {VEC_KIND[t]}"
        elif t == "switch":
            joint = self.joint(el["joint"], el_name)
            for k in ("press_above", "press_below", "hysteresis"):
                num(k)
            if "normally_closed" in el:
                cfg.append(("normally_closed", "1" if el["normally_closed"] else "0"))
            num("noise", "position_noise_sigma")
            used = {"joint", "press_above", "press_below", "hysteresis", "normally_closed", "noise"}
            head = f"sensor: {el_name}\nkind: switch"
        elif t == "camera":
            joint, mount = self.mount(el, el_name)
            cfg.append(("mount", mount))
            for k in ("period", "width", "height"):
                num(k)
            if "fx" not in el and "fy" not in el and "fovy" in el:
                f = float(el.get("height", 480)) / 2 / math.tan(math.radians(float(el["fovy"])) / 2)
                num("fx", value=f)
                num("fy", value=f)
            for k in ("fx", "fy", "cx", "cy", "near", "far"):
                num(k)
            used = {"body", "pos", "quat", "period", "width", "height", "fx", "fy", "cx", "cy", "near", "far", "fovy"}
            head = f"sensor: {el_name}\nkind: camera"
        elif t == "lidar":
            joint, mount = self.mount(el, el_name)
            cfg.append(("mount", mount))
            num("period")
            cfg.append(("horizontal_samples", str(int(el.get("samples", 360)))))
            num("fov", "horizontal_fov", value=el.get("fov", 2 * math.pi))
            num("min_range", value=el.get("min_range", 0.02))
            num("max_range", value=el.get("max_range", 12))
            num("noise", "range_noise")
            used = {"body", "pos", "quat", "period", "samples", "fov", "min_range", "max_range", "noise"}
            head = f"sensor: {el_name}\nkind: lidar"
            if int(el.get("samples", 360)) > 1024:
                raise SystemExit(f"elements_to_arow: {el_name}: Achilles lidars cast at most 1024 beams")
        else:
            self.gaps(f"type:{t}", f"MuJoCo element type '{t}' ({el_name}) has no Achilles counterpart; not served")
            return None
        self.unused(el_name, el, used)
        body = "\n".join(f"  {k}: {v}" for k, v in cfg)
        return (f"# Generated by hw-emulator tools/elements_to_arow.py from devices.yaml's\n"
                f"# bind.mujoco.{el_name}. Don't edit: regenerate it.\n"
                f"{head}\nattach: {{archetype: {self.archetype}, joint: {joint}}}\nconfig:\n{body}\n")


def apply_model_elements(sim, frames, robot_text, gaps):
    """sim.yaml mujoco.elements -> edits to the robot's .arow text."""
    elements = ((sim or {}).get("mujoco") or {}).get("elements") or {}
    links = frames["links"]
    jmap = frames["joints"]
    for name, el in elements.items():
        t = el.get("type")
        if t == "exclude":
            pats = (el.get("body1", ""), el.get("body2", ""))
            hits = [[l for l in links if fnmatch.fnmatchcase(l, p)] for p in pats]
            robot = [all(links[l]["joint"] != "world" and not links[l]["joint"].startswith("world__") for l in h)
                     for h in hits]
            if not all(hits) or not all(robot):
                gaps(f"exclude:{name}", f"exclude '{name}' ({pats[0]} / {pats[1]}) isn't between two robot bodies; "
                                        f"Achilles can't exclude it")
            # Otherwise already true: a robot's bodies never collide with each other.
        elif t == "joint":
            targets = sorted({jmap[j] for j in jmap if fnmatch.fnmatchcase(j, el.get("joints", ""))})
            if not targets:
                gaps(f"joint-none:{name}", f"joint element '{name}' matches no joint ({el.get('joints')})")
                continue
            arm = float(el.get("armature", 0))
            if arm:
                for j in targets:
                    robot_text = add_armature(robot_text, j, arm)
                gaps(f"armature:{name}", f"joint armature ({name}, {arm:g} on {len(targets)} joints) added to "
                                         f"each body's inertia about its axis: exact for the joint, approximate "
                                         f"for its coupling to the parent")
            for k in ("damping", "frictionloss"):
                if float(el.get(k, 0)):
                    gaps(f"{k}:{name}", f"joint {k} ({name}) isn't modelled by Achilles; ignored")
            for k in el:
                if k not in ("type", "joints", "armature", "damping", "frictionloss"):
                    gaps(f"joint-key:{k}", f"joint element key '{k}' ({name}) has no Achilles counterpart")
        else:
            gaps(f"model-element:{t}", f"sim.yaml mujoco element '{name}' (type {t}) has no Achilles counterpart")
    return robot_text


def add_armature(text, joint, armature):
    """Adds `armature` to Ixx (about the joint's own x) of one joint's rigid_body_inertia."""
    pat = re.compile(r"(  - name: " + re.escape(joint) + r"\n(?:    .*\n)*?      rigid_body_inertia: \[)([^\]]*)(\])")
    m = pat.search(text)
    if not m:
        raise SystemExit(f"elements_to_arow: no joint {joint} in the robot .arow")
    vals = [float(v) for v in m.group(2).split(",")]
    vals[4] += armature
    return text[:m.start(2)] + ", ".join(f"{v:.9g}" for v in vals) + text[m.end(2):]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--devices", required=True)
    ap.add_argument("--sim")
    ap.add_argument("--robot", required=True, help="the robot .arow (from urdf_to_arow.py); its includes are rewritten")
    ap.add_argument("--out", required=True, help="where the parts go")
    ap.add_argument("--archetype", help="the archetype attachments go on (default: the robot's)")
    ap.add_argument("--gaps")
    args = ap.parse_args()
    gaps = Gaps(args.gaps)

    frames_path = re.sub(r"\.arow$", ".frames.json", args.robot)
    with open(frames_path) as f:
        frames = json.load(f)
    archetype = args.archetype or frames["archetype"]
    with open(args.devices) as f:
        devices = yaml.safe_load(f)
    sim = None
    if args.sim:
        with open(args.sim) as f:
            sim = yaml.safe_load(f)

    conv = Converter(frames, archetype, gaps)
    os.makedirs(args.out, exist_ok=True)
    parts = []
    for dev in devices.get("devices", []):
        for el_name, el in ((dev.get("bind") or {}).get("mujoco") or {}).items():
            text = conv.part(el_name, el)
            if text is None:
                continue
            path = os.path.join(args.out, f"{el_name}.arow")
            with open(path, "w") as f:
                f.write(text)
            parts.append(os.path.relpath(path, os.path.dirname(os.path.abspath(args.robot))))

    with open(args.robot) as f:
        robot = f.read()
    robot = apply_model_elements(sim, frames, robot, gaps)
    robot = re.sub(r"^includes:\n(?:  - .*\n)*", "", robot, flags=re.M)
    inc = "includes:\n" + "".join(f"  - {p}\n" for p in sorted(parts))
    robot = re.sub(r"^(archetype: .*\n)", lambda m: m.group(1) + inc, robot, count=1, flags=re.M)
    with open(args.robot, "w") as f:
        f.write(robot)
    print(f"{args.out}: {len(parts)} attachments, included by {args.robot}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
