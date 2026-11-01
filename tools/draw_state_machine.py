#!/usr/bin/env python3
"""Draw the slave's main state machine from the transition table below.

    python3 tools/draw_state_machine.py                 # rewrite the SVG
    python3 tools/draw_state_machine.py --png out.png   # also write a PNG
    python3 tools/draw_state_machine.py --check         # table check only

The drawing is docs/slave/diagrams/MainStateMachine.svg, laid out by
Graphviz (the `neato` command must be on the PATH; the node
positions are pinned in LAYOUT, neato only routes the edges). The transition table
(TRANSITIONS) is a transcription of the IF/ELSIF chain in
slave-src/PRG/PRG_Main.st ("State Machine" section, plus the "Safety
Monitoring" block at the end of the file) and the states are the members
of E_SystemState in slave-src/DUT/DataTypes.st. Run the script after any
change to either; it stops with an error when
    - the states of the table differ from E_SystemState, or
    - the targets that PRG_Main assigns to G_sysCurrentState in a state's
      branch differ from the table's transitions out of that state,
so the diagram cannot go stale silently. The conditions on the edges are
written by hand and are not checked.
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATA_TYPES = ROOT / "slave-src" / "DUT" / "DataTypes.st"
PRG_MAIN = ROOT / "slave-src" / "PRG" / "PRG_Main.st"
SVG_OUT = ROOT / "docs" / "slave" / "diagrams" / "MainStateMachine.svg"

# %% ===== STATES =====
# name -> (group, mode code shown in the node, one-line description).
# The group sets the node colour; the order is the order of E_SystemState.
STATES = [
    ("ST_INIT", "init", None, "first scan"),
    ("ST_IDLE", "idle", None, "drive off, brake on"),
    ("ST_DRIVE_RESET", "trans", None, "MC_Reset"),
    ("ST_DRIVE_ENABLE", "trans", None, "MC_Power on"),
    ("ST_BRAKE_RELEASE", "trans", None, "brake delay"),
    ("ST_BRAKE_HOLD", "oper", "001", "drive on, brake on"),
    ("ST_POSITION_CTRL", "oper", "010", "position"),
    ("ST_VELOCITY_CTRL", "oper", "011", "velocity"),
    ("ST_TORQUE_CTRL", "oper", "100", "torque"),
    ("ST_PRESSURE_CTRL", "oper", "111", "pressure loop"),
    ("ST_GO_HOME", "oper", "101", "move to home"),
    ("ST_HOME_LIMIT", "oper", "110", "FB_HomeLimit"),
    ("ST_HOME_COMPLETE", "oper", None, "homing done"),
    ("ST_HOLD_POSITION", "trans", None, "MC_Stop"),
    ("ST_BRAKE_ENGAGE", "trans", None, "brake delay"),
    ("ST_DRIVE_DISABLE", "trans", None, "MC_Power off"),
    ("ST_FAULT", "fault", None, "MC_Stop, drive on"),
    ("ST_FAULT_IDLE", "fault", None, "drive off, brake on"),
    ("ST_RECOVERY", "recov", "001", "hub, MC_Stop"),
    ("ST_RECOVERY_POSITION", "recov", "010", "retract in position"),
    ("ST_RECOVERY_VELOCITY", "recov", "011", "retract in velocity"),
]

FILL = {
    "init": "#E3F2FD",
    "idle": "#FCE4EC",
    "trans": "#F3E5F5",
    "oper": "#E8F5E9",
    "fault": "#FFEBEE",
    "recov": "#FFF3E0",
}

# %% ===== TRANSITIONS =====
# (from, to, condition, kind). kind: "main" the normal path, "mode" a
# handshake-selected mode, "stop" motion enable dropped, "fault" into or
# inside the fault states, "recov" the limit-recovery path. The condition
# strings follow PRG_Main; "HS" is fbHandshake.HandshakeComplete (the
# master wrote the mode, the slave confirmed it, the master raised
# MotionEnable), "ME low" the filtered MotionEnable being low.
# Mode codes: 001 BRAKE_HOLD, 010 POSITION, 011 VELOCITY, 100 TORQUE,
# 101 GO_HOME, 110 HOME_LIMIT, 111 PRESSURE.
LIMIT = "limit still active"
TRANSITIONS = [
    # --- Start and activation (PRG_Main: IDLE ... BRAKE_RELEASE) ---
    ("ST_INIT", "ST_IDLE", "first scan", "main"),
    ("ST_IDLE", "ST_DRIVE_RESET", "HS, mode 1..7", "main"),
    ("ST_DRIVE_RESET", "ST_DRIVE_ENABLE", "reset done", "main"),
    ("ST_DRIVE_RESET", "ST_FAULT", "reset error / timeout", "fault"),
    ("ST_DRIVE_ENABLE", "ST_BRAKE_RELEASE", "power on", "main"),
    ("ST_DRIVE_ENABLE", "ST_FAULT", "power error / timeout", "fault"),
    # The brake delay ends in the state of the requested mode.
    ("ST_BRAKE_RELEASE", "ST_BRAKE_HOLD", "delay, mode 1", "mode"),
    ("ST_BRAKE_RELEASE", "ST_POSITION_CTRL", "delay, mode 2", "mode"),
    ("ST_BRAKE_RELEASE", "ST_VELOCITY_CTRL", "delay, mode 3", "mode"),
    ("ST_BRAKE_RELEASE", "ST_TORQUE_CTRL", "delay, mode 4", "mode"),
    ("ST_BRAKE_RELEASE", "ST_GO_HOME", "delay, mode 5", "mode"),
    ("ST_BRAKE_RELEASE", "ST_HOME_LIMIT", "delay, mode 6", "mode"),
    ("ST_BRAKE_RELEASE", "ST_PRESSURE_CTRL", "delay, mode 7", "mode"),
    ("ST_BRAKE_RELEASE", "ST_HOLD_POSITION", "delay, other mode", "stop"),
    ("ST_BRAKE_RELEASE", "ST_RECOVERY", "delay, recovery pending", "recov"),
    # --- Operational states ---
    ("ST_BRAKE_HOLD", "ST_BRAKE_RELEASE", "HS, mode 2..7", "mode"),
    ("ST_BRAKE_HOLD", "ST_DRIVE_DISABLE", "HS, mode 0", "main"),
    ("ST_POSITION_CTRL", "ST_HOLD_POSITION", "ME low, shaper at rest",
     "stop"),
    ("ST_VELOCITY_CTRL", "ST_HOLD_POSITION", "ME low", "stop"),
    ("ST_TORQUE_CTRL", "ST_HOLD_POSITION", "ME low", "stop"),
    ("ST_PRESSURE_CTRL", "ST_HOLD_POSITION", "ME low", "stop"),
    ("ST_GO_HOME", "ST_HOLD_POSITION", "ME low", "stop"),
    ("ST_GO_HOME", "ST_FAULT", "move error", "fault"),
    ("ST_HOME_LIMIT", "ST_HOME_COMPLETE", "homing done", "main"),
    ("ST_HOME_LIMIT", "ST_HOLD_POSITION", "ME low", "stop"),
    ("ST_HOME_LIMIT", "ST_FAULT", "homing error", "fault"),
    ("ST_HOME_COMPLETE", "ST_HOLD_POSITION", "ME low", "stop"),
    # --- Hold and deactivation ---
    ("ST_HOLD_POSITION", "ST_POSITION_CTRL", "halted, HS, mode 2", "mode"),
    ("ST_HOLD_POSITION", "ST_BRAKE_HOLD", "halted, HS, mode 1", "mode"),
    ("ST_HOLD_POSITION", "ST_VELOCITY_CTRL", "halted, HS, mode 3", "mode"),
    ("ST_HOLD_POSITION", "ST_TORQUE_CTRL", "halted, HS, mode 4", "mode"),
    ("ST_HOLD_POSITION", "ST_GO_HOME", "halted, HS, mode 5", "mode"),
    ("ST_HOLD_POSITION", "ST_HOME_LIMIT", "halted, HS, mode 6", "mode"),
    ("ST_HOLD_POSITION", "ST_PRESSURE_CTRL", "halted, HS, mode 7", "mode"),
    ("ST_HOLD_POSITION", "ST_BRAKE_ENGAGE",
     "halted, mode 0 / hold timeout", "main"),
    ("ST_BRAKE_ENGAGE", "ST_DRIVE_DISABLE", "brake delay", "main"),
    ("ST_DRIVE_DISABLE", "ST_IDLE", "power off", "main"),
    # --- Faults ---
    ("ST_FAULT", "ST_BRAKE_HOLD", "reset valid", "fault"),
    ("ST_FAULT", "ST_RECOVERY", "reset valid, " + LIMIT, "recov"),
    ("ST_FAULT", "ST_FAULT_IDLE", "fault-idle timeout", "fault"),
    ("ST_FAULT_IDLE", "ST_IDLE", "reset valid", "fault"),
    ("ST_FAULT_IDLE", "ST_DRIVE_RESET", "reset valid, " + LIMIT, "recov"),
    # --- Limit recovery ---
    ("ST_RECOVERY", "ST_RECOVERY_POSITION", "HS, mode 2", "recov"),
    ("ST_RECOVERY", "ST_RECOVERY_VELOCITY", "HS, mode 3", "recov"),
    ("ST_RECOVERY", "ST_BRAKE_HOLD", "HS, mode 1", "recov"),
    ("ST_RECOVERY", "ST_BRAKE_ENGAGE", "HS, mode 0", "recov"),
    ("ST_RECOVERY_POSITION", "ST_RECOVERY", "ME low, " + LIMIT, "recov"),
    ("ST_RECOVERY_POSITION", "ST_HOLD_POSITION", "ME low, limit clear",
     "stop"),
    ("ST_RECOVERY_VELOCITY", "ST_RECOVERY", "ME low, " + LIMIT, "recov"),
    ("ST_RECOVERY_VELOCITY", "ST_HOLD_POSITION", "ME low, limit clear",
     "stop"),
]

# The Safety Monitoring block at the end of PRG_Main moves every state
# but ST_FAULT and ST_FAULT_IDLE to ST_FAULT when fbSafetyMonitor reports
# a fault (drive, link loss, limit switch, position, pressure sensor,
# piston exit). ST_DRIVE_RESET ignores a drive fault. It is drawn as one
# edge from a note, not from each state.

# %% ===== CHECKS =====


def enum_members():
    """Return the members of E_SystemState in declaration order."""
    text = DATA_TYPES.read_text()
    body = re.search(r"E_SystemState\s*:\s*\((.*?)\)\s*;", text, re.S)
    body = re.sub(r"\(\*.*?\*\)", "", body.group(1), flags=re.S)
    return re.findall(r"\bST_[A-Z_]+\b", body)


def main_targets():
    """Return {state: targets assigned to G_sysCurrentState in its branch}.

    Only the "State Machine" section is read (up to "State Properties");
    the safety override at the end of the file is handled by SAFETY_*.
    """
    text = PRG_MAIN.read_text()
    start = text.index("IF G_sysCurrentState = E_SystemState#ST_IDLE THEN")
    stop = text.index("State Properties", start)
    body = text[start:stop]
    parts = re.split(
        r"(?:IF|ELSIF) G_sysCurrentState = E_SystemState#(ST_[A-Z_]+) THEN",
        body)
    out = {}
    for name, block in zip(parts[1::2], parts[2::2]):
        out[name] = set(re.findall(
            r"G_sysCurrentState := E_SystemState#(ST_[A-Z_]+);", block))
    return out


def check():
    """Raise SystemExit when the table and the sources disagree."""
    names = [s[0] for s in STATES]
    problems = []
    if names != enum_members():
        problems.append("states differ from E_SystemState: %s" % (
            sorted(set(names) ^ set(enum_members())) or "order"))
    table = {}
    for src, dst, _, _ in TRANSITIONS:
        table.setdefault(src, set()).add(dst)
    code = main_targets()
    # ST_INIT has no branch: the first-scan block sets ST_IDLE.
    code["ST_INIT"] = {"ST_IDLE"}
    for name in names:
        if table.get(name, set()) != code.get(name, set()):
            problems.append("%s: table %s, PRG_Main %s" % (
                name, sorted(table.get(name, set())),
                sorted(code.get(name, set()))))
    if problems:
        sys.exit("draw_state_machine: table and sources disagree:\n  "
                 + "\n  ".join(problems))


# %% ===== DRAWING =====

EDGE_STYLE = {
    "main": dict(color="#222222", penwidth="2.2"),
    "mode": dict(color="#2E7D32", penwidth="1.4"),
    "stop": dict(color="#555555", penwidth="1.2"),
    "fault": dict(color="#C62828", penwidth="1.6"),
    "recov": dict(color="#E65100", penwidth="1.6"),
}


def node_text(name, mode, note):
    """HTML-like label: state name, mode code and note."""
    head = "<B>%s</B>" % name
    sub = note if mode is None else "mode %s: %s" % (mode, note)
    return '<%s<BR/><FONT POINT-SIZE="10">%s</FONT>>' % (head, sub)


# Edges drawn through a hub or a bus, because the same condition leads
# to or from several states. The table keeps every transition; draw()
# checks that each hub or bus stands for exactly the table's edges.
HUB_ENTRY = ("ST_BRAKE_RELEASE", "ST_HOLD_POSITION")
HUB_TARGETS = ["ST_BRAKE_HOLD", "ST_POSITION_CTRL", "ST_VELOCITY_CTRL",
               "ST_TORQUE_CTRL", "ST_PRESSURE_CTRL", "ST_GO_HOME",
               "ST_HOME_LIMIT"]
BUS_MEMBERS = ["ST_POSITION_CTRL", "ST_VELOCITY_CTRL", "ST_TORQUE_CTRL",
               "ST_PRESSURE_CTRL", "ST_GO_HOME", "ST_HOME_LIMIT",
               "ST_HOME_COMPLETE"]

# %% ===== LAYOUT =====
# Node centres [in], x to the right and y down. The normal path runs
# down the column x = 0; the states a mode code selects form the row
# below ST_BRAKE_RELEASE; the fault and recovery states stand to the
# right. The order keeps the long edges short.
POS = {
    "ST_INIT": (0, 0), "ST_IDLE": (0, 1.4), "ST_DRIVE_RESET": (0, 2.8),
    "ST_DRIVE_ENABLE": (0, 4.2), "ST_BRAKE_RELEASE": (0, 5.6),
    "ST_POSITION_CTRL": (-9.9, 9.0), "ST_VELOCITY_CTRL": (-6.6, 9.0),
    "ST_TORQUE_CTRL": (-3.3, 9.0), "ST_PRESSURE_CTRL": (0, 9.0),
    "ST_GO_HOME": (3.3, 9.0), "ST_HOME_LIMIT": (6.6, 9.0),
    "ST_HOME_COMPLETE": (10.8, 9.0), "ST_BRAKE_HOLD": (14.1, 9.0),
    "ST_HOLD_POSITION": (0, 13.2), "ST_BRAKE_ENGAGE": (0, 14.6),
    "ST_DRIVE_DISABLE": (0, 16.0),
    "ST_FAULT_IDLE": (18.0, 0.7), "ST_FAULT": (17.4, 4.4),
    "ST_RECOVERY": (19.2, 9.0),
    "ST_RECOVERY_POSITION": (17.4, 11.6),
    "ST_RECOVERY_VELOCITY": (21.6, 12.8),
}
POS_AUX = {
    "hub": (0, 6.8),            # the requested mode selects the state
    "bus": (0, 11.2),           # the motion states' exits join here
    "safety": (9.4, 2.6),       # the safety note
    # waypoints of the edges routed round the outside
    "w_hold_a": (-12.6, 13.2), "w_hold_b": (-12.6, 6.8),
    "w_other_a": (-11.2, 5.6), "w_other_b": (-11.2, 12.8),
    "w_dis_a": (-13.6, 16.0), "w_dis_b": (-13.6, 1.4),
    "w_bh_a": (11.6, 6.0), "w_bhd_a": (14.1, 16.0),
    "w_rp_a": (16.0, 10.2), "w_rv_a": (23.2, 10.6),
    "w_re_a": (19.5, 14.6),
}

# Routes: (from, to) -> (waypoints, index of the segment that carries
# the label). A route is drawn as plain line segments; the last has the
# arrow head.
ROUTE = {
    ("ST_BRAKE_RELEASE", "ST_HOLD_POSITION"): (
        ["w_other_a", "w_other_b"], 1),
    ("ST_DRIVE_DISABLE", "ST_IDLE"): (["w_dis_a", "w_dis_b"], 1),
    ("ST_BRAKE_HOLD", "ST_BRAKE_RELEASE"): (["w_bh_a"], 1),
    ("ST_BRAKE_HOLD", "ST_DRIVE_DISABLE"): (["w_bhd_a"], 1),
    ("ST_RECOVERY", "ST_BRAKE_ENGAGE"): (["w_re_a"], 0),
    ("ST_RECOVERY_POSITION", "ST_RECOVERY"): (["w_rp_a"], 0),
    ("ST_RECOVERY_VELOCITY", "ST_RECOVERY"): (["w_rv_a"], 0),
}
HOLD_TO_HUB = (["w_hold_a", "w_hold_b"], 1)


def pt(name):
    """Position of a node or auxiliary point [in -> pt, y up]."""
    x, y = POS.get(name) or POS_AUX[name]
    return "%.2f,%.2f" % (x * 72, -y * 72)


def draw():
    """Return the neato input (nodes pinned) for the diagram."""
    table = {(s, d): k for s, d, _, k in TRANSITIONS}
    for src in HUB_ENTRY:
        for dst in HUB_TARGETS:
            assert table[(src, dst)] == "mode", (src, dst)
    for src in BUS_MEMBERS:
        assert table[(src, "ST_HOLD_POSITION")] == "stop", src
    hub_edges = {(s, d) for s in HUB_ENTRY for d in HUB_TARGETS}
    bus_edges = {(s, "ST_HOLD_POSITION") for s in BUS_MEMBERS}

    L = []
    w = L.append
    w("digraph MainStateMachine {")
    w('  graph [splines=true, fontname="Helvetica", fontsize=11,'
      ' pad=0.3, labelloc=t, label=<<B>PRG_Main state machine'
      ' (E_SystemState)</B><BR/><FONT POINT-SIZE="10">Generated by'
      ' tools/draw_state_machine.py from the transition table.'
      ' HS = mode handshake complete, ME = filtered MotionEnable.'
      '<BR/>Mode codes: 001 brake hold, 010 position, 011 velocity,'
      ' 100 torque, 101 go home, 110 home to limit, 111 pressure'
      '</FONT>>];')
    w('  node [shape=box, style="rounded,filled", fontname="Helvetica",'
      ' fontsize=12, margin="0.15,0.08", penwidth=1.5];')
    w('  edge [fontname="Helvetica", fontsize=10, arrowsize=0.8];')
    for name, grp, mode, note in STATES:
        w('  %s [pos="%s!", label=%s, fillcolor="%s"];' % (
            name, pt(name), node_text(name, mode, note), FILL[grp]))
    for name in ("hub", "bus"):
        w('  %s [pos="%s!", shape=circle, style=filled,'
          ' fillcolor="#222222", label="", width=0.14,'
          ' fixedsize=true];' % (name, pt(name)))
    w('  safety [pos="%s!", shape=note, style=dashed, fillcolor=white,'
      ' color="#C62828", fontcolor="#C62828", fontsize=10,'
      ' label="any state except ST_FAULT and ST_FAULT_IDLE:\\n'
      'fbSafetyMonitor fault (drive, link loss, limit switch,\\n'
      'position, pressure sensor, piston exit)"];' % pt("safety"))
    for name in POS_AUX:
        if name.startswith("w_"):
            w('  %s [pos="%s!", shape=point, width=0.01];' % (
                name, pt(name)))

    def edge(src, dst, text, kind, arrow=True, extra=""):
        st = EDGE_STYLE[kind]
        a = 'color="%s", fontcolor="%s", penwidth=%s' % (
            st["color"], st["color"], st["penwidth"])
        if text:
            a += ', label="%s"' % text
        if not arrow:
            a += ", arrowhead=none"
        w("  %s -> %s [%s%s];" % (src, dst, a, extra))

    def routed(src, dst, text, kind, wps, lab, arrow=True):
        chain = [src] + wps + [dst]
        for i in range(len(chain) - 1):
            edge(chain[i], chain[i + 1], text if i == lab else "", kind,
                 arrow=arrow and i == len(chain) - 2)

    # Hub: the requested mode picks the state (mode code on the edge).
    edge("ST_BRAKE_RELEASE", "hub", "brake delay done", "mode")
    routed("ST_HOLD_POSITION", "hub", "halted, HS", "mode",
           HOLD_TO_HUB[0], HOLD_TO_HUB[1])
    for dst in HUB_TARGETS:
        edge("hub", dst, MODE_OF[dst], "mode")
    # Bus: ME low from any motion state.
    for src in BUS_MEMBERS:
        edge(src, "bus", "", "stop", arrow=False)
    edge("bus", "ST_HOLD_POSITION",
         "ME low (position: once the shaper is at rest)", "stop")

    for src, dst, text, kind in TRANSITIONS:
        if (src, dst) in hub_edges or (src, dst) in bus_edges:
            continue
        if (src, dst) in ROUTE:
            routed(src, dst, text, kind, *ROUTE[(src, dst)])
        else:
            edge(src, dst, text, kind)
    edge("safety", "ST_FAULT", "", "fault", extra=", style=dashed")
    w("}")
    return "\n".join(L)


MODE_OF = {s: m for s, _, m, _ in STATES if m}
MODE_OF["ST_RECOVERY"] = None


def render(dot_src, fmt, out):
    """Run neato (node positions pinned) on the source."""
    res = subprocess.run(["neato", "-n", "-T" + fmt, "-o", str(out)],
                         input=dot_src, text=True)
    if res.returncode:
        sys.exit("draw_state_machine: neato failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--png", metavar="FILE", help="also write a PNG")
    ap.add_argument("--dot", metavar="FILE", help="also write the .dot")
    ap.add_argument("--check", action="store_true",
                    help="check the table against the sources only")
    args = ap.parse_args()
    check()
    if args.check:
        print("ok  %d states, %d transitions" % (len(STATES),
                                                 len(TRANSITIONS)))
        return
    src = draw()
    if args.dot:
        Path(args.dot).write_text(src)
    render(src, "svg", SVG_OUT)
    if args.png:
        render(src, "png", args.png)
    print("wrote", SVG_OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
