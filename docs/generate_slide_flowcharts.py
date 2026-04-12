#!/usr/bin/env python3
"""Generate 16:9 presentation-ready flowcharts for Google Slides."""

import subprocess
import os

OUT_DIR = os.path.join(os.path.dirname(__file__), "slide_flowcharts")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Color palette (high contrast for projection) ─────────────────────────
C = {
    "bridge":    "#2563EB",  # blue-600
    "planner":   "#059669",  # green-600
    "mpc":       "#D97706",  # amber-600
    "robot":     "#DC2626",  # red-600
    "ros":       "#7C3AED",  # purple-600
    "sensor":    "#0891B2",  # cyan-600
    "text":      "#0F172A",  # slate-900
    "gray":      "#64748B",  # slate-500
    "lightblue": "#DBEAFE",
    "lightgreen":"#D1FAE5",
    "lightamber":"#FEF3C7",
    "lightred":  "#FEE2E2",
    "lightpurp": "#EDE9FE",
    "lightcyan": "#CFFAFE",
    "white":     "#FFFFFF",
}

# Google Slides = 10" x 5.625" (16:9). We render at 13.33 x 7.5 for padding.
SLIDE = 'size="13.33,7.5!"; ratio=fill;'
DPI = 150  # Good balance of quality and file size


def render_dot(name, dot_source):
    dot_path = os.path.join(OUT_DIR, f"{name}.dot")
    png_path = os.path.join(OUT_DIR, f"{name}.png")
    with open(dot_path, "w") as f:
        f.write(dot_source)
    subprocess.run(
        ["dot", "-Tpng", f"-Gdpi={DPI}", dot_path, "-o", png_path],
        check=True,
    )
    os.remove(dot_path)
    print(f"  -> {png_path}")


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 1: System Architecture Overview
# ══════════════════════════════════════════════════════════════════════════
def slide_system_overview():
    dot = f"""
digraph system_overview {{
    graph [
        bgcolor=white, rankdir=TB, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>FarNav Amiga — System Architecture</B></FONT>>,
        labelloc=t, labeljust=c, pad=0.8, nodesep=0.7, ranksep=0.9,
        margin=0.4
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=14,
          style="filled,rounded", shape=box, penwidth=2, margin="0.2,0.12"];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=11,
          color="{C['gray']}", penwidth=1.8, arrowsize=1.0];

    // ── Robot ──
    subgraph cluster_robot {{
        label=<<B><FONT POINT-SIZE="16" COLOR="{C['robot']}">Amiga Robot (gRPC)</FONT></B>>;
        style="dashed,rounded"; color="{C['robot']}"; bgcolor="{C['lightred']}";
        fontname="Helvetica Neue,Helvetica,Arial"; penwidth=2; margin=18;

        canbus [label="CAN Bus\\nPort 6001", fillcolor=white, color="{C['robot']}"];
        oak    [label="OAK-D Cameras\\nPort 50010", fillcolor=white, color="{C['robot']}"];
        gps    [label="GPS\\nPort 50010", fillcolor=white, color="{C['robot']}"];
        filter [label="State Filter\\nPort 20001", fillcolor=white, color="{C['robot']}"];
        motors [label="Drive Motors", fillcolor=white, color="{C['robot']}"];
    }}

    // ── Bridge ──
    subgraph cluster_bridge {{
        label=<<B><FONT POINT-SIZE="16" COLOR="{C['bridge']}">gRPC-to-ROS 2 Bridge</FONT></B>>;
        style="dashed,rounded"; color="{C['bridge']}"; bgcolor="{C['lightblue']}";
        fontname="Helvetica Neue,Helvetica,Arial"; penwidth=2; margin=18;

        bridge [label="amiga_bridge_node\\n\\nSensors → ROS 2 topics\\ncmd_vel → gRPC /twist",
                fillcolor=white, color="{C['bridge']}"];
    }}

    // ── ROS Topics ──
    subgraph cluster_ros {{
        label=<<B><FONT POINT-SIZE="16" COLOR="{C['ros']}">ROS 2 Topics</FONT></B>>;
        style="dashed,rounded"; color="{C['ros']}"; bgcolor="{C['lightpurp']}";
        fontname="Helvetica Neue,Helvetica,Arial"; penwidth=2; margin=18;

        odom_t  [label="/amiga/odom", fillcolor=white, color="{C['ros']}", shape=ellipse];
        gps_t   [label="/amiga/gps", fillcolor=white, color="{C['ros']}", shape=ellipse];
        cam_t   [label="/amiga/oak/*", fillcolor=white, color="{C['ros']}", shape=ellipse];
        path_t  [label="/farnav/global_path", fillcolor=white, color="{C['ros']}", shape=ellipse];
        cmd_t   [label="/amiga/cmd_vel", fillcolor=white, color="{C['ros']}", shape=ellipse];
    }}

    // ── Navigation ──
    subgraph cluster_nav {{
        label=<<B><FONT POINT-SIZE="16" COLOR="{C['planner']}">Navigation Stack</FONT></B>>;
        style="dashed,rounded"; color="{C['planner']}"; bgcolor="{C['lightgreen']}";
        fontname="Helvetica Neue,Helvetica,Arial"; penwidth=2; margin=18;

        planner [label="Global Planner\\nGPS → cubic spline path",
                 fillcolor=white, color="{C['planner']}"];
        mpc     [label="MPC Controller\\nCasADi/IPOPT, 20 Hz",
                 fillcolor=white, color="{C['mpc']}"];
    }}

    // ── Edges ──
    canbus -> bridge [label=" velocity, state ", color="{C['robot']}"];
    oak    -> bridge [label=" RGB, stereo, IMU ", color="{C['robot']}"];
    gps    -> bridge [label=" lat/lon ", color="{C['robot']}"];
    filter -> bridge [label=" pose ", color="{C['robot']}"];

    bridge -> odom_t [color="{C['bridge']}"];
    bridge -> gps_t  [color="{C['bridge']}"];
    bridge -> cam_t  [color="{C['bridge']}"];

    odom_t -> planner [label=" alignment ", color="{C['planner']}"];
    gps_t  -> planner [label=" anchor ", color="{C['planner']}"];
    odom_t -> mpc     [label=" robot state ", color="{C['mpc']}"];

    planner -> path_t [color="{C['planner']}"];
    path_t  -> mpc    [label=" reference ", color="{C['mpc']}"];

    mpc    -> cmd_t   [label=" v, ω ", color="{C['mpc']}"];
    cmd_t  -> bridge  [color="{C['bridge']}"];
    bridge -> motors  [label=" gRPC /twist ", color="{C['robot']}"];
}}
"""
    render_dot("slide_01_system_architecture", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 2: Bridge Data Flow (horizontal)
# ══════════════════════════════════════════════════════════════════════════
def slide_bridge_flow():
    dot = f"""
digraph bridge {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>gRPC-to-ROS 2 Bridge — Data Flow</B></FONT>>,
        labelloc=t, pad=0.6, nodesep=0.4, ranksep=1.0
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=13,
          style="filled,rounded", shape=box, penwidth=2, margin="0.18,0.1"];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=11,
          penwidth=1.8, arrowsize=1.0];

    // ── gRPC ──
    subgraph cluster_grpc {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['robot']}">Amiga gRPC</FONT></B>>;
        style="dashed,rounded"; color="{C['robot']}"; bgcolor="{C['lightred']}";
        penwidth=2; margin=14;

        g1 [label="canbus:6001\\n/state", fillcolor=white, color="{C['robot']}"];
        g2 [label="filter:20001\\n/state", fillcolor=white, color="{C['robot']}"];
        g3 [label="gps:50010\\n/pvt", fillcolor=white, color="{C['robot']}"];
        g4 [label="oak/0:50010\\n/rgb, /left", fillcolor=white, color="{C['robot']}"];
        g5 [label="oak/0:50010\\n/imu", fillcolor=white, color="{C['robot']}"];
    }}

    // ── Bridge ──
    subgraph cluster_bridge {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['bridge']}">amiga_bridge_node</FONT></B>>;
        style="dashed,rounded"; color="{C['bridge']}"; bgcolor="{C['lightblue']}";
        penwidth=2; margin=14;

        b1 [label="stream_canbus()", fillcolor=white, color="{C['bridge']}"];
        b2 [label="stream_filter()", fillcolor=white, color="{C['bridge']}"];
        b3 [label="stream_gps()", fillcolor=white, color="{C['bridge']}"];
        b4 [label="stream_oak_rgb()\\nstream_oak_left()", fillcolor=white, color="{C['bridge']}"];
        b5 [label="stream_oak_imu()", fillcolor=white, color="{C['bridge']}"];
        b6 [label="send_twist()", fillcolor=white, color="{C['bridge']}"];
    }}

    // ── ROS 2 ──
    subgraph cluster_ros {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['ros']}">ROS 2 Topics</FONT></B>>;
        style="dashed,rounded"; color="{C['ros']}"; bgcolor="{C['lightpurp']}";
        penwidth=2; margin=14;

        r1 [label="/amiga/twist\\n/battery, /control_state", fillcolor=white, color="{C['ros']}", shape=ellipse];
        r2 [label="/amiga/odom\\n+ /tf broadcast", fillcolor=white, color="{C['ros']}", shape=ellipse];
        r3 [label="/amiga/gps", fillcolor=white, color="{C['ros']}", shape=ellipse];
        r4 [label="/amiga/oak/*/compressed", fillcolor=white, color="{C['ros']}", shape=ellipse];
        r5 [label="/amiga/imu", fillcolor=white, color="{C['ros']}", shape=ellipse];
        r6 [label="/cmd_vel\\n/amiga/cmd_vel", fillcolor=white, color="{C['mpc']}", shape=ellipse];
    }}

    // ── Motor output ──
    motor [label="gRPC /twist\\n→ Motors", fillcolor="{C['lightred']}", color="{C['robot']}"];

    // ── Forward path ──
    g1 -> b1 [color="{C['robot']}"];
    g2 -> b2 [color="{C['robot']}"];
    g3 -> b3 [color="{C['robot']}"];
    g4 -> b4 [color="{C['robot']}"];
    g5 -> b5 [color="{C['robot']}"];

    b1 -> r1 [color="{C['bridge']}"];
    b2 -> r2 [color="{C['bridge']}"];
    b3 -> r3 [color="{C['bridge']}"];
    b4 -> r4 [color="{C['bridge']}"];
    b5 -> r5 [color="{C['bridge']}"];

    // ── Reverse path ──
    r6 -> b6 [color="{C['mpc']}"];
    b6 -> motor [color="{C['robot']}"];
}}
"""
    render_dot("slide_02_bridge_data_flow", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 3: Global Planner Pipeline (horizontal)
# ══════════════════════════════════════════════════════════════════════════
def slide_planner_pipeline():
    dot = f"""
digraph planner {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>Global Planner — Processing Pipeline</B></FONT>>,
        labelloc=t, pad=0.6, nodesep=0.35, ranksep=0.55
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=13,
          style="filled,rounded", shape=box, penwidth=2, margin="0.2,0.12"];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=11,
          color="{C['gray']}", penwidth=2, arrowsize=1.0];

    // ── Inputs ──
    subgraph cluster_in {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['robot']}">Inputs</FONT></B>>;
        style="dashed,rounded"; color="{C['robot']}"; bgcolor="{C['lightred']}";
        penwidth=2; margin=14;

        yaml [label="row_data.yaml\\nGPS waypoints\\nA → B", fillcolor=white, color="{C['robot']}"];
        gps  [label="/amiga/gps\\nFirst fix (lat, lon)", fillcolor=white, color="{C['robot']}"];
        odom [label="/amiga/odom\\nFirst reading (x, y)", fillcolor=white, color="{C['robot']}"];
    }}

    // ── Pipeline ──
    parse     [label="Parse\\nWaypoints", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    utm       [label="WGS84 → UTM\\npyproj\\nEPSG:32616", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    align     [label="GPS / Odom\\nAlignment\\noffset = odom − utm", fillcolor="{C['lightcyan']}", color="{C['sensor']}"];
    transform [label="UTM → Odom\\nFrame\\n+ offset", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    spline    [label="Cubic Spline\\nInterpolation\\nscipy (clamped)", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    sample    [label="Sample\\n500 points", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    arc       [label="Arc-Length θ\\ncumulative\\ndistance", fillcolor="{C['lightgreen']}", color="{C['planner']}"];

    // ── Outputs ──
    subgraph cluster_out {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['mpc']}">Published @ 1 Hz</FONT></B>>;
        style="dashed,rounded"; color="{C['mpc']}"; bgcolor="{C['lightamber']}";
        penwidth=2; margin=14;

        path_out   [label="/farnav/global_path\\n(x, y, z=θ) × 500", fillcolor=white, color="{C['mpc']}"];
        marker_out [label="/farnav/\\nactive_row_marker", fillcolor=white, color="{C['mpc']}"];
        info_out   [label="/farnav/path_info\\nJSON metadata", fillcolor=white, color="{C['mpc']}"];
    }}

    // ── Flow ──
    yaml -> parse;
    parse -> utm;
    gps  -> align;
    odom -> align;
    utm  -> align;
    align -> transform -> spline -> sample -> arc;
    arc -> path_out;
    arc -> marker_out;
    arc -> info_out;
}}
"""
    render_dot("slide_03_planner_pipeline", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 4: MPC Control Loop (horizontal)
# ══════════════════════════════════════════════════════════════════════════
def slide_mpc_loop():
    dot = f"""
digraph mpc {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>MPC Controller — 20 Hz Control Loop</B></FONT>>,
        labelloc=t, pad=0.6, nodesep=0.3, ranksep=0.5
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=13,
          style="filled,rounded", shape=box, penwidth=2, margin="0.18,0.1"];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=11,
          color="{C['gray']}", penwidth=1.8, arrowsize=1.0];

    // ── Trigger ──
    start [label="Timer\\n50 ms", fillcolor="{C['lightcyan']}", color="{C['sensor']}",
           shape=circle, width=1.0, fixedsize=true, fontsize=12];

    // ── Decisions ──
    ready [label="Ready?", shape=diamond, width=1.3, height=1.0,
           fillcolor="{C['lightblue']}", color="{C['bridge']}", fontsize=13];
    endchk [label="Near\\nend?", shape=diamond, width=1.3, height=1.0,
            fillcolor="{C['lightblue']}", color="{C['bridge']}", fontsize=13];

    // ── Process ──
    zero    [label="Publish\\nzero velocity", fillcolor="{C['lightred']}", color="{C['robot']}"];
    project [label="Project onto path\\n→ arc-length θ", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    errors  [label="Compute errors\\ne_c  (cross-track)\\ne_ψ  (heading)", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    ref     [label="Generate\\nN-step reference\\n(20 pts ahead)", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    decel   [label="Decelerate\\nv_ref → 0", fillcolor="{C['lightred']}", color="{C['robot']}"];
    stop_n  [label="Goal reached\\nStop + disable", fillcolor="{C['lightred']}", color="{C['robot']}"];
    solve   [label="Solve NLP\\nCasADi / IPOPT\\n→ (v_cmd, ω_cmd)", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    pub     [label="Publish\\n/amiga/cmd_vel", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    viz     [label="Publish\\npredicted path", fillcolor="{C['lightpurp']}", color="{C['ros']}"];
    warm    [label="Warm-start\\nfor next cycle", fillcolor="{C['lightblue']}", color="{C['bridge']}"];

    // ── Main flow ──
    start -> ready;
    ready -> project [label=" Yes ", color="{C['planner']}", penwidth=2.5];
    project -> errors;
    errors -> ref;
    ref -> endchk;
    endchk -> solve [label=" No ", color="{C['planner']}", penwidth=2.5];
    decel -> solve;
    solve -> pub;
    pub -> viz;
    viz -> warm;

    // ── Branches ──
    ready -> zero [label=" No ", color="{C['robot']}", style=dashed];
    endchk -> decel [label=" Close ", color="{C['mpc']}"];
    endchk -> stop_n [label=" At goal ", color="{C['robot']}", style=dashed];

    // ── Loop ──
    warm -> start [style=dashed, label=" next cycle ", color="{C['gray']}",
                   constraint=false];
}}
"""
    render_dot("slide_04_mpc_control_loop", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 5: Robot State Machine (horizontal)
# ══════════════════════════════════════════════════════════════════════════
def slide_state_machine():
    dot = f"""
digraph states {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>Amiga Control State Machine</B></FONT>>,
        labelloc=t, pad=0.6, nodesep=1.0, ranksep=1.8
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=14,
          style=filled, shape=circle, penwidth=2.5, width=1.6, fixedsize=true];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=12, penwidth=2];

    boot     [label="BOOT\\n(0)", fillcolor="{C['lightcyan']}", color="{C['sensor']}"];
    manual_r [label="MANUAL\\nREADY\\n(1)", fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    manual_a [label="MANUAL\\nACTIVE\\n(2)", fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    auto_r   [label="AUTO\\nREADY\\n(4)", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    auto_a   [label="AUTO\\nACTIVE\\n(5)", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    estop    [label="E-STOP\\n(6)", fillcolor="{C['lightred']}", color="{C['robot']}"];

    boot -> manual_r [label="  Power on  "];
    manual_r -> manual_a [label="  Joystick  "];
    manual_r -> auto_r [label="  Pendant → AUTO  ", color="{C['planner']}", penwidth=3];
    auto_r -> auto_a [label="  First /twist cmd  ", color="{C['mpc']}", penwidth=3];
    auto_a -> auto_r [label="  Cmds stop  ", style=dashed];
    auto_r -> manual_r [label="  Pendant → MANUAL  ", style=dashed];

    auto_a -> estop [color="{C['robot']}", style=dashed, label="  E-STOP  "];
    estop -> boot [label="  Reset  ", style=dashed, color="{C['robot']}"];
}}
"""
    render_dot("slide_05_state_machine", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 6: Startup Sequence (horizontal)
# ══════════════════════════════════════════════════════════════════════════
def slide_startup_sequence():
    dot = f"""
digraph startup {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>Full System Startup Sequence</B></FONT>>,
        labelloc=t, pad=0.6, nodesep=0.25, ranksep=0.45
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=12,
          style="filled,rounded", shape=box, penwidth=2, margin="0.15,0.1"];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=10,
          color="{C['gray']}", penwidth=2, arrowsize=1.0];

    // ── Steps ──
    s1  [label="1\\nPower on\\nAmiga", fillcolor="{C['lightred']}", color="{C['robot']}"];
    s2  [label="2\\nConnect to\\nnetwork", fillcolor="{C['lightred']}", color="{C['robot']}"];
    s3  [label="3\\nSource\\nROS 2", fillcolor="{C['lightpurp']}", color="{C['ros']}"];
    s4  [label="4\\nLaunch\\nBridge", fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    s5  [label="5\\nVerify\\nstreams", fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    s6  [label="6\\nLaunch\\nPlanner", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    s7  [label="7\\nGPS/Odom\\nalignment", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    s8  [label="8\\nLaunch\\nMPC", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    s9  [label="9\\nPendant\\n→ AUTO", fillcolor="{C['lightred']}", color="{C['robot']}"];
    s10 [label="10\\nEnable\\ncontroller", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    s11 [label="11\\nAutonomous\\nnavigation", fillcolor="{C['lightamber']}", color="{C['mpc']}",
         penwidth=3];
    s12 [label="12\\nReach goal\\nauto-stop", fillcolor="{C['lightgreen']}", color="{C['planner']}"];

    rviz [label="(Opt)\\nRViz", fillcolor="{C['lightpurp']}", color="{C['ros']}"];

    s1 -> s2 -> s3 -> s4 -> s5 -> s6 -> s7 -> s8 -> s9 -> s10 -> s11 -> s12;
    s5 -> rviz [style=dashed, label=" opt "];
}}
"""
    render_dot("slide_06_startup_sequence", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 7: Coordinate Transform Pipeline (horizontal)
# ══════════════════════════════════════════════════════════════════════════
def slide_coord_pipeline():
    dot = f"""
digraph coords {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>Coordinate Transform Pipeline</B></FONT>>,
        labelloc=t, pad=0.8, nodesep=0.8, ranksep=1.2
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=16,
          style="filled,rounded", shape=box, penwidth=2.5, width=2.8, height=1.2];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=14,
          penwidth=3, arrowsize=1.2];

    gps    [label="WGS84\\n(lat, lon)\\nGlobal GPS",
            fillcolor="{C['lightred']}", color="{C['robot']}"];
    utm    [label="UTM\\n(easting, northing)\\nMeters",
            fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    odom   [label="Odom Frame\\n(x, y)\\nRobot-local",
            fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    spline [label="Cubic Spline\\n(x(t), y(t))\\nSmooth path",
            fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    arc    [label="Arc-Length θ\\nCumulative\\ndistance",
            fillcolor="{C['lightpurp']}", color="{C['ros']}"];

    gps -> utm [label="  pyproj  ", color="{C['bridge']}"];
    utm -> odom [label="  + offset  ", color="{C['planner']}"];
    odom -> spline [label="  scipy CubicSpline  ", color="{C['mpc']}"];
    spline -> arc [label="  cumsum  ", color="{C['ros']}"];
}}
"""
    render_dot("slide_07_coordinate_pipeline", dot)


# ══════════════════════════════════════════════════════════════════════════
# SLIDE 8: MPC Solver Internals
# ══════════════════════════════════════════════════════════════════════════
def slide_mpc_solver():
    dot = f"""
digraph solver {{
    graph [
        bgcolor=white, rankdir=TB, fontname="Helvetica Neue,Helvetica,Arial",
        {SLIDE}
        label=<<FONT POINT-SIZE="28" COLOR="{C['text']}"><B>MPC Solver — CasADi / IPOPT</B></FONT>>,
        labelloc=t, pad=0.6, nodesep=0.6, ranksep=0.8
    ];
    node [fontname="Helvetica Neue,Helvetica,Arial", fontsize=14,
          style="filled,rounded", shape=box, penwidth=2, margin="0.2,0.12"];
    edge [fontname="Helvetica Neue,Helvetica,Arial", fontsize=12,
          color="{C['gray']}", penwidth=2, arrowsize=1.0];

    // ── Inputs ──
    subgraph cluster_in {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['sensor']}">Inputs</FONT></B>>;
        style="dashed,rounded"; color="{C['sensor']}"; bgcolor="{C['lightcyan']}";
        penwidth=2; margin=14;

        x0   [label="Robot State x₀\\n[X, Y, ψ, v]", fillcolor=white, color="{C['sensor']}"];
        ref  [label="Reference Trajectory\\nN points from path", fillcolor=white, color="{C['sensor']}"];
        prev [label="Previous Solution\\n(warm-start)", fillcolor=white, color="{C['sensor']}"];
    }}

    // ── Formulation ──
    dynamics [label="Dynamics (N=20)\\nUnicycle + velocity lag\\ndt = 0.1s, τ = 0.3s",
              fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    cost [label="Cost Function\\nw_cte·e²_c + w_head·e²_ψ + w_v·(v−v_ref)² + smoothness",
          fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    constr [label="Constraints\\n|e_c| ≤ 0.45m   |v±ωb| ≤ 1.0 m/s\\n|ω| ≤ 1.0 rad/s   |Δω| ≤ 0.5 rad/s",
            fillcolor="{C['lightamber']}", color="{C['mpc']}"];

    // ── Solver ──
    ipopt [label="IPOPT Interior-Point Solver\\nWarm-started, nonlinear optimization",
           fillcolor="{C['lightblue']}", color="{C['bridge']}"];

    // ── Outputs ──
    subgraph cluster_out {{
        label=<<B><FONT POINT-SIZE="15" COLOR="{C['planner']}">Outputs</FONT></B>>;
        style="dashed,rounded"; color="{C['planner']}"; bgcolor="{C['lightgreen']}";
        penwidth=2; margin=14;

        u [label="Optimal Control u*₀\\nv_cmd, ω_cmd → /amiga/cmd_vel", fillcolor=white, color="{C['planner']}"];
        pred [label="Predicted Trajectory\\nN+1 states → RViz", fillcolor=white, color="{C['planner']}"];
        ws [label="Full Solution\\n→ warm-start next cycle", fillcolor=white, color="{C['planner']}"];
    }}

    // ── Flow ──
    x0   -> dynamics;
    ref  -> cost;
    prev -> ipopt [style=dashed, label=" warm start "];

    dynamics -> ipopt;
    cost     -> ipopt;
    constr   -> ipopt;

    ipopt -> u;
    ipopt -> pred;
    ipopt -> ws;
}}
"""
    render_dot("slide_08_mpc_solver", dot)


# ══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("Generating slide-format flowcharts (16:9)...")
    slide_system_overview()
    slide_bridge_flow()
    slide_planner_pipeline()
    slide_mpc_loop()
    slide_state_machine()
    slide_startup_sequence()
    slide_coord_pipeline()
    slide_mpc_solver()
    print(f"\nDone! All slides saved to: {OUT_DIR}/")
