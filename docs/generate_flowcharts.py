#!/usr/bin/env python3
"""Generate system flowcharts for FarNav Amiga project."""

import subprocess
import os
import textwrap

OUT_DIR = os.path.join(os.path.dirname(__file__), "flowcharts")
os.makedirs(OUT_DIR, exist_ok=True)

# ── Color palette ─────────────────────────────────────────────────────────
C = {
    "bridge":   "#3B82F6",  # blue
    "planner":  "#10B981",  # green
    "mpc":      "#F59E0B",  # amber
    "robot":    "#EF4444",  # red
    "ros":      "#8B5CF6",  # purple
    "sensor":   "#06B6D4",  # cyan
    "bg":       "#FFFFFF",
    "text":     "#1E293B",
    "gray":     "#94A3B8",
    "lightblue":"#DBEAFE",
    "lightgreen":"#D1FAE5",
    "lightamber":"#FEF3C7",
    "lightred": "#FEE2E2",
    "lightpurp":"#EDE9FE",
    "lightcyan":"#CFFAFE",
}


def render_dot(name, dot_source):
    """Write a .dot file and render to PNG."""
    dot_path = os.path.join(OUT_DIR, f"{name}.dot")
    png_path = os.path.join(OUT_DIR, f"{name}.png")
    with open(dot_path, "w") as f:
        f.write(dot_source)
    subprocess.run(
        ["dot", "-Tpng", "-Gdpi=200", dot_path, "-o", png_path],
        check=True,
    )
    os.remove(dot_path)
    print(f"  -> {png_path}")


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 1: System Architecture Overview
# ══════════════════════════════════════════════════════════════════════════
def chart_system_overview():
    dot = f"""
digraph system_overview {{
    graph [
        bgcolor=white, rankdir=TB, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">FarNav Amiga — System Architecture</FONT></B>>,
        labelloc=t, labeljust=c, pad=0.5, nodesep=0.6, ranksep=0.8
    ];
    node [fontname="Helvetica", fontsize=11, style="filled,rounded", shape=box, penwidth=1.5];
    edge [fontname="Helvetica", fontsize=9, color="{C['gray']}", penwidth=1.2];

    // ── Robot Hardware ──
    subgraph cluster_robot {{
        label=<<B><FONT POINT-SIZE="14" COLOR="{C['robot']}">Amiga Robot Hardware</FONT></B>>;
        style="dashed,rounded"; color="{C['robot']}"; bgcolor="{C['lightred']}";
        fontname="Helvetica"; penwidth=1.5; margin=20;

        canbus  [label="CAN Bus\\nPort 6001", fillcolor=white, color="{C['robot']}"];
        oak     [label="OAK-D Cameras\\nPort 50010", fillcolor=white, color="{C['robot']}"];
        gps     [label="GPS Receiver\\nPort 50010", fillcolor=white, color="{C['robot']}"];
        filter  [label="State Filter\\nPort 20001", fillcolor=white, color="{C['robot']}"];
        motors  [label="Drive Motors", fillcolor=white, color="{C['robot']}"];
    }}

    // ── Bridge ──
    subgraph cluster_bridge {{
        label=<<B><FONT POINT-SIZE="14" COLOR="{C['bridge']}">gRPC-to-ROS 2 Bridge</FONT></B>>;
        style="dashed,rounded"; color="{C['bridge']}"; bgcolor="{C['lightblue']}";
        fontname="Helvetica"; penwidth=1.5; margin=20;

        bridge [label="amiga_bridge_node\\n\\nStreams sensors → ROS 2 topics\\nForwards cmd_vel → gRPC /twist",
                fillcolor=white, color="{C['bridge']}"];
    }}

    // ── Navigation ──
    subgraph cluster_nav {{
        label=<<B><FONT POINT-SIZE="14" COLOR="{C['planner']}">Navigation Stack</FONT></B>>;
        style="dashed,rounded"; color="{C['planner']}"; bgcolor="{C['lightgreen']}";
        fontname="Helvetica"; penwidth=1.5; margin=20;

        planner [label="Global Planner\\n\\nGPS waypoints → cubic spline\\nArc-length parameterized path",
                 fillcolor=white, color="{C['planner']}"];
        mpc     [label="MPC Controller\\n\\nCasADi/IPOPT solver, 20 Hz\\nOptimal velocity commands",
                 fillcolor=white, color="{C['mpc']}"];
    }}

    // ── ROS 2 Topics ──
    subgraph cluster_ros {{
        label=<<B><FONT POINT-SIZE="14" COLOR="{C['ros']}">ROS 2 Topics</FONT></B>>;
        style="dashed,rounded"; color="{C['ros']}"; bgcolor="{C['lightpurp']}";
        fontname="Helvetica"; penwidth=1.5; margin=20;

        odom_topic [label="/amiga/odom\\nOdometry", fillcolor=white, color="{C['ros']}", shape=ellipse];
        gps_topic  [label="/amiga/gps\\nNavSatFix", fillcolor=white, color="{C['ros']}", shape=ellipse];
        cam_topic  [label="/amiga/oak/*/compressed\\nCompressedImage", fillcolor=white, color="{C['ros']}", shape=ellipse];
        path_topic [label="/farnav/global_path\\nPath", fillcolor=white, color="{C['ros']}", shape=ellipse];
        cmd_topic  [label="/amiga/cmd_vel\\nTwistStamped", fillcolor=white, color="{C['ros']}", shape=ellipse];
    }}

    // ── Edges ──
    canbus -> bridge [label="  velocity, battery,\\n  control state  ", color="{C['robot']}"];
    oak    -> bridge [label="  RGB, stereo,\\n  IMU  ", color="{C['robot']}"];
    gps    -> bridge [label="  lat/lon/alt  ", color="{C['robot']}"];
    filter -> bridge [label="  pose +\\n  velocity  ", color="{C['robot']}"];

    bridge -> odom_topic [color="{C['bridge']}"];
    bridge -> gps_topic  [color="{C['bridge']}"];
    bridge -> cam_topic  [color="{C['bridge']}"];

    odom_topic -> planner [label="  alignment  ", color="{C['planner']}"];
    gps_topic  -> planner [label="  waypoint anchor  ", color="{C['planner']}"];
    odom_topic -> mpc     [label="  robot state  ", color="{C['mpc']}"];

    planner -> path_topic [color="{C['planner']}"];
    path_topic -> mpc     [label="  reference path  ", color="{C['mpc']}"];

    mpc -> cmd_topic      [label="  v, ω  ", color="{C['mpc']}"];
    cmd_topic -> bridge   [label="  ", color="{C['bridge']}"];
    bridge -> motors      [label="  gRPC /twist  ", color="{C['robot']}"];
}}
"""
    render_dot("01_system_architecture", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 2: Bridge Data Flow
# ══════════════════════════════════════════════════════════════════════════
def chart_bridge_flow():
    dot = f"""
digraph bridge_flow {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">gRPC-to-ROS 2 Bridge — Data Flow</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.5, ranksep=1.2
    ];
    node [fontname="Helvetica", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5];
    edge [fontname="Helvetica", fontsize=9, penwidth=1.2];

    // ── gRPC Sources ──
    subgraph cluster_grpc {{
        label=<<B><FONT POINT-SIZE="12" COLOR="{C['robot']}">Amiga gRPC Services</FONT></B>>;
        style="dashed,rounded"; color="{C['robot']}"; bgcolor="{C['lightred']}";
        margin=15;

        can_state [label="canbus:6001\\n/state", fillcolor=white, color="{C['robot']}"];
        filt_state [label="filter:20001\\n/state", fillcolor=white, color="{C['robot']}"];
        gps_pvt [label="gps:50010\\n/pvt", fillcolor=white, color="{C['robot']}"];
        oak_rgb [label="oak/0:50010\\n/rgb", fillcolor=white, color="{C['robot']}"];
        oak_left [label="oak/0:50010\\n/left", fillcolor=white, color="{C['robot']}"];
        oak_imu [label="oak/0:50010\\n/imu", fillcolor=white, color="{C['robot']}"];
    }}

    // ── Bridge ──
    subgraph cluster_bridge {{
        label=<<B><FONT POINT-SIZE="12" COLOR="{C['bridge']}">amiga_bridge_node</FONT></B>>;
        style="dashed,rounded"; color="{C['bridge']}"; bgcolor="{C['lightblue']}";
        margin=15;

        stream_can [label="stream_canbus()\\nasync task", fillcolor=white, color="{C['bridge']}"];
        stream_filt [label="stream_filter()\\nasync task", fillcolor=white, color="{C['bridge']}"];
        stream_gps [label="stream_gps()\\nasync task", fillcolor=white, color="{C['bridge']}"];
        stream_rgb [label="stream_oak_rgb()\\nasync task", fillcolor=white, color="{C['bridge']}"];
        stream_left [label="stream_oak_left()\\nasync task", fillcolor=white, color="{C['bridge']}"];
        stream_imu [label="stream_oak_imu()\\nasync task", fillcolor=white, color="{C['bridge']}"];
        send_twist [label="send_twist()\\ncallback", fillcolor=white, color="{C['bridge']}"];
    }}

    // ── ROS 2 Topics ──
    subgraph cluster_topics {{
        label=<<B><FONT POINT-SIZE="12" COLOR="{C['ros']}">ROS 2 Topics</FONT></B>>;
        style="dashed,rounded"; color="{C['ros']}"; bgcolor="{C['lightpurp']}";
        margin=15;

        t_twist [label="/amiga/twist\\n+ /battery\\n+ /control_state", fillcolor=white, color="{C['ros']}", shape=ellipse];
        t_odom [label="/amiga/odom\\n+ /tf", fillcolor=white, color="{C['ros']}", shape=ellipse];
        t_gps [label="/amiga/gps", fillcolor=white, color="{C['ros']}", shape=ellipse];
        t_rgb [label="/amiga/oak/rgb/\\ncompressed", fillcolor=white, color="{C['ros']}", shape=ellipse];
        t_left [label="/amiga/oak/left/\\ncompressed", fillcolor=white, color="{C['ros']}", shape=ellipse];
        t_imu [label="/amiga/imu", fillcolor=white, color="{C['ros']}", shape=ellipse];
        t_cmd [label="/cmd_vel\\n/amiga/cmd_vel", fillcolor=white, color="{C['mpc']}", shape=ellipse];
    }}

    // ── gRPC command target ──
    twist_ep [label="gRPC /twist\\n→ Amiga motors", fillcolor="{C['lightred']}", color="{C['robot']}",
              shape=box, style="filled,rounded"];

    // ── Edges: gRPC → bridge ──
    can_state  -> stream_can  [color="{C['robot']}"];
    filt_state -> stream_filt [color="{C['robot']}"];
    gps_pvt    -> stream_gps  [color="{C['robot']}"];
    oak_rgb    -> stream_rgb  [color="{C['robot']}"];
    oak_left   -> stream_left [color="{C['robot']}"];
    oak_imu    -> stream_imu  [color="{C['robot']}"];

    // ── Edges: bridge → ROS ──
    stream_can  -> t_twist [color="{C['bridge']}"];
    stream_filt -> t_odom  [color="{C['bridge']}"];
    stream_gps  -> t_gps   [color="{C['bridge']}"];
    stream_rgb  -> t_rgb   [color="{C['bridge']}"];
    stream_left -> t_left  [color="{C['bridge']}"];
    stream_imu  -> t_imu   [color="{C['bridge']}"];

    // ── Edges: ROS → bridge → gRPC ──
    t_cmd -> send_twist [color="{C['mpc']}"];
    send_twist -> twist_ep [color="{C['robot']}"];
}}
"""
    render_dot("02_bridge_data_flow", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 3: Global Planner Pipeline
# ══════════════════════════════════════════════════════════════════════════
def chart_planner_pipeline():
    dot = f"""
digraph planner_pipeline {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">Global Planner — Processing Pipeline</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.35, ranksep=0.6
    ];
    node [fontname="Helvetica", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5, width=2.0];
    edge [fontname="Helvetica", fontsize=8, color="{C['gray']}", penwidth=1.5];

    // ── Input (rank 1) ──
    subgraph cluster_input {{
        label=<<B><FONT POINT-SIZE="11" COLOR="{C['robot']}">Inputs</FONT></B>>;
        style="dashed,rounded"; color="{C['robot']}"; bgcolor="{C['lightred']}"; margin=12;

        yaml [label="row_data.yaml\\nGPS waypoints\\nA → intermediates → B",
              fillcolor=white, color="{C['robot']}"];
        gps_fix [label="/amiga/gps\\nFirst GPS fix\\n(lat, lon)",
                 fillcolor=white, color="{C['robot']}"];
        odom_fix [label="/amiga/odom\\nFirst odom reading\\n(x, y)",
                  fillcolor=white, color="{C['robot']}"];
    }}

    // ── Processing ──
    parse [label="Parse\\nWaypoints",
           fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    utm [label="WGS84 → UTM\\npyproj EPSG:32616",
         fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    align [label="GPS / Odom\\nAlignment\\noffset = odom − utm",
           fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    transform [label="UTM → Odom\\nFrame\\n+ offset",
               fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    spline [label="Cubic Spline\\nInterpolation\\nscipy (clamped)",
            fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    sample [label="Sample\\n500 points",
            fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    arclength [label="Arc-Length θ\\ncumulative\\ndistance",
               fillcolor="{C['lightgreen']}", color="{C['planner']}"];

    // ── Output ──
    subgraph cluster_output {{
        label=<<B><FONT POINT-SIZE="11" COLOR="{C['mpc']}">Published Topics</FONT></B>>;
        style="dashed,rounded"; color="{C['mpc']}"; bgcolor="{C['lightamber']}"; margin=12;

        path_out [label="/farnav/global_path\\n(x, y, z=θ) × 500 pts\\n@ 1 Hz",
                  fillcolor=white, color="{C['mpc']}"];
        marker_out [label="/farnav/\\nactive_row_marker\\nLINE_STRIP",
                    fillcolor=white, color="{C['mpc']}"];
        info_out [label="/farnav/path_info\\nJSON metadata",
                  fillcolor=white, color="{C['mpc']}"];
    }}

    // ── Edges ──
    yaml -> parse;
    parse -> utm;
    gps_fix -> align;
    odom_fix -> align;
    utm -> align;
    align -> transform;
    transform -> spline;
    spline -> sample;
    sample -> arclength;
    arclength -> path_out;
    arclength -> marker_out;
    arclength -> info_out;
}}
"""
    render_dot("03_planner_pipeline", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 4: MPC Control Loop
# ══════════════════════════════════════════════════════════════════════════
def chart_mpc_loop():
    dot = f"""
digraph mpc_loop {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">MPC Controller — 20 Hz Control Loop</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.3, ranksep=0.55
    ];
    node [fontname="Helvetica", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5, width=1.8];
    edge [fontname="Helvetica", fontsize=8, color="{C['gray']}", penwidth=1.5];

    // ── Start ──
    start [label="Timer fires\\n(every 50 ms)",
           fillcolor="{C['lightcyan']}", color="{C['sensor']}"];

    // ── Decision nodes ──
    check_ready [label="Enabled?\\nPath?\\nOdom?",
                 shape=diamond, width=1.6, height=1.2,
                 fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    check_end   [label="Near end\\nof path?",
                 shape=diamond, width=1.6, height=1.2,
                 fillcolor="{C['lightblue']}", color="{C['bridge']}"];

    // ── Process nodes ──
    zero [label="Publish zero\\nvelocity (v=0, ω=0)",
          fillcolor="{C['lightred']}", color="{C['robot']}"];
    project [label="Project onto path\\nproject_onto_path()\\n→ arc-length θ",
             fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    errors [label="Compute errors\\ne_c (cross-track)\\ne_ψ (heading)",
            fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    ref [label="Generate N-step\\nreference\\n20 pts on spline",
         fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    decel [label="Decelerate\\nv_ref → 0",
           fillcolor="{C['lightred']}", color="{C['robot']}"];
    stop_node [label="Goal reached\\nStop + disable",
          fillcolor="{C['lightred']}", color="{C['robot']}"];
    solve [label="Solve NLP\\nCasADi/IPOPT\\n→ (v_cmd, ω_cmd)",
           fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    publish [label="Publish\\n/amiga/cmd_vel",
             fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    viz [label="Publish\\npredicted path\\n(N+1 states)",
         fillcolor="{C['lightpurp']}", color="{C['ros']}"];
    warm [label="Store for\\nwarm-start",
          fillcolor="{C['lightblue']}", color="{C['bridge']}"];

    // ── Main flow (left to right) ──
    start -> check_ready;
    check_ready -> project [label=" Yes ", color="{C['planner']}"];
    project -> errors;
    errors -> ref;
    ref -> check_end;
    check_end -> solve [label=" No ", color="{C['planner']}"];
    decel -> solve;
    solve -> publish;
    publish -> viz;
    viz -> warm;

    // ── Branch: not ready ──
    check_ready -> zero [label=" No ", color="{C['robot']}"];

    // ── Branch: near end ──
    check_end -> decel [label=" Close ", color="{C['mpc']}"];
    check_end -> stop_node [label=" At goal ", color="{C['robot']}"];

    // ── Loop back ──
    warm -> start [style=dashed, label=" next cycle ", color="{C['gray']}",
                   constraint=false];
}}
"""
    render_dot("04_mpc_control_loop", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 5: Robot Control State Machine
# ══════════════════════════════════════════════════════════════════════════
def chart_state_machine():
    dot = f"""
digraph state_machine {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">Amiga Control State Machine</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.8, ranksep=1.5
    ];
    node [fontname="Helvetica", fontsize=11, style="filled", shape=circle,
          penwidth=2, width=1.5, fixedsize=true];
    edge [fontname="Helvetica", fontsize=10, penwidth=1.5];

    boot [label="BOOT\\n(0)", fillcolor="{C['lightcyan']}", color="{C['sensor']}"];
    manual_r [label="MANUAL\\nREADY\\n(1)", fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    manual_a [label="MANUAL\\nACTIVE\\n(2)", fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    cc [label="CRUISE\\nCONTROL\\n(3)", fillcolor="{C['lightpurp']}", color="{C['ros']}"];
    auto_r [label="AUTO\\nREADY\\n(4)", fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    auto_a [label="AUTO\\nACTIVE\\n(5)", fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    estop [label="E-STOP\\n(6)", fillcolor="{C['lightred']}", color="{C['robot']}"];

    boot -> manual_r [label="  Power on  "];
    manual_r -> manual_a [label="  Joystick input  "];
    manual_r -> auto_r [label="  Pendant → AUTO  ", color="{C['planner']}"];
    auto_r -> auto_a [label="  First /twist cmd  ", color="{C['mpc']}", penwidth=2.5];
    auto_a -> auto_r [label="  Commands stop  ", style=dashed];
    auto_r -> manual_r [label="  Pendant → MANUAL  "];
    manual_r -> cc [label="  Cruise btn  "];

    // E-stop from anywhere
    manual_a -> estop [color="{C['robot']}", style=dashed];
    auto_a -> estop [color="{C['robot']}", style=dashed];
    auto_r -> estop [color="{C['robot']}", style=dashed, label="  E-STOP  "];
    estop -> boot [label="  Reset  ", style=dashed, color="{C['robot']}"];
}}
"""
    render_dot("05_state_machine", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 6: Full Startup Sequence
# ══════════════════════════════════════════════════════════════════════════
def chart_startup_sequence():
    dot = f"""
digraph startup {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">Full System Startup Sequence</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.3, ranksep=0.5
    ];
    node [fontname="Helvetica", fontsize=10, style="filled,rounded", shape=box, penwidth=1.5, width=2.0];
    edge [fontname="Helvetica", fontsize=8, color="{C['gray']}", penwidth=1.5];

    // ── Row 1: Hardware setup ──
    s1 [label="1. Power on\\nAmiga robot",
        fillcolor="{C['lightred']}", color="{C['robot']}"];
    s2 [label="2. Connect to\\nrobot network",
        fillcolor="{C['lightred']}", color="{C['robot']}"];
    s3 [label="3. Source ROS 2\\nworkspace",
        fillcolor="{C['lightpurp']}", color="{C['ros']}"];

    // ── Row 2: Bridge ──
    s4 [label="4. Launch\\nBridge",
        fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    s5 [label="5. Verify\\nsensor streams",
        fillcolor="{C['lightblue']}", color="{C['bridge']}"];

    // ── Row 3: Navigation ──
    s6 [label="6. Launch\\nGlobal Planner",
        fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    s7 [label="7. Wait for\\nGPS/Odom align",
        fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    s8 [label="8. Launch\\nMPC Controller",
        fillcolor="{C['lightamber']}", color="{C['mpc']}"];

    // ── Row 4: Activate ──
    s9 [label="9. Pendant\\n→ AUTO",
        fillcolor="{C['lightred']}", color="{C['robot']}"];
    s10 [label="10. Enable\\ncontroller",
         fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    s11 [label="11. Autonomous\\nnavigation\\nAUTO_ACTIVE",
         fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    s12 [label="12. Reach goal\\nauto-stop",
         fillcolor="{C['lightgreen']}", color="{C['planner']}"];

    // ── Optional ──
    rviz [label="(Optional)\\nLaunch RViz",
          fillcolor="{C['lightpurp']}", color="{C['ros']}"];

    // ── Main chain ──
    s1 -> s2 -> s3 -> s4 -> s5 -> s6 -> s7 -> s8 -> s9 -> s10 -> s11 -> s12;
    s5 -> rviz [style=dashed, label=" opt "];
}}
"""
    render_dot("06_startup_sequence", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 7: Coordinate Transform Pipeline
# ══════════════════════════════════════════════════════════════════════════
def chart_coordinate_pipeline():
    dot = f"""
digraph coords {{
    graph [
        bgcolor=white, rankdir=LR, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">Coordinate Transform Pipeline</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.6, ranksep=1.0
    ];
    node [fontname="Helvetica", fontsize=12, style="filled,rounded", shape=box, penwidth=2, width=2.5];
    edge [fontname="Helvetica", fontsize=10, penwidth=2];

    gps [label="WGS84\\n(lat, lon)\\nGlobal GPS coords",
         fillcolor="{C['lightred']}", color="{C['robot']}"];
    utm [label="UTM\\n(easting, northing)\\nMeters, EPSG:32616",
         fillcolor="{C['lightblue']}", color="{C['bridge']}"];
    odom [label="Odom Frame\\n(x, y)\\nRobot-local meters",
          fillcolor="{C['lightgreen']}", color="{C['planner']}"];
    spline [label="Spline Path\\n(x(t), y(t))\\nCubic interpolation",
            fillcolor="{C['lightamber']}", color="{C['mpc']}"];
    arc [label="Arc-Length θ\\nCumulative distance\\nalong path",
         fillcolor="{C['lightpurp']}", color="{C['ros']}"];

    gps -> utm [label="  pyproj  ", color="{C['bridge']}"];
    utm -> odom [label="  + offset  ", color="{C['planner']}"];
    odom -> spline [label="  scipy\\n  CubicSpline  ", color="{C['mpc']}"];
    spline -> arc [label="  cumulative\\n  distance  ", color="{C['ros']}"];
}}
"""
    render_dot("07_coordinate_pipeline", dot)


# ══════════════════════════════════════════════════════════════════════════
# FLOWCHART 8: MPC Solver Internals
# ══════════════════════════════════════════════════════════════════════════
def chart_mpc_solver():
    dot = f"""
digraph mpc_solver {{
    graph [
        bgcolor=white, rankdir=TB, fontname="Helvetica",
        label=<<B><FONT POINT-SIZE="22" COLOR="{C['text']}">MPC Solver — CasADi/IPOPT Internals</FONT></B>>,
        labelloc=t, pad=0.5, nodesep=0.5, ranksep=0.7
    ];
    node [fontname="Helvetica", fontsize=11, style="filled,rounded", shape=box, penwidth=1.5, width=3.5];
    edge [fontname="Helvetica", fontsize=9, color="{C['gray']}", penwidth=1.5];

    // ── Inputs ──
    subgraph cluster_inputs {{
        label=<<B><FONT POINT-SIZE="12" COLOR="{C['sensor']}">Inputs</FONT></B>>;
        style="dashed,rounded"; color="{C['sensor']}"; bgcolor="{C['lightcyan']}"; margin=15;

        x0 [label="Robot State x₀\\n[X, Y, ψ, v]\\nfrom /amiga/odom", fillcolor=white, color="{C['sensor']}"];
        path_ref [label="Reference Trajectory\\nN points from global path\\n(x_ref, y_ref, ψ_ref)", fillcolor=white, color="{C['sensor']}"];
        prev_sol [label="Previous Solution\\nWarm-start initial guess", fillcolor=white, color="{C['sensor']}"];
    }}

    // ── Formulation ──
    dynamics [label="Dynamics Model (N=20 steps)\\nx_{'{k+1}'} = f(x_k, u_k)\\nUnicycle + velocity lag (τ=0.3s)",
              fillcolor="{C['lightamber']}", color="{C['mpc']}"];

    cost [label=<Cost Function<BR/>
J = Σ [ w<SUB>cte</SUB>·e<SUB>c</SUB>² + w<SUB>head</SUB>·e<SUB>ψ</SUB>² + w<SUB>v</SUB>·(v−v<SUB>ref</SUB>)²<BR/>
    + w<SUB>Δv</SUB>·Δv² + w<SUB>Δω</SUB>·Δω² ]>,
         fillcolor="{C['lightamber']}", color="{C['mpc']}"];

    constraints [label="Constraints\\n|e_c| ≤ 0.45m  (row width)\\n|v ± ωb| ≤ 1.0 m/s  (wheel limits)\\n|ω| ≤ 1.0 rad/s  |Δω| ≤ 0.5 rad/s",
                 fillcolor="{C['lightamber']}", color="{C['mpc']}"];

    // ── Solver ──
    ipopt [label="IPOPT Nonlinear Solver\\nInterior-point method\\nWarm-started from previous solution",
           fillcolor="{C['lightblue']}", color="{C['bridge']}"];

    // ── Outputs ──
    subgraph cluster_outputs {{
        label=<<B><FONT POINT-SIZE="12" COLOR="{C['planner']}">Outputs</FONT></B>>;
        style="dashed,rounded"; color="{C['planner']}"; bgcolor="{C['lightgreen']}"; margin=15;

        u_star [label="Optimal Control u*₀\\nv_cmd (m/s), ω_cmd (rad/s)\\n→ /amiga/cmd_vel", fillcolor=white, color="{C['planner']}"];
        pred [label="Predicted Trajectory\\nN+1 states\\n→ /farnav/mpc_predicted_path", fillcolor=white, color="{C['planner']}"];
        warm_out [label="Full Solution\\nStored for warm-start\\non next cycle", fillcolor=white, color="{C['planner']}"];
    }}

    // ── Edges ──
    x0 -> dynamics;
    path_ref -> cost;
    prev_sol -> ipopt [style=dashed, label="  warm start  "];

    dynamics -> ipopt;
    cost -> ipopt;
    constraints -> ipopt;

    ipopt -> u_star;
    ipopt -> pred;
    ipopt -> warm_out;
}}
"""
    render_dot("08_mpc_solver_internals", dot)


# ══════════════════════════════════════════════════════════════════════════
# Run all
# ══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("Generating flowcharts...")
    chart_system_overview()
    chart_bridge_flow()
    chart_planner_pipeline()
    chart_mpc_loop()
    chart_state_machine()
    chart_startup_sequence()
    chart_coordinate_pipeline()
    chart_mpc_solver()
    print(f"\nDone! All flowcharts saved to: {OUT_DIR}/")
