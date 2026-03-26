"""
Amiga Project — Technical Architecture PDF
6 pages of real codebase diagrams. v2 — fixed alignment.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import FancyBboxPatch
import numpy as np

plt.rcParams.update({
    'font.family': 'sans-serif',
    'font.sans-serif': ['DejaVu Sans'],
    'font.size': 10,
    'axes.facecolor': 'white',
    'figure.facecolor': 'white',
    'axes.edgecolor': '#cccccc',
    'axes.grid': False,
    'axes.spines.top': False,
    'axes.spines.right': False,
    'axes.spines.left': False,
    'axes.spines.bottom': False,
})

BLUE    = '#266EF1'
TEAL    = '#00B4D8'
GREEN   = '#2ECC71'
ORANGE  = '#F09319'
RED     = '#E74C3C'
PURPLE  = '#9B59B6'
NAVY    = '#1A1A2E'
DGRAY   = '#333333'
MGRAY   = '#666666'
LGRAY   = '#999999'
VLGRAY  = '#e0e0e0'
BG_BLUE = '#EBF3FE'
BG_GREEN= '#E8FAF0'
BG_ORANGE='#FFF5E6'
BG_RED  = '#FDECEB'
BG_PURPLE='#F4EBF9'
BG_GRAY = '#F5F5F5'
DARK_BG = '#2C3E50'

LIGHT_COLORS = {BG_BLUE, BG_GREEN, BG_ORANGE, BG_RED, BG_PURPLE, BG_GRAY, 'white', '#F5F5F5', VLGRAY}

def box(ax, x, y, w, h, color, text, fontsize=9, text_color=None, alpha=1.0):
    r = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.08', facecolor=color, edgecolor='none', alpha=alpha)
    ax.add_patch(r)
    tc = text_color or ('white' if color not in LIGHT_COLORS else DGRAY)
    ax.text(x + w/2, y + h/2, text, ha='center', va='center', fontsize=fontsize,
            color=tc, fontweight='bold', linespacing=1.4, clip_on=True)

def arrow(ax, x1, y1, x2, y2, color='#aaaaaa', style='->', lw=1.5):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw))

def section_bg(ax, x, y, w, h, color, title='', title_size=10):
    r = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.1', facecolor=color,
                        edgecolor=VLGRAY, alpha=0.5, linewidth=1)
    ax.add_patch(r)
    if title:
        ax.text(x + 0.15, y + h - 0.15, title, fontsize=title_size, color=MGRAY, fontweight='bold', va='top')

def page_header(fig, title, subtitle=''):
    fig.text(0.06, 0.955, title, fontsize=20, fontweight='bold', color=NAVY)
    if subtitle:
        fig.text(0.06, 0.925, subtitle, fontsize=10, color=LGRAY)
    fig.add_artist(plt.Line2D([0.06, 0.94], [0.915, 0.915], transform=fig.transFigure, color=VLGRAY, linewidth=1))

def footer(fig, page, total=6):
    fig.text(0.06, 0.02, 'Amiga Robot Project — Technical Architecture', fontsize=8, color=LGRAY)
    fig.text(0.94, 0.02, f'{page}/{total}', fontsize=8, color=LGRAY, ha='right')


pdf_path = "/home/het/lab/amiga_project/27marchppt/amiga_system_overview.pdf"

with PdfPages(pdf_path) as pdf:

    # ═══════════════════════════════════════════════════════
    # PAGE 1 — Full Codebase Architecture (unchanged — looked fine)
    # ═══════════════════════════════════════════════════════
    fig = plt.figure(figsize=(11.69, 8.27))
    page_header(fig, 'Codebase Architecture', 'Package structure, class hierarchy, and module dependencies')
    footer(fig, 1)

    ax = fig.add_axes([0.04, 0.05, 0.92, 0.84])
    ax.set_xlim(0, 20); ax.set_ylim(0, 14)
    ax.axis('off')

    section_bg(ax, 0.1, 0.2, 6.5, 13.5, BG_GRAY, 'Project Structure (~1400 LOC)')

    tree = [
        (0.4, 12.6, 'amiga_project/', 11, NAVY),
        (0.8, 12.0, 'src/', 10, DGRAY),
        (1.2, 11.4, 'amiga_ros2_bridge/', 10, BLUE),
        (1.8, 10.8, 'amiga_bridge_node.py  (348L)', 9, DGRAY),
        (1.8, 10.35,'streams_node.py  (326L) [legacy]', 8, LGRAY),
        (1.8, 9.95, 'cmd_vel_node.py  (90L)  [legacy]', 8, LGRAY),
        (1.6, 9.4,  'config/', 9, DGRAY),
        (2.0, 9.0,  'amiga_bridge.yaml', 8, DGRAY),
        (2.0, 8.65, 'amiga_bridge.rviz', 8, DGRAY),
        (1.6, 8.15, 'launch/', 9, DGRAY),
        (2.0, 7.8,  'amiga_bridge.launch.py', 8, DGRAY),
        (1.2, 7.2,  'farnav/', 10, GREEN),
        (1.8, 6.6,  'global_planner.py  (265L)', 9, DGRAY),
        (1.8, 6.2,  'controller_mpc.py  [planned]', 8, LGRAY),
        (1.8, 5.8,  'path_utils.py  [planned]', 8, LGRAY),
        (1.6, 5.25, 'config/', 9, DGRAY),
        (2.0, 4.9,  'row_data.yaml', 8, DGRAY),
        (0.8, 4.3,  'docs/', 10, PURPLE),
        (1.2, 3.8,  'plans/', 9, DGRAY),
        (1.6, 3.4,  'controller-mpc-plan.md', 8, DGRAY),
        (1.6, 3.05, 'controller-mpc-design.md', 8, DGRAY),
        (1.2, 2.55, 'architecture.mmd', 8, DGRAY),
        (1.2, 2.2,  'global_planner.mmd', 8, DGRAY),
        (0.8, 1.6,  'test_*.py  (debug scripts)', 9, ORANGE),
        (1.2, 1.15, 'test_canbus / test_cmd / test_oak ...', 8, MGRAY),
    ]
    for (x, y, txt, fs, col) in tree:
        ax.text(x, y, txt, fontsize=fs, color=col, fontweight='bold' if fs >= 10 else 'normal', fontfamily='monospace')
    for (x, y, txt, fs, col) in tree:
        if x > 0.4:
            ax.plot([x - 0.25, x - 0.05], [y + 0.08, y + 0.08], color=VLGRAY, linewidth=1)

    section_bg(ax, 7.0, 7.0, 12.7, 6.7, BG_BLUE, 'Class Hierarchy (rclpy.Node)')
    box(ax, 9.5, 12.5, 3.0, 0.7, NAVY, 'rclpy.Node\n(ROS2 Base)', fontsize=9)
    box(ax, 7.3, 11.0, 3.5, 0.85, BLUE, 'AmigaBridgeNode\namiga_bridge_node.py', fontsize=9)
    box(ax, 11.2, 11.0, 3.2, 0.85, LGRAY, 'AmigaStreamsNode\n[legacy]', fontsize=8)
    box(ax, 14.8, 11.0, 3.2, 0.85, LGRAY, 'AmigaCmdVelNode\n[legacy]', fontsize=8)
    box(ax, 7.3, 9.5, 3.5, 0.85, GREEN, 'GlobalPlanner\nglobal_planner.py', fontsize=9)
    arrow(ax, 11.0, 12.5, 9.0, 11.85, DGRAY, '->', 1.5)
    arrow(ax, 11.0, 12.5, 12.8, 11.85, LGRAY, '->', 1)
    arrow(ax, 11.0, 12.5, 16.4, 11.85, LGRAY, '->', 1)
    arrow(ax, 11.0, 12.5, 9.0, 10.35, DGRAY, '->', 1.5)

    box(ax, 7.3, 7.5, 5.5, 1.2, BG_BLUE,
        'Publishers: /amiga/twist, /odom, /gps, /imu,\n/oak/left, /oak/rgb, /battery, /control_state, /tf\nSubscribers: /cmd_vel, /amiga/cmd_vel',
        fontsize=8, text_color=DGRAY)
    box(ax, 13.2, 7.5, 5.5, 1.2, BG_GREEN,
        'Publishers: /farnav/global_path,\n/farnav/path_info, /farnav/active_row_marker\nService: /farnav/set_active_row',
        fontsize=8, text_color=DGRAY)

    section_bg(ax, 7.0, 0.2, 12.7, 6.5, BG_ORANGE, 'Key Dependencies')
    deps = [
        (7.4, 5.3, 'farm-ng-core', BLUE, 'EventClient\ngRPC transport'),
        (10.6, 5.3, 'farm-ng-amiga', BLUE, 'Protobuf messages\nAmigaV6CanbusState'),
        (13.8, 5.3, 'rclpy', GREEN, 'ROS2 Python\nclient library'),
        (17.0, 5.3, 'tf2_ros', GREEN, 'TF broadcaster\nodom→base_link'),
        (7.4, 3.5, 'numpy', PURPLE, 'Array ops'),
        (10.0, 3.5, 'scipy', PURPLE, 'CubicSpline\ninterpolation'),
        (12.6, 3.5, 'pyproj', PURPLE, 'WGS84→UTM\ncoord transform'),
        (15.5, 3.5, 'asyncio', ORANGE, 'Async gRPC\nevent loop'),
    ]
    for (x, y, name, col, desc) in deps:
        box(ax, x, y, 2.3, 0.9, col, f'{name}\n{desc}', fontsize=7.5)

    ax.text(7.4, 2.2, 'Entry Points:', fontsize=9, fontweight='bold', color=DGRAY)
    ax.text(7.4, 1.6, 'ros2 run amiga_ros2_bridge amiga_bridge  → AmigaBridgeNode', fontsize=8, color=DGRAY, fontfamily='monospace')
    ax.text(7.4, 1.15,'ros2 run farnav global_planner            → GlobalPlanner', fontsize=8, color=DGRAY, fontfamily='monospace')

    pdf.savefig(fig); plt.close()

    # ═══════════════════════════════════════════════════════
    # PAGE 2 — ROS2 Bridge Design  (FIXED: no overlapping text)
    # ═══════════════════════════════════════════════════════
    fig = plt.figure(figsize=(11.69, 8.27))
    page_header(fig, 'ROS2 Bridge Design', 'AmigaBridgeNode — bidirectional gRPC ↔ ROS2 bridge (amiga_bridge_node.py)')
    footer(fig, 2)

    ax = fig.add_axes([0.03, 0.05, 0.94, 0.84])
    ax.set_xlim(0, 24); ax.set_ylim(0, 16)
    ax.axis('off')

    # ── Left column: Amiga gRPC Services ──
    section_bg(ax, 0.1, 0.2, 5.5, 15.5, BG_RED, 'Amiga Robot (172.16.107.128)')

    svc = [
        (0.4, 13.0, 'Canbus :6001',  RED,  '/state → CanbusState\n/twist ← Twist2d'),
        (0.4, 10.6, 'Filter :20001', RED,  '/state → pose + vel'),
        (0.4, 8.6,  'GPS :50010',    RED,  '/pvt → lat, lon, alt'),
        (0.4, 6.0,  'OAK/0 :50010',  RED,  '/left → JPEG\n/rgb  → JPEG\n/imu  → accel+gyro'),
    ]
    for (x, y, name, col, detail) in svc:
        box(ax, x, y, 2.8, 1.2, col, name, fontsize=9)
        ax.text(x + 0.1, y - 0.3, detail, fontsize=7.5, color=DGRAY, fontfamily='monospace', va='top')

    # ── Center column: Bridge internals ──
    section_bg(ax, 6.2, 0.2, 7.6, 15.5, BG_BLUE, 'AmigaBridgeNode')

    # Threading — shorter text to avoid cutoff
    box(ax, 6.5, 14.0, 7.0, 1.2, NAVY, 'Threading Model\nMain: ROS2 Executor  |  Background: asyncio (gRPC)', fontsize=8)

    # Streaming tasks
    box(ax, 6.5, 11.8, 7.0, 1.8, BLUE, '', fontsize=8)
    ax.text(10.0, 13.25, 'Async Streaming Tasks (Amiga → ROS2)', fontsize=9, ha='center', color='white', fontweight='bold')
    tasks = ['_stream_canbus()', '_stream_filter()', '_stream_gps()',
             '_stream_oak_left()', '_stream_oak_rgb()', '_stream_oak_imu()']
    for i, t in enumerate(tasks):
        col_i = i % 3
        row_i = i // 3
        ax.text(6.8 + col_i * 2.2, 12.7 - row_i * 0.45, t, fontsize=7, color='white', fontfamily='monospace')

    # Subscribe pattern
    box(ax, 6.5, 10.3, 7.0, 1.1, BG_BLUE,
        'Pattern: EventClient.subscribe(SubscribeRequest(uri, every_n))\nasync for event, msg in stream: → publish to ROS2',
        fontsize=7.5, text_color=DGRAY)

    # Command path
    box(ax, 6.5, 7.5, 7.0, 2.4, TEAL, '', fontsize=8)
    ax.text(10.0, 9.55, 'Command Path (ROS2 → Amiga)', fontsize=9, ha='center', color='white', fontweight='bold')
    cmd_lines = [
        '/cmd_vel callback  (main thread)',
        '        ↓',
        'asyncio.run_coroutine_threadsafe(',
        '    _send_twist(lx, ly, az), loop)',
        '        ↓',
        'Twist2d protobuf → client.request_reply("/twist")',
    ]
    for i, line in enumerate(cmd_lines):
        ax.text(7.0, 9.0 - i * 0.38, line, fontsize=7, color='white', fontfamily='monospace')

    # Parameters
    box(ax, 6.5, 4.5, 7.0, 2.6, BG_BLUE, '', fontsize=8)
    ax.text(10.0, 6.8, 'ROS2 Parameters (amiga_bridge.yaml)', fontsize=9, ha='center', color=DGRAY, fontweight='bold')
    params = [
        'host: "172.16.107.128"',
        'canbus_port: 6001   filter_port: 20001',
        'oak_port: 50010     gps_port: 50010',
        'oak_service_name: "oak/0"',
        'enable_canbus/oak/gps/filter: true',
        'odom_frame: "odom"  base_frame: "base_link"',
    ]
    for i, p in enumerate(params):
        ax.text(7.0, 6.3 - i * 0.35, p, fontsize=7.5, color=DGRAY, fontfamily='monospace')

    # Decimation
    box(ax, 6.5, 2.5, 7.0, 1.6, BG_GRAY,
        'Decimation Control\ncanbus_every_n=1  oak_every_n=2\ngps_every_n=1  filter_every_n=1\n→ Controls publish rate from gRPC to ROS2',
        fontsize=7.5, text_color=DGRAY)

    # ── Right column: ROS2 Topics ──
    section_bg(ax, 14.4, 0.2, 9.3, 15.5, BG_GREEN, 'ROS2 Ecosystem')

    ax.text(14.8, 14.8, 'Published Topics (9)', fontsize=11, fontweight='bold', color=GREEN)
    topics_pub = [
        ('/amiga/twist',              'TwistStamped',   'Measured vel'),
        ('/amiga/odom',               'Odometry',       'Fused pose+vel'),
        ('/amiga/gps',                'NavSatFix',      'Lat/lon/alt'),
        ('/amiga/imu',                'Imu',            'Accel + gyro'),
        ('/amiga/oak/left/compressed','CompressedImage','Left camera'),
        ('/amiga/oak/rgb/compressed', 'CompressedImage','RGB camera'),
        ('/amiga/battery',            'Float32',        'Charge %'),
        ('/amiga/control_state',      'UInt32',         'State 0-6'),
        ('/tf',                       'TF',             'odom→base_link'),
    ]
    for i, (topic, msg_type, desc) in enumerate(topics_pub):
        y = 14.2 - i * 0.55
        ax.text(14.8, y, topic, fontsize=7.5, color=DGRAY, fontfamily='monospace', fontweight='bold')
        ax.text(19.5, y, msg_type, fontsize=7, color=GREEN, fontfamily='monospace')
        ax.text(22.0, y, desc, fontsize=7, color=MGRAY)

    ax.text(14.8, 8.8, 'Subscribed Topics (2)', fontsize=11, fontweight='bold', color=BLUE)
    topics_sub = [
        ('/cmd_vel',       'Twist',        '_cmd_vel_cb()'),
        ('/amiga/cmd_vel', 'TwistStamped', '_cmd_vel_stamped_cb()'),
    ]
    for i, (topic, msg_type, desc) in enumerate(topics_sub):
        y = 8.2 - i * 0.6
        ax.text(14.8, y, topic, fontsize=7.5, color=DGRAY, fontfamily='monospace', fontweight='bold')
        ax.text(19.5, y, msg_type, fontsize=7, color=BLUE, fontfamily='monospace')
        ax.text(21.5, y, '→ ' + desc, fontsize=7, color=MGRAY)

    ax.text(14.8, 6.3, 'Downstream Consumers', fontsize=11, fontweight='bold', color=PURPLE)
    consumers = [
        ('RViz2',                     'Visualization of odom, path, cams'),
        ('GlobalPlanner',             'Reads /amiga/gps for localization'),
        ('MPC Controller [planned]',  'Reads /amiga/odom → /cmd_vel'),
        ('Nav2 [future]',             'Full autonomous navigation'),
    ]
    for i, (name, desc) in enumerate(consumers):
        y = 5.6 - i * 0.65
        ax.text(14.8, y, f'● {name}', fontsize=8.5, color=PURPLE, fontweight='bold')
        ax.text(19.5, y, desc, fontsize=8, color=MGRAY)

    # Arrows: gRPC → Bridge
    for (x, y, name, col, detail) in svc:
        arrow(ax, 5.6, y + 0.6, 6.2, y + 0.6, DGRAY, '->', 1.5)
    # Bridge → Topics
    arrow(ax, 13.5, 12.5, 14.4, 12.5, GREEN, '->', 2)
    # Topics → Bridge (cmd_vel)
    arrow(ax, 14.4, 7.9, 13.5, 8.5, BLUE, '->', 2)

    pdf.savefig(fig); plt.close()

    # ═══════════════════════════════════════════════════════
    # PAGE 3 — Global Planner  (FIXED: no title overlap)
    # ═══════════════════════════════════════════════════════
    fig = plt.figure(figsize=(11.69, 8.27))
    page_header(fig, 'Global Planner Design', 'GlobalPlanner node — GPS waypoints → cubic spline path (global_planner.py)')
    footer(fig, 3)

    ax = fig.add_axes([0.03, 0.05, 0.94, 0.84])
    ax.set_xlim(0, 20); ax.set_ylim(0, 16)
    ax.axis('off')

    # ── Pipeline (top) ──
    section_bg(ax, 0.1, 10.0, 19.7, 5.7, BG_GRAY, 'Planning Pipeline')

    steps = [
        (0.5,  11.5, 2.8, 2.0, ORANGE, 'row_data.yaml\n\nrows:\n  row_1:\n    A: [lat, lon]\n    waypoints: [...]\n    B: [lat, lon]'),
        (3.8,  11.5, 2.5, 2.0, BLUE,   '_parse_rows()\n\nRead ROS2 params\nBuild row list\n[{name, A, wps, B}]'),
        (6.8,  11.5, 2.8, 2.0, TEAL,   '_latlon_to_utm()\n\npyproj Transformer\nWGS84 → UTM\nEPSG:32616'),
        (10.1, 11.5, 3.0, 2.0, GREEN,  '_build_path()\n\nCubicSpline(t, x)\nCubicSpline(t, y)\nclamped boundary'),
        (13.6, 11.5, 2.8, 2.0, PURPLE, 'Arc-Length θ\n\nCumulative dist\nalong spline\nθ in pose.z'),
        (16.9, 11.5, 2.5, 2.0, RED,    '_publish_path()\n\nPath msg →\n/farnav/global_path\n@ 1 Hz'),
    ]
    for (x, y, w, h, col, txt) in steps:
        box(ax, x, y, w, h, col, txt, fontsize=7.5)
    for i, (x, y, w, h, col, txt) in enumerate(steps):
        ax.text(x + w/2, y + h + 0.25, f'Step {i+1}', fontsize=8, ha='center', color=MGRAY, fontweight='bold')
    for i in range(len(steps)-1):
        arrow(ax, steps[i][0]+steps[i][2], 12.5, steps[i+1][0], 12.5, DGRAY, '->', 2)

    # ── Bottom left: Spline math ──
    section_bg(ax, 0.1, 0.2, 10.0, 9.3, BG_BLUE, 'Spline Interpolation Detail')

    math_text = (
        "Input: GPS points  P₀(A), P₁...Pₙ(waypoints), Pₙ₊₁(B)\n\n"
        "1. Chord-length parameterization:\n"
        "     t₀ = 0,  tₖ = tₖ₋₁ + ||Pₖ - Pₖ₋₁||\n"
        "     normalize t ∈ [0,1]\n\n"
        "2. Fit clamped cubic splines:\n"
        "     cs_x = CubicSpline(t, easting, bc='clamped')\n"
        "     cs_y = CubicSpline(t, northing, bc='clamped')\n\n"
        "3. Sample at spline_resolution=500:\n"
        "     t_fine = linspace(0, 1, 500)\n"
        "     X = cs_x(t_fine),  Y = cs_y(t_fine)\n\n"
        "4. Compute arc-length θ:\n"
        "     θₖ = Σ ||P(tₖ) - P(tₖ₋₁)||  (cumulative)\n\n"
        "5. Output PoseStamped[] where:\n"
        "     position.x = UTM easting\n"
        "     position.y = UTM northing\n"
        "     position.z = arc-length θ  (for MPC)"
    )
    ax.text(0.5, 9.0, math_text, fontsize=8, color=DGRAY, va='top', fontfamily='monospace', linespacing=1.4)

    # ── Bottom right top: Published Outputs ──
    section_bg(ax, 10.5, 5.8, 9.3, 3.7, BG_GREEN, 'Published Outputs')

    outputs = [
        ('/farnav/global_path',        'Path',   'PoseStamped[] (easting, northing, θ)'),
        ('/farnav/path_info',           'String', 'JSON: row_name, arc_length, num_pts'),
        ('/farnav/active_row_marker',   'Marker', 'LINE_STRIP for RViz (green)'),
    ]
    for i, (topic, msg, desc) in enumerate(outputs):
        y = 8.5 - i * 0.95
        box(ax, 10.8, y, 3.5, 0.7, GREEN, f'{topic}\n({msg})', fontsize=7)
        ax.text(14.6, y + 0.35, desc, fontsize=7.5, color=DGRAY, va='center')

    # ── Bottom right bottom: Service ──
    section_bg(ax, 10.5, 0.2, 9.3, 5.1, BG_PURPLE, 'Service & Row Management')

    ax.text(10.9, 4.7, '/farnav/set_active_row  (std_srvs/Trigger)', fontsize=9, color=PURPLE, fontweight='bold')
    service_desc = (
        "• Cycles rows round-robin: row_1 → row_2 → row_1\n"
        "• Triggers _build_path() to recompute spline\n"
        "• Immediately republishes new path\n"
        "• Returns success=True with new row name"
    )
    ax.text(10.9, 4.1, service_desc, fontsize=8, color=DGRAY, va='top', linespacing=1.6)

    ax.text(10.9, 1.7, 'Row format (row_data.yaml):', fontsize=8, color=DGRAY, fontweight='bold')
    ax.text(10.9, 1.2, 'row_1:\n  name: "Row 1"\n  A: [17.385, 78.486]\n  waypoints: [[...], [...]]\n  B: [17.385, 78.486]',
            fontsize=7.5, color=DGRAY, fontfamily='monospace', va='top', linespacing=1.3)

    pdf.savefig(fig); plt.close()

    # ═══════════════════════════════════════════════════════
    # PAGE 4 — MPC Controller  (FIXED: boxes moved up, no dead space)
    # ═══════════════════════════════════════════════════════
    fig = plt.figure(figsize=(11.69, 8.27))
    page_header(fig, 'MPC Controller Design', 'Planned controller_mpc.py — model-predictive control for row following')
    footer(fig, 4)

    ax = fig.add_axes([0.03, 0.05, 0.94, 0.84])
    ax.set_xlim(0, 20); ax.set_ylim(0, 16)
    ax.axis('off')

    # ── Control loop (top) ──
    section_bg(ax, 0.1, 11.5, 19.7, 4.2, BG_GRAY, 'MPC Control Loop @ 20 Hz')

    loop_boxes = [
        (0.5,  12.2, 2.8, 1.8, BLUE,   '/amiga/odom\n\nSubscribe to\nOdometry msg\n(x, y, ψ, v, ω)'),
        (3.8,  12.2, 2.8, 1.8, TEAL,   'Path Projection\n\npath_utils.project()\n→ closest θ\n→ (x_ref, y_ref, ψ, κ)'),
        (7.1,  12.2, 2.8, 1.8, PURPLE, 'Error Compute\n\nCross-track (e_ct)\nHeading (e_ψ)\nVelocity (e_v)'),
        (10.4, 12.2, 3.0, 1.8, GREEN,  'MPC Solver\n\nCasADi + IPOPT\nN=20, dt=0.05s\nHorizon = 1.0s'),
        (13.9, 12.2, 2.8, 1.8, ORANGE, 'Optimal Control\n\nv_cmd* (m/s)\nω_cmd* (rad/s)\n→ first step only'),
        (17.2, 12.2, 2.3, 1.8, RED,    '/amiga/\ncmd_vel\n\nTwistStamped\npublish'),
    ]
    for (x, y, w, h, col, txt) in loop_boxes:
        box(ax, x, y, w, h, col, txt, fontsize=7.5)
    for i in range(len(loop_boxes)-1):
        arrow(ax, loop_boxes[i][0]+loop_boxes[i][2], 13.1, loop_boxes[i+1][0], 13.1, DGRAY, '->', 2)

    # Feedback arrow (below boxes)
    ax.annotate('', xy=(0.5, 11.9), xytext=(19.5, 11.9),
                arrowprops=dict(arrowstyle='->', color=RED, lw=2, connectionstyle='arc3,rad=-0.12'))
    ax.text(10, 11.5, 'Feedback: robot executes cmd → new odom reading', fontsize=8, color=RED, ha='center')

    # ── Bottom left: Dynamics ──
    section_bg(ax, 0.1, 0.2, 9.5, 11.0, BG_BLUE, 'Unicycle Dynamics Model')

    dynamics = (
        "State:   ξ = [X, Y, ψ, v]ᵀ\n"
        "Control: u = [v_cmd, ω_cmd]ᵀ\n\n"
        "Discrete dynamics (Euler, dt=0.05s):\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "  X_{k+1} = X_k + dt·v_k·cos(ψ_k)\n"
        "  Y_{k+1} = Y_k + dt·v_k·sin(ψ_k)\n"
        "  ψ_{k+1} = ψ_k + dt·ω_cmd_k\n"
        "  v_{k+1} = v_k + dt·(v_cmd_k - v_k)/τ_v\n\n"
        "Where τ_v ≈ 0.3s (velocity time constant)\n\n"
        "Cost function (minimize over N=20):\n"
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n"
        "  J = Σ [ w_cte · e_ct²\n"
        "        + w_head · e_ψ²\n"
        "        + w_v · (v - v_ref)²\n"
        "        + w_dv · Δv_cmd²\n"
        "        + w_dω · Δω_cmd² ]"
    )
    ax.text(0.5, 10.7, dynamics, fontsize=8.5, color=DGRAY, va='top', fontfamily='monospace', linespacing=1.4)

    # ── Bottom right top: Constraints ──
    section_bg(ax, 10.0, 5.7, 9.7, 5.6, BG_RED, 'Constraints')

    constraints = (
        "Hard constraints (infeasible if violated):\n"
        "  |e_ct| ≤ row_width / 2   (stay in row)\n"
        "  0 ≤ v_cmd ≤ v_max        (no reverse)\n"
        "  |ω_cmd| ≤ ω_max          (steer limit)\n\n"
        "Soft constraints (penalized in cost):\n"
        "  |Δv_cmd| ≤ a_max · dt    (smooth accel)\n"
        "  |Δω_cmd| ≤ α_max · dt    (smooth steer)\n\n"
        "Robot limits (Amiga skid-steer):\n"
        "  v_max ≈ 1.0 m/s\n"
        "  ω_max ≈ 1.5 rad/s\n"
        "  track_width ≈ 0.87m\n"
        "  wheel_radius ≈ 0.16m"
    )
    ax.text(10.4, 10.8, constraints, fontsize=8.5, color=DGRAY, va='top', fontfamily='monospace', linespacing=1.4)

    # ── Bottom right bottom: path_utils ──
    section_bg(ax, 10.0, 0.2, 9.7, 5.0, BG_PURPLE, 'path_utils.py (helper library)')

    utils = (
        "project(path_x, path_y, robot_x, robot_y)\n"
        "  → θ_closest: arc-length of nearest\n"
        "    point on spline\n\n"
        "get_path_state(θ, cs_x, cs_y)\n"
        "  → (x_ref, y_ref, ψ_ref, κ)\n"
        "    reference pose + curvature\n\n"
        "compute_errors(robot_x/y/ψ, θ, path)\n"
        "  → (e_ct, e_ψ)\n"
        "    cross-track and heading errors"
    )
    ax.text(10.4, 4.7, utils, fontsize=8.5, color=DGRAY, va='top', fontfamily='monospace', linespacing=1.4)

    pdf.savefig(fig); plt.close()

    # ═══════════════════════════════════════════════════════
    # PAGE 5 — State Machine + Data Flow  (FIXED: no overlaps)
    # ═══════════════════════════════════════════════════════
    fig = plt.figure(figsize=(11.69, 8.27))
    page_header(fig, 'Control State Machine & Full Data Flow', 'Amiga 7-state control model + end-to-end message routing')
    footer(fig, 5)

    ax = fig.add_axes([0.03, 0.05, 0.94, 0.84])
    ax.set_xlim(0, 20); ax.set_ylim(0, 16)
    ax.axis('off')

    # ── State Machine (top half) ──
    section_bg(ax, 0.1, 8.5, 19.7, 7.2, BG_GRAY, 'Amiga Control State Machine (7 states)')

    # State boxes — sized bigger, more spacing
    bw, bh = 2.4, 1.2
    states = [
        (1.0,  12.5, 'BOOT (0)',           LGRAY),
        (4.5,  13.5, 'MANUAL_READY (1)',   MGRAY),
        (4.5,  10.5, 'MANUAL_ACTIVE (2)',  DGRAY),
        (9.0,  13.5, 'CC_ACTIVE (3)',      ORANGE),
        (13.5, 13.5, 'AUTO_READY (4)',     TEAL),
        (13.5, 10.5, 'AUTO_ACTIVE (5)',    GREEN),
        (18.0, 12.0, 'ESTOPPED (6)',       RED),
    ]
    for (x, y, name, col) in states:
        box(ax, x - bw/2, y - bh/2, bw, bh, col, name, fontsize=8.5)

    # Transitions with labels that don't overlap
    trans = [
        (2.2, 12.5, 3.3, 13.5, 'boot'),
        (5.7, 13.5, 7.8, 13.5, 'pendant sw'),
        (10.2,13.5, 12.3,13.5, 'pendant sw'),
        (4.5, 12.9, 4.5, 11.1, 'joystick'),
        (4.5, 11.1, 4.5, 12.9, 'released'),
        (13.5,12.9, 13.5,11.1, '/twist cmd'),
        (14.7,10.5, 16.8,12.0, 'e-stop'),
        (12.3,10.5, 5.7, 10.5, 'pendant→manual'),
    ]
    for (x1, y1, x2, y2, lbl) in trans:
        arrow(ax, x1, y1, x2, y2, DGRAY, '->', 1.5)
        mx, my = (x1+x2)/2, (y1+y2)/2
        dy = 0.35 if y1 == y2 else 0
        dx = 0.6 if x1 == x2 else 0
        ax.text(mx + dx, my + dy, lbl, fontsize=7, color=MGRAY, ha='center',
                bbox=dict(boxstyle='round,pad=0.12', facecolor='white', edgecolor=VLGRAY, alpha=0.95))

    # Key callout
    box(ax, 3.0, 9.0, 14.0, 0.5, BG_BLUE,
        'KEY: Pendant → AUTO_READY (4), then /twist command → AUTO_ACTIVE (5). Timeout/E-stop → ESTOPPED (6)',
        fontsize=8, text_color=BLUE)

    # ── Data Flow (bottom half) ──
    section_bg(ax, 0.1, 0.2, 19.7, 8.0, BG_BLUE, 'End-to-End Data Flow')

    # Hardware row
    ax.text(0.4, 7.7, 'Hardware', fontsize=9, color=RED, fontweight='bold')
    hw_items = ['Motors &\nEncoders', 'IMU +\nWheels', 'GNSS\nReceiver', 'OAK-D\nCameras ×4', 'Pendant']
    for i, txt in enumerate(hw_items):
        box(ax, 0.4 + i * 2.4, 6.5, 2.0, 0.9, RED, txt, fontsize=7.5)

    # gRPC row
    ax.text(0.4, 5.8, 'gRPC Services', fontsize=9, color=ORANGE, fontweight='bold')
    grpc_items = ['Canbus\n:6001', 'Filter\n:20001', 'GPS\n:50010', 'OAK/0\n:50010']
    for i, txt in enumerate(grpc_items):
        box(ax, 0.4 + i * 2.4, 4.6, 2.0, 0.9, ORANGE, txt, fontsize=7.5)
    # hw → grpc arrows
    for i in range(4):
        arrow(ax, 1.4 + i*2.4, 6.5, 1.4 + i*2.4, 5.5, DGRAY, '->', 1)

    # Bridge row
    ax.text(0.4, 3.9, 'Bridge', fontsize=9, color=BLUE, fontweight='bold')
    box(ax, 0.4, 2.8, 11.0, 0.9, BLUE, 'AmigaBridgeNode  (gRPC EventClient ↔ ROS2 Publishers/Subscribers)', fontsize=9)
    for i in range(4):
        arrow(ax, 1.4 + i*2.4, 4.6, 5.9, 3.7, DGRAY, '->', 1)

    # ROS2 Topics row
    ax.text(0.4, 2.1, 'ROS2 Topics', fontsize=9, color=TEAL, fontweight='bold')
    topic_groups = [
        '/amiga/twist\n/battery\n/control_state',
        '/amiga/odom\n/tf',
        '/amiga/gps',
        '/amiga/oak/*\n/amiga/imu',
    ]
    for i, txt in enumerate(topic_groups):
        box(ax, 0.4 + i * 2.8, 0.5, 2.4, 1.2, TEAL, txt, fontsize=7)
    for i in range(4):
        arrow(ax, 1.6 + i*2.8, 2.8, 1.6 + i*2.8, 1.7, DGRAY, '->', 1)

    # Right side: consumers
    consumer_data = [
        (12.5, 6.5, 'GlobalPlanner',          GREEN),
        (12.5, 4.3, 'MPC Controller\n[planned]', PURPLE),
        (12.5, 2.0, 'Nav2 [future]',          LGRAY),
    ]
    output_data = [
        (16.0, 6.5, '/farnav/\nglobal_path',  GREEN),
        (16.0, 4.3, '/amiga/\ncmd_vel',       PURPLE),
        (16.0, 2.0, '/cmd_vel',               LGRAY),
    ]
    desc_data = [
        (18.5, 6.9, 'GPS → cubic\nspline path'),
        (18.5, 4.7, 'odom+path →\noptimal v, ω'),
        (18.5, 2.4, 'Full autonomy'),
    ]
    for (x, y, txt, col) in consumer_data:
        box(ax, x, y, 2.8, 0.9, col, txt, fontsize=8)
    for (x, y, txt, col) in output_data:
        box(ax, x, y, 2.0, 0.9, col, txt, fontsize=7.5)
    for (x, y, txt) in desc_data:
        ax.text(x, y, txt, fontsize=7.5, color=MGRAY, va='center')
    # consumer → output arrows
    for i in range(3):
        arrow(ax, 15.3, consumer_data[i][1]+0.45, 16.0, output_data[i][1]+0.45, DGRAY, '->', 1.5)
    # cmd_vel back to bridge
    arrow(ax, 16.0, 4.3, 11.4, 3.25, PURPLE, '->', 1.5)

    pdf.savefig(fig); plt.close()

    # ═══════════════════════════════════════════════════════
    # PAGE 6 — Future Roadmap  (FIXED: legend not cut off)
    # ═══════════════════════════════════════════════════════
    fig = plt.figure(figsize=(11.69, 8.27))
    page_header(fig, 'Future Architecture & Roadmap', 'Planned extensions: MPC controller, Nav2 integration, perception pipeline')
    footer(fig, 6)

    ax = fig.add_axes([0.03, 0.05, 0.94, 0.84])
    ax.set_xlim(0, 20); ax.set_ylim(0, 16)
    ax.axis('off')

    # ── Current (left) ──
    section_bg(ax, 0.1, 9.5, 9.3, 6.2, BG_GREEN, 'CURRENT (Implemented)')

    current_items = [
        (0.4, 14.0, 4.0, 0.75, GREEN, 'AmigaBridgeNode ✓\nBidirectional gRPC ↔ ROS2'),
        (0.4, 12.8, 4.0, 0.75, GREEN, 'GlobalPlanner ✓\nGPS → CubicSpline path'),
        (0.4, 11.6, 4.0, 0.75, GREEN, 'Config System ✓\namiga_bridge.yaml + row_data.yaml'),
        (4.8, 14.0, 4.0, 0.75, GREEN, '9 ROS2 Topics ✓\nSensor streams + TF'),
        (4.8, 12.8, 4.0, 0.75, GREEN, 'Launch System ✓\namiga_bridge.launch.py'),
        (4.8, 11.6, 4.0, 0.75, GREEN, 'RViz Visualization ✓\nodom + path + cameras'),
    ]
    for (x, y, w, h, col, txt) in current_items:
        box(ax, x, y, w, h, col, txt, fontsize=7.5)

    ax.text(0.4, 10.5, '~1400 LOC Python  |  2 packages  |  2 active nodes', fontsize=8, color=GREEN, fontweight='bold')

    # ── Future (right) ──
    section_bg(ax, 10.0, 9.5, 9.7, 6.2, BG_ORANGE, 'PLANNED / FUTURE')

    future_items = [
        (10.3, 14.0, 4.3, 0.75, ORANGE, 'MPC Controller (Phase 1)\ncontroller_mpc.py + path_utils.py'),
        (10.3, 12.8, 4.3, 0.75, ORANGE, 'Headland Turns (Phase 2)\nU-turn at row ends'),
        (10.3, 11.6, 4.3, 0.75, ORANGE, 'Multi-Row Sequencing (Phase 3)\nFull field coverage planner'),
        (15.0, 14.0, 4.3, 0.75, RED,    'Nav2 Integration\nFull autonomous navigation'),
        (15.0, 12.8, 4.3, 0.75, RED,    'Perception Pipeline\nOAK stereo → obstacles'),
        (15.0, 11.6, 4.3, 0.75, RED,    'Fleet Management\nMulti-robot coordination'),
    ]
    for (x, y, w, h, col, txt) in future_items:
        box(ax, x, y, w, h, col, txt, fontsize=7.5)

    ax.text(10.3, 10.5, 'Orange = designed  |  Red = conceptual', fontsize=8, color=ORANGE, fontweight='bold')

    # ── Target Architecture (bottom) ──
    section_bg(ax, 0.1, 0.2, 19.7, 8.8, BG_GRAY, 'Target Architecture (Full Autonomy Stack)')

    # Legend (inside section, not cut off)
    legend_items = [
        (12.0, 8.5, GREEN,  'Implemented'),
        (14.0, 8.5, PURPLE, 'Designed'),
        (15.8, 8.5, TEAL,   'Partial'),
        (17.3, 8.5, LGRAY,  'Future'),
    ]
    for (x, y, col, txt) in legend_items:
        r = FancyBboxPatch((x, y), 0.3, 0.3, boxstyle='round,pad=0.02', facecolor=col, edgecolor='none')
        ax.add_patch(r)
        ax.text(x + 0.45, y + 0.15, txt, fontsize=8, color=DGRAY, va='center')

    # Layer boxes
    box(ax, 0.5, 6.8, 18.8, 1.0, RED,
        'Hardware:   Motors  |  OAK-D ×4  |  GNSS  |  IMU  |  Pendant  |  CAN bus', fontsize=9)
    box(ax, 0.5, 5.3, 18.8, 1.0, ORANGE,
        'Services:   Canbus :6001  |  Filter :20001  |  OAK :50010  |  GPS :50010  |  Track Follower :20101', fontsize=9)
    box(ax, 0.5, 3.8, 18.8, 1.0, BLUE,
        'Bridge:   AmigaBridgeNode (gRPC ↔ ROS2)  |  TF tree  |  Diagnostics', fontsize=9)

    # ROS2 nodes at bottom
    ax.text(0.5, 3.3, 'ROS2 Nodes', fontsize=9, color=DGRAY, fontweight='bold')
    nodes = [
        (0.5,  1.0, 2.8, 1.6, GREEN,  'GlobalPlanner\n\n/farnav/global_path\nCubicSpline paths'),
        (3.7,  1.0, 2.8, 1.6, PURPLE, 'MPC Controller\n\nCasADi + IPOPT\n20 Hz control loop'),
        (6.9,  1.0, 2.8, 1.6, TEAL,   'Perception\n\nStereo depth\nObstacle detect'),
        (10.1, 1.0, 2.8, 1.6, DARK_BG,'Nav2 Stack\n\nCostmap\nRecovery behaviors'),
        (13.3, 1.0, 2.8, 1.6, LGRAY,  'Coverage\nPlanner\n\nField-level\nrow sequencing'),
        (16.5, 1.0, 2.8, 1.6, LGRAY,  'Fleet\nManager\n\nMulti-robot\ncoordination'),
    ]
    for (x, y, w, h, col, txt) in nodes:
        box(ax, x, y, w, h, col, txt, fontsize=7.5)

    # Bridge → nodes arrows
    for (x, y, w, h, col, txt) in nodes:
        arrow(ax, x + w/2, 3.8, x + w/2, y + h, DGRAY, '->', 1)

    # Horizontal flow arrows between nodes
    for i in range(len(nodes)-1):
        x1 = nodes[i][0] + nodes[i][2]
        x2 = nodes[i+1][0]
        arrow(ax, x1, 1.8, x2, 1.8, DGRAY, '->', 1.2)

    pdf.savefig(fig); plt.close()

print(f"Saved → {pdf_path}")
