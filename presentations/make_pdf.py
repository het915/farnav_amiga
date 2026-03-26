#!/usr/bin/env python3
"""Generate a 1-page project summary PDF."""

from fpdf import FPDF

pdf = FPDF(orientation="L", unit="mm", format="A4")
pdf.set_auto_page_break(auto=False)
pdf.add_page()

# ── Background ──
pdf.set_fill_color(15, 23, 42)  # dark slate
pdf.rect(0, 0, 297, 210, "F")

# ── Title bar ──
pdf.set_fill_color(30, 64, 175)  # blue accent
pdf.rect(0, 0, 297, 28, "F")

pdf.set_font("Helvetica", "B", 22)
pdf.set_text_color(255, 255, 255)
pdf.set_xy(10, 5)
pdf.cell(200, 12, "Amiga Autonomous Navigation Project", ln=True)

pdf.set_font("Helvetica", "", 10)
pdf.set_xy(10, 17)
pdf.cell(200, 6, "ROS 2 Bridge  |  Global Planner  |  MPC Controller  |  Farm-ng Amiga Robot")

pdf.set_font("Helvetica", "I", 9)
pdf.set_xy(230, 17)
pdf.cell(60, 6, "March 2026")

# ── Helper for section boxes ──
def section_box(x, y, w, h, title, items, accent=(59, 130, 246)):
    # box bg
    pdf.set_fill_color(30, 41, 59)
    pdf.rect(x, y, w, h, "F")
    # accent strip
    pdf.set_fill_color(*accent)
    pdf.rect(x, y, 3, h, "F")
    # title
    pdf.set_font("Helvetica", "B", 10)
    pdf.set_text_color(*accent)
    pdf.set_xy(x + 6, y + 2)
    pdf.cell(w - 10, 5, title)
    # items
    pdf.set_font("Helvetica", "", 7.5)
    pdf.set_text_color(203, 213, 225)
    yy = y + 8
    for item in items:
        pdf.set_xy(x + 6, yy)
        pdf.multi_cell(w - 12, 3.5, f"* {item}", align="L")
        yy += 3.5 * (1 + item.count("\n"))


# ── Row 1: Three main sections ──
col_w = 88
gap = 5
start_y = 32

section_box(
    8, start_y, col_w, 52,
    "1. ROS 2 Bridge",
    [
        "gRPC-to-ROS 2 bridge for all Amiga sensors & actuators",
        "Streams: CAN bus (16 Hz), OAK cameras, GPS, IMU, odometry",
        "Publishes /amiga/twist, /amiga/odom, /amiga/gps, /tf, images",
        "Subscribes /cmd_vel for teleop & autonomous control",
        "Discovered correct service ports via systematic scan",
        "Fixed 6 critical config issues (endpoints, paths, DDS)",
        "RViz 2 pre-configured for full sensor visualization",
    ],
    accent=(59, 130, 246),
)

section_box(
    8 + col_w + gap, start_y, col_w, 52,
    "2. Global Path Planner",
    [
        "Converts GPS waypoints (lat/lon) to cubic spline in odom frame",
        "GPS/odom alignment via UTM coordinate transform",
        "Arc-length parameterized path for MPC reference",
        "Pure NumPy path_utils library (project, interpolate, errors)",
        "Configurable multi-row YAML with A-point, waypoints, B-point",
        "Publishes /farnav/global_path for controller consumption",
        "Full test suite: projection, errors, boundary cases -- all passing",
    ],
    accent=(16, 185, 129),
)

section_box(
    8 + 2 * (col_w + gap), start_y, col_w, 52,
    "3. MPC Controller",
    [
        "Model Predictive Control with N=20 horizon (2s lookahead)",
        "Unicycle dynamics + first-order velocity lag model",
        "CasADi/IPOPT NLP solver with warm-starting, 20 Hz loop",
        "Cost: cross-track, heading, velocity, smoothness weights",
        "Constraints: row width (0.45m), wheel speed, angular rate",
        "End-of-path deceleration & smooth stop logic",
        "Predicted trajectory visualization in RViz 2",
    ],
    accent=(245, 158, 11),
)

# ── Row 2: Controllers Tested + Results + Architecture ──
row2_y = start_y + 56

section_box(
    8, row2_y, 135, 48,
    "Controllers Tested & Results",
    [
        "MPC (primary): Best tracking -- converges from 0.3m offset in <5s, stays within row",
        "Closed-loop sim: cross-track error < 0.05m, heading error < 0.1 rad after convergence",
        "Row-width constraint validated: robot stays within 0.45m bounds even from 0.4m offset",
        "Smooth control outputs: rate-of-change penalties prevent jerky steering",
        "Velocity tracking: reaches 0.5 m/s reference with first-order lag model accuracy",
        "All unit + integration tests passing (path utils, solver build, closed-loop stability)",
    ],
    accent=(239, 68, 68),
)

section_box(
    8 + 135 + gap, row2_y, 146, 48,
    "System Architecture & Integration",
    [
        "4-terminal launch: Bridge -> Global Planner -> MPC Controller -> Enable + Monitor",
        "Robot: Farm-ng Amiga skid-steer | Host: lavender-latency (172.16.107.128)",
        "Ports: CAN bus=6001, Filter=20001, OAK+GPS=50010",
        "Control flow: Pendant AUTO_READY(4) -> /twist cmd -> AUTO_ACTIVE(5)",
        "Full docs: env setup, MPC design, implementation plan, testing summary",
        "Stack: ROS 2 Humble, Python 3.10, CasADi, SciPy, pyproj, farm-ng SDK",
    ],
    accent=(168, 85, 247),
)

# ── Bottom bar ──
pdf.set_fill_color(30, 64, 175)
pdf.rect(0, 200, 297, 10, "F")
pdf.set_font("Helvetica", "I", 8)
pdf.set_text_color(200, 220, 255)
pdf.set_xy(10, 201)
pdf.cell(277, 7, "Amiga Project  --  Autonomous Crop-Row Navigation  --  University of Illinois", align="C")

pdf.output("/home/het/lab/amiga_project/send_amey.pdf")
print("PDF saved to /home/het/lab/amiga_project/send_amey.pdf")
