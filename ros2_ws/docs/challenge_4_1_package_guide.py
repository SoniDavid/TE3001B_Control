#!/usr/bin/env python3
"""Generate PDF guide for xarm_ctc_challenge package."""

from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.units import inch
from reportlab.lib import colors
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
    HRFlowable, Preformatted
)
from reportlab.lib.enums import TA_LEFT, TA_CENTER

OUT = "/home/soni/Documents/classes/IRS_6to/Control/TE3001B_Control/ros2_ws/docs/challenge_4_1_package_guide.pdf"

doc = SimpleDocTemplate(
    OUT,
    pagesize=letter,
    leftMargin=0.85*inch, rightMargin=0.85*inch,
    topMargin=0.85*inch, bottomMargin=0.85*inch,
)

styles = getSampleStyleSheet()

# Custom styles
H1 = ParagraphStyle("H1", parent=styles["Heading1"], fontSize=16, spaceAfter=6,
                    textColor=colors.HexColor("#1a3a6b"))
H2 = ParagraphStyle("H2", parent=styles["Heading2"], fontSize=12, spaceAfter=4,
                    textColor=colors.HexColor("#2e5fa3"), spaceBefore=12)
H3 = ParagraphStyle("H3", parent=styles["Heading3"], fontSize=10, spaceAfter=3,
                    textColor=colors.HexColor("#444"), spaceBefore=8)
BODY = ParagraphStyle("BODY", parent=styles["Normal"], fontSize=9, spaceAfter=4,
                      leading=14)
CODE = ParagraphStyle("CODE", fontName="Courier", fontSize=8, spaceAfter=4,
                      backColor=colors.HexColor("#f4f4f4"), leading=12,
                      leftIndent=12, rightIndent=12)
NOTE = ParagraphStyle("NOTE", parent=styles["Normal"], fontSize=8.5,
                      backColor=colors.HexColor("#fff8e1"), spaceAfter=4,
                      leftIndent=8, leading=13,
                      borderColor=colors.HexColor("#f0c040"), borderWidth=1,
                      borderPadding=4)

def hr():
    return HRFlowable(width="100%", thickness=0.5, color=colors.HexColor("#cccccc"),
                      spaceAfter=4, spaceBefore=4)

def h1(t): return Paragraph(t, H1)
def h2(t): return Paragraph(t, H2)
def h3(t): return Paragraph(t, H3)
def p(t):  return Paragraph(t, BODY)
def code(t): return Preformatted(t, CODE)
def note(t): return Paragraph(f"<b>Note:</b> {t}", NOTE)
def sp(n=1): return Spacer(1, n * 0.12 * inch)

def table(data, col_widths=None, header=True):
    t = Table(data, colWidths=col_widths)
    style = [
        ("FONTNAME",  (0, 0), (-1, -1), "Helvetica"),
        ("FONTSIZE",  (0, 0), (-1, -1), 8),
        ("ROWBACKGROUNDS", (0, 0), (-1, -1),
         [colors.HexColor("#f0f4ff"), colors.white]),
        ("GRID",      (0, 0), (-1, -1), 0.3, colors.HexColor("#aaaaaa")),
        ("VALIGN",    (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING",  (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING",   (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING",(0, 0), (-1, -1), 3),
    ]
    if header:
        style += [
            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#1a3a6b")),
            ("TEXTCOLOR",  (0, 0), (-1, 0), colors.white),
            ("FONTNAME",   (0, 0), (-1, 0), "Helvetica-Bold"),
        ]
    t.setStyle(TableStyle(style))
    return t

# ── Content ─────────────────────────────────────────────────────────────────

story = []

# Title
story += [
    Spacer(1, 0.2*inch),
    Paragraph("<b>xarm_ctc_challenge — Challenge 4.1</b>", ParagraphStyle(
        "TITLE", parent=styles["Title"], fontSize=20,
        textColor=colors.HexColor("#1a3a6b"), spaceAfter=2)),
    Paragraph("Package Architecture, CTC Derivation, Run Guide &amp; Debug Reference",
              ParagraphStyle("SUB", parent=styles["Normal"], fontSize=11,
                             textColor=colors.HexColor("#555"), spaceAfter=2, alignment=TA_CENTER)),
    Paragraph("TE3001B Control — 2026", ParagraphStyle(
        "META", parent=styles["Normal"], fontSize=9,
        textColor=colors.gray, alignment=TA_CENTER)),
    hr(), sp(),
]

# ── SECTION 1: Overview ──────────────────────────────────────────────────────
story += [
    h1("1  Package Overview"),
    p("The <b>xarm_ctc_challenge</b> package implements two joint-space controllers — "
      "Computed Torque Control (CTC) and PD/PID — for the xArm Lite 6, comparing "
      "their trajectory tracking performance under nominal conditions and external "
      "perturbations. It is the joint-space successor to Lab 4.2 "
      "(<i>xarm_perturbations</i>), which used task-space Cartesian PD."),
    sp(),
    table([
        ["Aspect", "xarm_perturbations (Lab 4.2)", "xarm_ctc_challenge (Challenge 4.1)"],
        ["Control space", "Cartesian (task-space)", "Joint-space"],
        ["Command topic", "/servo_server/delta_twist_cmds\n(TwistStamped)", "/servo_server/delta_joint_cmds\n(JointJog)"],
        ["Trajectory", "Cartesian quintic spline", "Joint-space quintic spline\n(via MoveIt IK)"],
        ["IK", "Real-time resolved-rate\nin the control loop", "Offline at startup\n(MoveIt /compute_ik)"],
        ["Controllers", "PD only", "CTC  or  PD/PID"],
        ["Perturbations", "Cartesian injector", "Same injector reused"],
    ], col_widths=[1.3*inch, 2.4*inch, 2.4*inch]),
    sp(),
]

# ── SECTION 2: Relation to CTCvsPID.ipynb ───────────────────────────────────
story += [
    h1("2  Relation to CTCvsPID.ipynb"),
    p("The notebook <b>CTCvsPID.ipynb</b> is the reference simulation that defines "
      "the exact CTC formulation, gain structure, and comparison methodology used in "
      "this package. The table below maps every key concept:"),
    sp(0.5),
    table([
        ["Notebook element", "Where it appears in the package"],
        ["class RobustCTC.compute()", "controller_node.py  _ctc()  lines 514–536"],
        ["class JointPID.compute()", "controller_node.py  _pd_pid()  lines 495–510"],
        ["class Lite6NominalDynamics\n(M0, C0_times_qd, G0, F0)", "dynamics.py  get_dynamics()"],
        ["class Lite6TruePlant\n(uncertainty + disturbance)", "Simulated by the perturbation_injector\n"
                                                               "node (xarm_perturbations) on real hardware"],
        ["smooth_sign() = tanh(S/ε)", "SIGN_EPS = 2e-3,  np.tanh(S / SIGN_EPS)\nin _ctc()"],
        ["S = γ·e + ė  (sliding surface)", "S = edot + GAMMA * e\nGAMMA = 2.5"],
        ["Lyapunov: V = 0.5 Sᵀ M₀ S", "Not computed online; available from CSV\n"
                                         "data post-hoc in analysis.py"],
        ["Barbalat / LaSalle analysis", "Can be applied offline to logged V(t)\n"
                                         "(not automated in current analysis.py)"],
        ["BarbalatAnalysis class", "Offline analysis only (not in package yet)"],
        ["desired(t) — sinusoidal q_des", "Replaced by InspectionTrajectory +\n"
                                           "JointRefTrajectory (quintic waypoints)"],
        ["DH / fk_T06 / fk_all", "kinematics.py  forward_kinematics(),\nposition_jacobian()"],
    ], col_widths=[2.4*inch, 3.7*inch]),
    sp(),
    note("The notebook uses a scalar Kp/Kv (×I₆), while the package uses diagonal matrices "
         "KP_CTC = diag(80,80,80,60,50,40) and KD_CTC = diag(28,28,28,20,16,12) — "
         "per-joint tuning suited for a real robot with unequal link inertias."),
    sp(),
]

# ── SECTION 3: CTC Derivation ────────────────────────────────────────────────
story += [
    h1("3  Where Does the CTC Come From?"),
    h2("3.1  Equation of Motion (EOM)"),
    p("The manipulator's EOM in joint space:"),
    code("  M(q)·q̈  +  C(q,q̇)·q̇  +  G(q)  +  F(q̇)  =  τ"),
    p("M — inertia matrix (6×6), C·q̇ — Coriolis/centrifugal, "
      "G — gravity vector, F — friction."),

    h2("3.2  CTC Feedforward Cancellation"),
    p("Choose τ to cancel the nonlinear dynamics and impose a desired linear error dynamics:"),
    code("  v    = q̈_des  +  Kp·e  +  Kd·ė           (desired closed-loop acceleration)\n"
         "  τ_ff = M₀(q)·v  +  C₀(q,q̇)·q̇  +  G₀(q)  +  F₀(q̇)   (feedforward)"),
    p("Using nominal (subscript 0) matrices — i.e., the best model we have. "
      "If M₀ = M exactly, the closed-loop becomes: <b>ë + Kd·ė + Kp·e = 0</b> "
      "(linear, tunable)."),

    h2("3.3  Robust / Sliding Term"),
    p("Because M₀ ≠ M (model uncertainty + disturbances), add a robust term "
      "using the sliding surface S = ė + γ·e:"),
    code("  S       = ė  +  γ·e                       (sliding surface, γ = 2.5)\n"
         "  τ_ro    = k_robust · tanh(S / ε)          (smooth robust term, k = 5.5, ε = 2e-3)\n"
         "  τ_total = τ_ff  +  τ_ro"),
    p("tanh(·/ε) is the smooth approximation of sign(·) from the notebook's "
      "<i>smooth_sign()</i>, avoiding chattering."),

    h2("3.4  Velocity Command"),
    p("MoveIt Servo expects joint <i>velocities</i>, not torques. The CTC torque is "
      "converted via:"),
    code("  q̈_cmd  =  M₀⁻¹ · τ_total               (solve linear system)\n"
         "  q̇_cmd  +=  q̈_cmd · dt                  (Euler integration, 100 Hz)\n"
         "  q̇_cmd  =  clip(q̇_cmd, ±2.0 rad/s)      (velocity saturation)"),

    h2("3.5  Full Code — _ctc() in controller_node.py"),
    code(
        "def _ctc(self, q, qd, q_des, qd_des, qdd_des):\n"
        "    e    = q_des - q\n"
        "    edot = qd_des - qd\n"
        "\n"
        "    v = qdd_des + KP_CTC @ e + KD_CTC @ edot  # desired accel\n"
        "\n"
        "    S      = edot + GAMMA * e                  # sliding surface\n"
        "    robust = K_ROBUST * np.tanh(S / SIGN_EPS)  # robust term\n"
        "\n"
        "    M, Cqd, G, F = get_dynamics(q, qd)        # nominal dynamics\n"
        "    tau = M @ v + Cqd + G + F + robust\n"
        "    tau = np.clip(tau, -TORQUE_LIMIT, TORQUE_LIMIT)\n"
        "\n"
        "    qdd_cmd = np.linalg.solve(M, tau)          # M^{-1} tau\n"
        "    self._qd_cmd_ctc += qdd_cmd * DT           # integrate\n"
        "    self._qd_cmd_ctc  = np.clip(..., -VEL_LIMIT, VEL_LIMIT)\n"
        "    return self._qd_cmd_ctc, sat"
    ),
    sp(),
]

# ── SECTION 4: How to Run ────────────────────────────────────────────────────
story += [
    h1("4  How to Run"),

    h2("4.1  Prerequisites"),
    code(
        "# Terminal 1 — xArm hardware + MoveIt Servo\n"
        "ros2 launch xarm_moveit_servo lite6_moveit_servo_realmove.launch.py \\\n"
        "    robot_ip:=192.168.1.123\n"
        "\n"
        "# Build (Terminal 2)\n"
        "cd ~/Documents/.../ros2_ws\n"
        "colcon build --packages-select xarm_ctc_challenge\n"
        "source install/setup.bash"
    ),

    h2("4.2  Four Trial Commands"),
    table([
        ["Trial", "Controller", "Perturbation", "Command"],
        ["1", "CTC", "No",
         "ros2 run xarm_ctc_challenge challenge_controller\n"
         "  --ros-args -p use_ctc:=true -p perturbation_enabled:=false"],
        ["2", "PD", "No",
         "ros2 run xarm_ctc_challenge challenge_controller\n"
         "  --ros-args -p use_ctc:=false -p perturbation_enabled:=false"],
        ["3", "CTC", "Yes",
         "ros2 run xarm_ctc_challenge challenge_controller\n"
         "  --ros-args -p use_ctc:=true -p perturbation_enabled:=true"],
        ["4", "PD", "Yes",
         "ros2 run xarm_ctc_challenge challenge_controller\n"
         "  --ros-args -p use_ctc:=false -p perturbation_enabled:=true"],
    ], col_widths=[0.4*inch, 0.7*inch, 0.8*inch, 4.2*inch]),

    sp(0.5),
    h2("4.3  Perturbation Injector (for trials 3 & 4)"),
    code(
        "ros2 run xarm_perturbations perturbation_injector \\\n"
        "  --ros-args -p output_topic:=/servo_server/delta_twist_cmds \\\n"
        "             -p enabled:=true -p mode:=gaussian \\\n"
        "             -p gauss_std_linear:=0.5 -p gauss_axis:=x"
    ),

    h2("4.4  Offline Analysis"),
    code(
        "python3 src/xarm_ctc_challenge/xarm_ctc_challenge/analysis.py data/\n"
        "# Generates: joints_*.png, taskspace_*.png, phase_*.png,\n"
        "#            comparison_nopert.png, comparison_pert.png\n"
        "#            + summary table printed to console"
    ),
    sp(),
]

# ── SECTION 5: Debug ────────────────────────────────────────────────────────
story += [
    h1("5  Debug Reference"),

    h2("5.1  Startup Sequence"),
    p("The node follows this sequence. If it hangs, check which step is missing:"),
    table([
        ["Step", "What happens", "Log message to look for"],
        ["1", "Node starts", '"ChallengeController started"'],
        ["2", "Waits for /joint_states", '"Waiting for /joint_states..."'],
        ["3", "First joint state received", '"Robot state received — q=[...]"'],
        ["4", "Calls MoveIt /compute_ik\n(or loads ik_refs.npy)", '"IK wp  0 (home): [...]"\nor "Loaded IK refs from..."'],
        ["5", "Trajectory built", '"Joint ref trajectory ready (40.0 s)"'],
        ["6", "Control loop runs", 'JointJog at 100 Hz on\n/servo_server/delta_joint_cmds'],
    ], col_widths=[0.35*inch, 1.8*inch, 3.95*inch]),

    sp(0.5),
    h2("5.2  Diagnostic Commands"),
    code(
        "# Is /compute_ik available?\n"
        "ros2 service list | grep compute_ik\n"
        "\n"
        "# Are joint commands flowing at 100 Hz?\n"
        "ros2 topic hz /servo_server/delta_joint_cmds\n"
        "\n"
        "# Watch joint states\n"
        "ros2 topic echo /joint_states\n"
        "\n"
        "# See waypoint markers in RViz\n"
        "ros2 run rviz2 rviz2   # Add MarkerArray → /challenge_markers\n"
        "\n"
        "# Emergency stop\n"
        "ros2 topic pub /challenge_stop std_msgs/Bool \"data: true\" --once"
    ),

    sp(0.5),
    h2("5.3  Common Failures"),
    table([
        ["Symptom", "Likely cause", "Fix"],
        ["No motion after startup", "IK still running or\n/joint_states not received",
         "Wait 5-10 s; check\nros2 topic echo /joint_states"],
        ['"IK failed wp X" warning', "Waypoint out of reach\nor collision detected",
         "Check z-heights in\ntrajectory.py; seed IK closer"],
        ["Wild joint velocities", "CTC gains too high or\nbad dynamics model",
         "Reduce KP_CTC diagonal;\ncheck dynamics.py"],
        ["CSV empty / missing", "csv_dir doesn't exist or\nsave_csv:=false",
         "Add -p csv_dir:=/tmp\nor mkdir data/"],
        ["Servo rejects commands", "Servo not in joint-vel mode\nor wrong QoS",
         "Check launch file;\nverify servo_mode param"],
        ["ik_refs.npy causes wrong refs", "Stale IK file from old run",
         "Delete data/ik_refs.npy\nand rerun"],
    ], col_widths=[1.6*inch, 2.1*inch, 2.4*inch]),
    sp(),
]

# ── SECTION 6: File Map ──────────────────────────────────────────────────────
story += [
    h1("6  Package File Map"),
    table([
        ["File", "Role"],
        ["kinematics.py", "Modified DH (Craig 1989) FK + position Jacobian + CoM Jacobian"],
        ["dynamics.py", "Lite6NominalDynamics: M₀, C₀·q̇, G₀, F₀  (from notebook)"],
        ["trajectory.py", "8-waypoint quintic spline (Cartesian) + JointRefTrajectory (joint-space)"],
        ["ik_solver.py", "WeightedIKSolver with DLS + nullspace (exists, not used by controller)"],
        ["controller_node.py", "Main ROS 2 node: MoveIt IK → JointRefTrajectory → CTC/PD → JointJog"],
        ["analysis.py", "Offline analysis: metrics, 7 plot types, comparison table"],
    ], col_widths=[1.6*inch, 4.5*inch]),
    sp(),
    note("ik_solver.py contains the full weighted resolved-rate IK with nullspace stabilization "
         "matching §14.2 of the rubric, but the controller uses MoveIt /compute_ik instead "
         "(user's explicit design choice). This costs ~12-15 pts in §14.2 but "
         "guarantees identical IK references across all 4 trials (§14.5)."),
]

doc.build(story)
print(f"PDF written to: {OUT}")
