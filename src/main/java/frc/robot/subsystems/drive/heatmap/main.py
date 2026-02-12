# app.py
# Interactive FRC robot cycle-time heatmap with Plotly Dash

import numpy as np
from heapq import heappush, heappop
from dataclasses import dataclass
from typing import List, Tuple, Optional
import os
import base64
import json
from datetime import datetime

import dash
from dash import dcc, html, Output, Input, State, ctx
import plotly.graph_objects as go

#Fetch values from Drive.getinstance().getCtreDrive().getState().Pose?

import ntcore

inst = ntcore.NetworkTableInstance.getDefault()
table = inst.getTable("SmartDashboard")
pose_topic = table.getDoubleArrayTopic("Field/Robot")
subscriber = pose_topic.subscribe([0.0, 0.0, 0.0]) # may debatably swap current x, y order

inst.startClient4("Python-Client") #Python-Monitor
inst.setServerTeam(1458)

print("Waiting for connection...")
#while not inst.isConnected():
 #   x = 0

print("Connected! Streaming Pose:")

try:
    for i in range(10): # limit infinite loop for testing?
        pose = subscriber.get()
        #print(pose)
        #print(pose[0], pose[1], pose[2]) # cleaner
except KeyboardInterrupt:
    pass
#============================================

def add_paper_background(fig: go.Figure, img_or_data_uri: str, opacity: float = 1.0):
    """
    Add a full-plot (paper-aligned) background image.
    This image does NOT move/scale with axes; it fills the plotting region.
    """
    if not img_or_data_uri:
        return
    fig.add_layout_image({
        "source": img_or_data_uri,
        "xref": "paper", "yref": "paper",
        "x": 0, "y": 1,
        "sizex": 1, "sizey": 1,
        "xanchor": "left", "yanchor": "top",
        "sizing": "stretch",
        "layer": "below",
        "opacity": opacity,
    })

# -----------------------------
# Robot Kinematics Constants
# -----------------------------
@dataclass
class RobotKinematics:
    max_velocity: float = 19.685      # feet per second (6 m/s converted to feet/s)
    acceleration: float = 13.1234     # feet per second^2 (4 m/s^2 converted to feet/s^2)
    deceleration: float = 13.1234     # feet per second^2 (4 m/s^2 converted to feet/s^2)
    turn_time: float = 0.2            # seconds per 90-degree turn (for path changes)
    
def distance_to_time(distance_feet: float, kinematics: RobotKinematics = RobotKinematics()) -> float:
    """
    Convert distance in feet to time in seconds using robot kinematics.
    Uses trapezoidal velocity profile: accelerate -> cruise -> decelerate
    """
    if distance_feet <= 0:
        return 0.0
    
    # Distance needed to reach max velocity
    accel_distance = (kinematics.max_velocity ** 2) / (2 * kinematics.acceleration)
    decel_distance = (kinematics.max_velocity ** 2) / (2 * kinematics.deceleration)
    
    # Total distance needed for full acceleration/deceleration
    min_distance_for_max_speed = accel_distance + decel_distance
    
    if distance_feet <= min_distance_for_max_speed:
        # Triangular profile - never reach max speed
        # v_max_achieved^2 = 2 * a * d_accel = 2 * a * (distance / 2) = a * distance
        # Assuming symmetric accel/decel for simplicity
        avg_accel = (kinematics.acceleration + kinematics.deceleration) / 2
        v_max_achieved = np.sqrt(avg_accel * distance_feet)
        time_accel = v_max_achieved / kinematics.acceleration
        time_decel = v_max_achieved / kinematics.deceleration
        return time_accel + time_decel
    else:
        # Trapezoidal profile - reach max speed
        time_accel = kinematics.max_velocity / kinematics.acceleration
        time_decel = kinematics.max_velocity / kinematics.deceleration
        cruise_distance = distance_feet - accel_distance - decel_distance
        time_cruise = cruise_distance / kinematics.max_velocity
        return time_accel + time_cruise + time_decel

# -----------------------------
# Grid model
# -----------------------------
@dataclass
class GridModel:
    w: int = 58  # 57 feet 6 7/8 inches ≈ 58 feet
    h: int = 29  # 26 feet 5 inches ≈ 27 feet
    robot: Tuple[int, int] = (13, 29)  # row, col (roughly center of field)
    blocked: List[Tuple[int, int]] = None

    def __post_init__(self):
        if self.blocked is None:
            # Example obstacles - you can modify these for actual field elements
            self.blocked = []

    def is_blocked(self, r: int, c: int) -> bool:
        return (r, c) in set(self.blocked)

    def toggle_blocked(self, r: int, c: int):
        if (r, c) == self.robot:
            return  # do not block the robot cell
        if (r, c) in self.blocked:
            self.blocked.remove((r, c))
        else:
            self.blocked.append((r, c))

    def set_robot(self, r: int, c: int):
        if (r, c) in self.blocked:
            # if user puts robot on blocked cell, un-block it first
            self.blocked.remove((r, c))
        self.robot = (r, c)

# -----------------------------
# Pathfinding (Dijkstra 4-neighbor)
# -----------------------------
def dijkstra_distances(w: int, h: int, start: Tuple[int, int], blocked: List[Tuple[int, int]]) -> np.ndarray:
    blocked_set = set(blocked)
    sr, sc = start
    dist = np.full((h, w), np.inf, dtype=float)
    dist[sr, sc] = 0.0
    pq = [(0.0, sr, sc)]
    while pq:
        d, r, c = heappop(pq)
        if d > dist[r, c]:
            continue
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and (nr, nc) not in blocked_set:
                nd = d + 1.0
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    heappush(pq, (nd, nr, nc))
    return dist

# -----------------------------
# Figure builder
# -----------------------------
def build_figure(model: GridModel, kinematics: RobotKinematics = None) -> go.Figure:
    if kinematics is None:
        kinematics = RobotKinematics()
    dist = dijkstra_distances(model.w, model.h, model.robot, model.blocked)
    time_matrix = np.zeros_like(dist, dtype=float)
    for r in range(model.h):
        for c in range(model.w):
            time_matrix[r, c] = distance_to_time(dist[r, c], kinematics) if not np.isinf(dist[r, c]) else np.nan

    viz = time_matrix.copy()
    for r, c in model.blocked:
        viz[r, c] = np.nan
    rr, cc = model.robot
    viz[rr, cc] = 0.0

    x_vals = list(range(model.w))
    y_vals = list(range(model.h))
    zmin = np.nanmin(viz)
    zmax = np.nanmax(viz)

    heat = go.Heatmap(
        z=viz,
        x=x_vals,
        y=y_vals,
        colorscale="Viridis",
        zmin=zmin,
        zmax=zmax,
        colorbar=dict(title="Time (s)", thickness=16, len=0.85, x=1.02),
        hovertemplate="r %{y} • c %{x}<br>%{z:.2f}s<extra></extra>",
        opacity=0.58,          # transparency so field underneath is visible
        showscale=True,
    )

    # Blocked cells (black) drawn above the heat layer
    mask = np.full_like(viz, np.nan, dtype=float)
    for r, c in model.blocked:
        mask[r, c] = 1.0
    blocked_layer = go.Heatmap(
        z=mask,
        x=x_vals,
        y=y_vals,
        colorscale=[[0, "black"], [1, "black"]],
        showscale=False,
        hoverinfo="skip",
        zmin=1,
        zmax=1,
        opacity=1.0,
    )

    robot = go.Scatter(
        x=[cc],
        y=[rr],
        mode="markers",
        marker=dict(symbol="star", size=22, line=dict(color="white", width=2)),
        name="Robot",
        hovertemplate="Robot<br>r %{y} c %{x}<extra></extra>",
        showlegend=False,
    )

    fig = go.Figure(data=[heat, blocked_layer, robot])

    # Axis ranges (cell edges) and square scaling
    fig.update_xaxes(visible=False, range=[-0.5, model.w - 0.5], constrain="domain")
    fig.update_yaxes(visible=False, range=[model.h - 0.5, -0.5], scaleanchor="x", constrain="domain")

    # Paper-aligned subtle background (does not move with pan/zoom)
    if FIELD_IMAGE_DATA_URI:
        add_paper_background(fig, FIELD_IMAGE_DATA_URI, opacity=1.0)

    # keep same aspect ratio but scale down
    orig_w, orig_h = 1405, 652
    scale = 0.9  # adjust (0.5 = half size, 0.8 = 80%, etc.)
    fig.update_layout(
        width=int(orig_w * scale),
        height=int(orig_h * scale),
        margin=dict(l=0, r=0, t=0, b=0),
        paper_bgcolor="#1c2128",
        plot_bgcolor="#1c2128",
        font=dict(color="#e6edf3"),
        showlegend=False,
        autosize=False,
    )
    return fig

# -----------------------------
# Dash app
# -----------------------------
# Explicitly point to the assets folder (resolves path issues when launching from elsewhere)
ASSETS_PATH = os.path.join(os.path.dirname(__file__), "assets")

app = dash.Dash(
    __name__,
    assets_folder=ASSETS_PATH,
    serve_locally=True,
)

# Optional: force no cache for dev (uncomment if needed)
# app.config.update({
#     "assets_ignore": r"^$",
#     "serve_locally": True,
# })

print("Serving assets from:", ASSETS_PATH, "Exists:", os.path.isdir(ASSETS_PATH))

app.title = "FRC Heatmap"

# Path to persist layout
LAYOUT_SAVE_PATH = os.path.join(os.path.dirname(__file__), "saved_layout.json")

def load_saved_layout() -> Optional[dict]:
    if os.path.isfile(LAYOUT_SAVE_PATH):
        try:
            with open(LAYOUT_SAVE_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            # minimal validation
            if all(k in data for k in ("w", "h", "robot", "blocked")):
                return data
        except Exception as e:
            print("Failed to load saved layout:", e)
    return None

# Stores
# - model_state: the whole model (robot + blocked)
# - ui_mode: "robot" or "blocked"
_saved = load_saved_layout()
if _saved:
    try:
        default_model = GridModel(
            w=_saved["w"],
            h=_saved["h"],
            robot=tuple(_saved["robot"]),
            blocked=[tuple(x) for x in _saved["blocked"]],
        )
        print("Loaded saved layout from disk.")
    except Exception as e:
        print("Invalid saved layout, using defaults:", e)
        default_model = GridModel()
else:
    default_model = GridModel()

# Encode field.png once (shown below heatmap)
FIELD_IMAGE_DATA_URI = None
_field_path = os.path.join(os.path.dirname(__file__), "field.png")
if os.path.isfile(_field_path):
    with open(_field_path, "rb") as _f:
        FIELD_IMAGE_DATA_URI = "data:image/png;base64," + base64.b64encode(_f.read()).decode()
else:
    print("field.png not found; field image below heatmap will be hidden.")

app.layout = html.Div(
    className="container",
    children=[
        html.Link(rel="stylesheet", href="/assets/style.css"),
        html.H2("FRC Robot Cycle Time Heatmap"),
        html.P("Interactive visualization of travel time based on drivetrain kinematics and field obstacles."),
        html.Div(
            className="toolbar",
            children=[
                html.Button("Robot mode", id="btn-robot", n_clicks=0, className="btn"),
                html.Button("Blocked mode", id="btn-blocked", n_clicks=0, className="btn"),
                html.Button("Clear obstacles", id="btn-clear", n_clicks=0, className="btn outline"),
                html.Button("Reset", id="btn-reset", n_clicks=0, className="btn outline"),
                html.Button("Save layout", id="btn-save", n_clicks=0, className="btn outline"),
                html.Span(id="mode-label", className="status-label inline-badge"),
                html.Span(id="save-status", className="status-label", style={"marginLeft": "10px"}),
            ]
        ),
        html.Div(
            className="kinematics-panel",
            children=[
                html.H4("Robot Kinematics", className="kinematics-header"),
                html.Div(
                    className="kinematics-inputs",
                    children=[
                        html.Div(className="input-group", children=[
                            html.Label("Max Velocity", htmlFor="input-max-velocity"),
                            dcc.Input(
                                id="input-max-velocity",
                                type="number",
                                value=19.685,
                                min=0.1,
                                max=50,
                                step=0.1,
                                className="kinematics-input",
                            ),
                            html.Span("ft/s", className="input-unit"),
                        ]),
                        html.Div(className="input-group", children=[
                            html.Label("Acceleration", htmlFor="input-acceleration"),
                            dcc.Input(
                                id="input-acceleration",
                                type="number",
                                value=13.1234,
                                min=0.1,
                                max=50,
                                step=0.1,
                                className="kinematics-input",
                            ),
                            html.Span("ft/s²", className="input-unit"),
                        ]),
                        html.Div(className="input-group", children=[
                            html.Label("Deceleration", htmlFor="input-deceleration"),
                            dcc.Input(
                                id="input-deceleration",
                                type="number",
                                value=13.1234,
                                min=0.1,
                                max=50,
                                step=0.1,
                                className="kinematics-input",
                            ),
                            html.Span("ft/s²", className="input-unit"),
                        ]),
                        html.Div(className="input-group", children=[
                            html.Label("Turn Time (90°)", htmlFor="input-turn-time"),
                            dcc.Input(
                                id="input-turn-time",
                                type="number",
                                value=0.2,
                                min=0.01,
                                max=5,
                                step=0.01,
                                className="kinematics-input",
                            ),
                            html.Span("sec", className="input-unit"),
                        ]),
                    ]
                ),
            ]
        ),
        html.Div(
            className="panel-card",
            children=[
                html.H4("Field Heatmap", style={"marginTop": 0, "marginBottom": "8px"}),
                dcc.Graph(
                    id="heatmap",
                    className="graph-wrapper",
                    style={"height": "640px", "width": "100%"},
                    config={"displaylogo": False, "modeBarButtonsToRemove": ["zoom2d","pan2d","lasso2d","select2d"]}
                ),
                # Removed standalone field <img> since we now overlay it under the heatmap.
            ]
        ),
        html.Div(
            className="footer",
            children=[
                html.Span("Made for FRC strategy & path planning • "),
                html.Span("Adjust robot/obstacles by clicking the heatmap. Code uses Dijkstra + kinematic time model."),
            ]
        ),
        dcc.Store(id="model_state", data={
            "w": default_model.w,
            "h": default_model.h,
            "robot": list(default_model.robot),
            "blocked": [list(p) for p in default_model.blocked],
        }),
        dcc.Store(id="ui_mode", data="robot"),
        dcc.Store(id="kinematics_state", data={
            "max_velocity": 19.685,
            "acceleration": 13.1234,
            "deceleration": 13.1234,
            "turn_time": 0.2,
        }),
    ]
)

# Update mode label when mode changes
@app.callback(
    Output("mode-label", "children"),
    Input("ui_mode", "data"),
)
def show_mode(mode):
    return f"Current mode: {mode.capitalize()}"

# Mode switching and clear/reset buttons
@app.callback(
    Output("ui_mode", "data", allow_duplicate=True),
    Output("model_state", "data", allow_duplicate=True),
    Input("btn-robot", "n_clicks"),
    Input("btn-blocked", "n_clicks"),
    Input("btn-clear", "n_clicks"),
    Input("btn-reset", "n_clicks"),
    State("ui_mode", "data"),
    State("model_state", "data"),
    prevent_initial_call=True,
)
def on_toolbar(robot_clicks, blocked_clicks, clear_clicks, reset_clicks, mode, data):
    trigger = ctx.triggered_id
    model = GridModel(
        w=data["w"], h=data["h"],
        robot=tuple(data["robot"]),
        blocked=[tuple(x) for x in data["blocked"]]
    )
    if trigger == "btn-robot":
        return "robot", data
    if trigger == "btn-blocked":
        return "blocked", data
    if trigger == "btn-clear":
        model.blocked = []
        return mode, {"w": model.w, "h": model.h, "robot": list(model.robot), "blocked": [list(p) for p in model.blocked]}
    if trigger == "btn-reset":
        model = GridModel()  # back to defaults
        return "robot", {"w": model.w, "h": model.h, "robot": list(model.robot), "blocked": [list(p) for p in model.blocked]}
    return dash.no_update, dash.no_update

# Handle clicks on the heatmap: place robot or toggle block
@app.callback(
    Output("model_state", "data"),
    Input("heatmap", "clickData"),
    State("ui_mode", "data"),
    State("model_state", "data"),
    prevent_initial_call=True,
)

def on_click(click_data, mode, data):
    if not click_data or "points" not in click_data or not click_data["points"]:
        return dash.no_update
    pt = click_data["points"][0]
    r = int(pose[1]) # non-updating:    r = int(pt["y"])        |        c = int(pt["x"])
    c = int(pose[0]) # int casting optional (still runs without error); float otherwise
    #uselessRot = int(pose[2])

    model = GridModel(
        w=data["w"], h=data["h"],
        robot=tuple(data["robot"]),
        blocked=[tuple(x) for x in data["blocked"]]
    )

    if mode == "robot":
        model.set_robot(r, c)
    else:
        model.toggle_blocked(r, c)

    return {"w": model.w, "h": model.h, "robot": list(model.robot), "blocked": [list(p) for p in model.blocked]}

# Sync kinematics inputs to state store
@app.callback(
    Output("kinematics_state", "data"),
    Input("input-max-velocity", "value"),
    Input("input-acceleration", "value"),
    Input("input-deceleration", "value"),
    Input("input-turn-time", "value"),
    prevent_initial_call=True,
)
def sync_kinematics(max_vel, accel, decel, turn):
    def safe_float(val, default):
        try:
            v = float(val)
            return v if v > 0 else default
        except (TypeError, ValueError):
            return default

    return {
        "max_velocity": safe_float(max_vel, 19.685),
        "acceleration": safe_float(accel, 13.1234),
        "deceleration": safe_float(decel, 13.1234),
        "turn_time": safe_float(turn, 0.2),
    }

# Redraw figure whenever the model or kinematics changes
@app.callback(
    Output("heatmap", "figure"),
    Input("model_state", "data"),
    Input("kinematics_state", "data"),
)
def redraw(data, kin_data):
    model = GridModel(
        w=data["w"], h=data["h"],
        robot=tuple(data["robot"]),
        blocked=[tuple(x) for x in data["blocked"]]
    )
    kinematics = RobotKinematics(
        max_velocity=kin_data["max_velocity"],
        acceleration=kin_data["acceleration"],
        deceleration=kin_data["deceleration"],
        turn_time=kin_data["turn_time"],
    )
    return build_figure(model, kinematics)

# Highlight active mode button
@app.callback(
    Output("btn-robot", "className"),
    Output("btn-blocked", "className"),
    Input("ui_mode", "data"),
)
def highlight_active(mode):
    base = "btn"
    robot_cls = base + (" active" if mode == "robot" else "")
    blocked_cls = base + (" active" if mode == "blocked" else "")
    return robot_cls, blocked_cls

@app.callback(
    Output("save-status", "children"),
    Input("btn-save", "n_clicks"),
    State("model_state", "data"),
    prevent_initial_call=True,
)
def save_layout(n, data):
    try:
        with open(LAYOUT_SAVE_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
        return f"Saved {datetime.now().strftime('%H:%M:%S')}"
    except Exception as e:
        return f"Save failed: {e}"

if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', port=8050)
