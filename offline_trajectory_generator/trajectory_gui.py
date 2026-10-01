#!/usr/bin/env python3
"""Tkinter GUI for offline trajectory generation and RT lane preview."""

from __future__ import annotations

import argparse
import threading
import time
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import tkinter as tk
import tkinter.font as tkfont
import yaml
from tkinter import filedialog, messagebox, ttk

from generate_global_trajectory import (
    DEFAULT_VELOCITY_LIMITS_CSV,
    GenerationResult,
    default_output_dir,
    generate_trajectory,
    write_debug_image,
    write_outputs,
    world_to_pixel,
)


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
DEFAULT_PARAMS_YAML = SCRIPT_DIR / "gui_params.yaml"
NO_GENERATE_KEYS = {"output_dir", "debug_image", "show_centerline", "show_rt_lane"}


@dataclass(frozen=True)
class NumericSpec:
    key: str
    label: str
    default: float
    min_value: float
    max_value: float
    resolution: float
    integer: bool = False
    group: str = "General"


NUMERIC_SPECS = [
    # Sampling resolution
    NumericSpec("waypoint_step", "Waypoint step", 0.1, 0.03, 0.5, 0.01, group="Sampling"),
    NumericSpec("optimizer_step", "Optimizer step", 0.2, 0.1, 0.8, 0.01, group="Sampling"),
    NumericSpec("raceline_smooth_sigma", "RL smooth sigma", 1.0, 0.0, 4.0, 0.1, group="Sampling"),
    # Track width & safety
    NumericSpec("safety_width", "Safety width", 0.35, 0.08, 1.0, 0.01, group="Track & safety"),
    NumericSpec("boundary_margin", "Boundary margin", 0.03, 0.0, 0.25, 0.005, group="Track & safety"),
    NumericSpec("max_width_distance", "Max width dist", 5.0, 0.3, 8.0, 0.1, group="Track & safety"),
    # Speed profile
    NumericSpec("max_speed", "Max speed", 4.0, 0.5, 9.0, 0.1, group="Speed profile"),
    NumericSpec("min_speed", "Min speed", 1.0, 0.1, 4.0, 0.1, group="Speed profile"),
    NumericSpec("max_curvature", "Max curvature", 1.2, 0.0, 3.0, 0.05, group="Speed profile"),
    # Map cleanup & smoothing
    NumericSpec("smooth_sigma", "Smooth sigma", 2.0, 0.0, 6.0, 0.1, group="Map cleanup"),
    NumericSpec("median_kernel", "Median kernel", 3, 1, 11, 2, integer=True, group="Map cleanup"),
    NumericSpec("morph_kernel", "Morph kernel", 5, 1, 15, 2, integer=True, group="Map cleanup"),
    NumericSpec("morph_open_iterations", "Morph open", 1, 0, 4, 1, integer=True, group="Map cleanup"),
    NumericSpec("morph_close_iterations", "Morph close", 1, 0, 4, 1, integer=True, group="Map cleanup"),
    # Centerline extraction
    NumericSpec("skeleton_prune_iterations", "Skel prune", 80, 0, 250, 5, integer=True, group="Centerline"),
    NumericSpec("min_skeleton_component_area", "Skel min area", 40, 0, 1000, 10, integer=True, group="Centerline"),
    NumericSpec("min_track_width", "Min track width", 0.3, 0.0, 1.5, 0.05, group="Centerline"),
    NumericSpec("min_centerline_angle", "Min path angle", 75.0, 0.0, 120.0, 1.0, group="Centerline"),
    NumericSpec("spike_filter_iterations", "Spike filter", 8, 0, 12, 1, integer=True, group="Centerline"),
    # Min-curvature optimizer
    NumericSpec("max_optimizer_iter", "Optimizer iter", 200, 40, 400, 10, integer=True, group="Min-curvature"),
    NumericSpec("curvature_weight", "Curvature wt", 1.0, 0.0, 5.0, 0.05, group="Min-curvature"),
    NumericSpec("smooth_weight", "Smooth wt", 0.04, 0.0, 0.3, 0.005, group="Min-curvature"),
    NumericSpec("length_weight", "Length wt", 0.002, 0.0, 0.02, 0.0005, group="Min-curvature"),
    # D-ratio offset optimizer (positive = toward d_right, negative = toward d_left)
    NumericSpec("d_ratio", "D ratio", 0.0, -1.0, 1.0, 0.05, group="D-ratio"),
    NumericSpec("d_ratio_alpha_smooth_sigma", "Alpha smooth", 0.0, 0.0, 10.0, 0.5, group="D-ratio"),
    # Straight-segment replacement
    NumericSpec("straight_kappa_threshold", "Straight kappa", 0.2, 0.0, 0.3, 0.005, group="Straightening"),
    NumericSpec("straight_min_length", "Straight min len", 1.5, 0.3, 8.0, 0.1, group="Straightening"),
    NumericSpec("straight_clearance_margin", "Straight margin", 0.03, 0.0, 0.3, 0.005, group="Straightening"),
    NumericSpec("straight_blend_length", "Straight blend", 0.5, 0.0, 2.0, 0.05, group="Straightening"),
]
SPEC_BY_KEY = {spec.key: spec for spec in NUMERIC_SPECS}


def normalize_numeric_value(
    spec: NumericSpec,
    value: Any,
    *,
    clamp_to_slider: bool = False,
) -> float | int:
    number = float(value)
    if spec.resolution > 0.0:
        steps = round((number - spec.min_value) / spec.resolution)
        number = spec.min_value + steps * spec.resolution
    number = max(number, spec.min_value)
    if clamp_to_slider:
        number = min(number, spec.max_value)
    if spec.integer:
        return int(round(number))
    return float(number)


def numeric_decimals(spec: NumericSpec) -> int:
    if spec.integer:
        return 0
    text = f"{spec.resolution:.10f}".rstrip("0").rstrip(".")
    if "." not in text:
        return 0
    return len(text.split(".", maxsplit=1)[1])


def format_numeric_value(spec: NumericSpec, value: Any) -> str:
    normalized = normalize_numeric_value(spec, value)
    if spec.integer:
        return f"{int(normalized)}"
    return f"{float(normalized):.{numeric_decimals(spec)}f}"


def normalize_gui_values(values: dict[str, Any]) -> dict[str, Any]:
    normalized = dict(values)
    for spec in NUMERIC_SPECS:
        try:
            normalized[spec.key] = normalize_numeric_value(spec, normalized.get(spec.key, spec.default))
        except (TypeError, ValueError):
            normalized[spec.key] = spec.default
    return normalized


def first_existing_map() -> Path:
    candidates = [
        REPO_ROOT / "monte_carlo_localization/maps/slam_map.yaml",
        REPO_ROOT / "monte_carlo_localization/maps/first_map.yaml",
        REPO_ROOT / "src/monte_carlo_localization/maps/map.yaml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return REPO_ROOT / "monte_carlo_localization/maps/slam_map.yaml"


def default_gui_values() -> dict[str, Any]:
    values: dict[str, Any] = {spec.key: spec.default for spec in NUMERIC_SPECS}
    values.update(
        {
            "map_yaml": "",
            "output_dir": "",
            "velocity_limits_csv": str(DEFAULT_VELOCITY_LIMITS_CSV.relative_to(REPO_ROOT)),
            "optimizer": "centerline",
            "width_mode": "distance",
            "show_centerline": True,
            "show_rt_lane": True,
            "debug_image": True,
            "reverse": False,
            "straighten_straights": True,
            "no_flip_y": False,
            "unknown_as_free": False,
        }
    )
    return values


def load_gui_params(path: Path) -> dict[str, Any]:
    values = default_gui_values()
    if not path.exists():
        return values
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if isinstance(loaded, dict):
        for key in values:
            if key in loaded:
                values[key] = loaded[key]
    return normalize_gui_values(values)


def save_gui_params(path: Path, values: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    serializable: dict[str, Any] = {}
    for key, value in values.items():
        if isinstance(value, Path):
            serializable[key] = str(value)
        elif isinstance(value, np.generic):
            serializable[key] = value.item()
        else:
            serializable[key] = value
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(serializable, stream, sort_keys=False, allow_unicode=True)


def make_namespace(values: dict[str, Any]) -> argparse.Namespace:
    values = normalize_gui_values(values)
    map_yaml_text = str(values["map_yaml"]).strip()
    if not map_yaml_text:
        raise ValueError("map YAML is empty")
    map_yaml = Path(map_yaml_text).expanduser()
    if not map_yaml.is_file():
        raise ValueError(f"map YAML does not exist: {map_yaml}")
    velocity_limits_text = str(values["velocity_limits_csv"]).strip()
    if not velocity_limits_text:
        raise ValueError("velocity limits CSV is empty")
    velocity_limits_csv = Path(velocity_limits_text).expanduser()
    if not velocity_limits_csv.is_absolute():
        velocity_limits_csv = REPO_ROOT / velocity_limits_csv
    if not velocity_limits_csv.is_file():
        raise ValueError(f"velocity limits CSV does not exist: {velocity_limits_csv}")

    return argparse.Namespace(
        map_yaml=map_yaml,
        output_dir=Path(values["output_dir"]).expanduser() if values.get("output_dir") else None,
        waypoint_step=float(values["waypoint_step"]),
        optimizer_step=float(values["optimizer_step"]),
        raceline_smooth_sigma=float(values["raceline_smooth_sigma"]),
        safety_width=float(values["safety_width"]),
        boundary_margin=float(values["boundary_margin"]),
        max_width_distance=float(values["max_width_distance"]),
        width_mode=str(values["width_mode"]),
        max_speed=float(values["max_speed"]),
        min_speed=float(values["min_speed"]),
        velocity_limits_csv=velocity_limits_csv,
        max_curvature=float(values["max_curvature"]),
        smooth_sigma=float(values["smooth_sigma"]),
        median_kernel=int(values["median_kernel"]),
        morph_kernel=int(values["morph_kernel"]),
        morph_open_iterations=int(values["morph_open_iterations"]),
        morph_close_iterations=int(values["morph_close_iterations"]),
        skeleton_prune_iterations=int(values["skeleton_prune_iterations"]),
        min_skeleton_component_area=int(values["min_skeleton_component_area"]),
        min_track_width=float(values["min_track_width"]),
        min_centerline_angle=float(values["min_centerline_angle"]),
        spike_filter_iterations=int(values["spike_filter_iterations"]),
        optimizer=str(values["optimizer"]),
        max_optimizer_iter=int(values["max_optimizer_iter"]),
        curvature_weight=float(values["curvature_weight"]),
        smooth_weight=float(values["smooth_weight"]),
        length_weight=float(values["length_weight"]),
        d_ratio=float(values["d_ratio"]),
        d_ratio_alpha_smooth_sigma=float(values["d_ratio_alpha_smooth_sigma"]),
        straight_kappa_threshold=float(values["straight_kappa_threshold"]),
        straight_min_length=float(values["straight_min_length"]),
        straight_clearance_margin=float(values["straight_clearance_margin"]),
        straight_blend_length=float(values["straight_blend_length"]),
        reverse=bool(values["reverse"]),
        straighten_straights=bool(values["straighten_straights"]),
        no_flip_y=bool(values["no_flip_y"]),
        unknown_as_free=bool(values["unknown_as_free"]),
        debug_image=bool(values.get("debug_image", False)),
    )


def draw_polyline(
    image_rgb: np.ndarray,
    points_xy: np.ndarray,
    result: GenerationResult,
    color: tuple[int, int, int],
    thickness: int,
) -> None:
    pixels = np.round(world_to_pixel(points_xy, result.map_info, result.flip_y)).astype(np.int32)
    pixels[:, 0] = np.clip(pixels[:, 0], 0, result.map_info.width - 1)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, result.map_info.height - 1)
    for p0, p1 in zip(pixels, np.roll(pixels, -1, axis=0)):
        cv2.line(image_rgb, tuple(p0), tuple(p1), color, thickness, cv2.LINE_AA)


def render_preview_rgb(
    result: GenerationResult,
    target_size: tuple[int, int] | None = None,
    show_centerline: bool = True,
    show_rt_lane: bool = True,
) -> np.ndarray:
    image_rgb = cv2.cvtColor(result.image, cv2.COLOR_GRAY2RGB)
    thickness = max(1, int(round(max(result.map_info.width, result.map_info.height) / 700)))

    if show_centerline:
        draw_polyline(image_rgb, result.center_traj.points_xy, result, (0, 120, 255), thickness)
    if show_rt_lane:
        draw_polyline(image_rgb, result.global_traj.points_xy, result, (255, 70, 45), thickness + 1)

    start_px = np.round(
        world_to_pixel(result.global_traj.points_xy[:1], result.map_info, result.flip_y)[0]
    ).astype(np.int32)
    start_px[0] = np.clip(start_px[0], 0, result.map_info.width - 1)
    start_px[1] = np.clip(start_px[1], 0, result.map_info.height - 1)
    cv2.circle(image_rgb, tuple(start_px), max(3, thickness + 2), (0, 220, 90), -1, cv2.LINE_AA)

    if target_size is None:
        return image_rgb

    target_w, target_h = target_size
    if target_w <= 1 or target_h <= 1:
        return image_rgb
    scale = min(target_w / image_rgb.shape[1], target_h / image_rgb.shape[0])
    scale = max(scale, 0.01)
    new_size = (
        max(1, int(round(image_rgb.shape[1] * scale))),
        max(1, int(round(image_rgb.shape[0] * scale))),
    )
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_NEAREST
    return cv2.resize(image_rgb, new_size, interpolation=interpolation)


def rgb_to_photoimage(image_rgb: np.ndarray) -> tk.PhotoImage:
    # Binary PPM (P6) is understood by every Tk version (8.5 has no PNG) and
    # skips the PNG+base64 encode, which keeps resize re-renders cheap.
    height, width = image_rgb.shape[:2]
    header = f"P6 {width} {height} 255 ".encode("ascii")
    data = header + np.ascontiguousarray(image_rgb).tobytes()
    return tk.PhotoImage(data=data, format="PPM")


class TrajectoryGui:
    def __init__(
        self,
        root: tk.Tk,
        initial_map: Path | None,
        initial_output: Path | None,
        params_path: Path,
        initial_values: dict[str, Any],
    ):
        self.root = root
        self.root.title("Offline Trajectory Generator")
        self.root.geometry("1280x820")
        self.root.minsize(940, 620)
        self._setup_style()

        self.variables: dict[str, tk.Variable] = {}
        self.scale_variables: dict[str, tk.DoubleVar] = {}
        self.label_variables: dict[str, tk.StringVar] = {}
        self.entry_variables: dict[str, tk.StringVar] = {}
        self.pending_after: str | None = None
        self.generation_id = 0
        self.running = False
        self.current_result: GenerationResult | None = None
        self.current_args: argparse.Namespace | None = None
        self.photo: tk.PhotoImage | None = None
        self.last_render_size = (0, 0)
        self.params_path = params_path
        self._resize_after: str | None = None

        self._build_layout(initial_map, initial_output, initial_values)
        self.save_current_params()
        if initial_map is None:
            self.status_var.set("Choose a map YAML to start.")
            self.root.after(100, self.prompt_initial_map)
        else:
            self.schedule_generate(delay_ms=100)

    # palette shared by the ttk styles and the plain-tk widgets
    BG = "#eef1f5"          # window background
    CARD = "#ffffff"        # input fields / selected tab
    BORDER = "#d5dae2"
    TEXT = "#1f2430"
    SUBTEXT = "#67707f"
    ACCENT = "#2f6fed"
    ACCENT_DARK = "#2458c4"
    VIEWER_BG = "#14181d"   # preview canvas
    STATUS_BG = "#1b202a"

    def _setup_style(self) -> None:
        style = ttk.Style()
        for preferred in ("clam", "alt", "default"):
            if preferred in style.theme_names():
                style.theme_use(preferred)
                break

        base = tkfont.nametofont("TkDefaultFont")
        family = base.cget("family")
        base.configure(size=10)
        tkfont.nametofont("TkTextFont").configure(size=10)
        self.font_section = (family, 12, "bold")
        self.font_group = (family, 10, "bold")
        self.font_small = (family, 9)

        self.root.configure(background=self.BG)
        style.configure(".", background=self.BG, foreground=self.TEXT)
        style.configure("TFrame", background=self.BG)
        style.configure("TLabel", background=self.BG, foreground=self.TEXT)
        style.configure("Section.TLabel", font=self.font_section, foreground=self.TEXT)
        style.configure("Hint.TLabel", font=self.font_small, foreground=self.SUBTEXT)

        style.configure(
            "TCheckbutton", background=self.BG, foreground=self.TEXT, focuscolor=self.BG
        )
        style.map("TCheckbutton", background=[("active", self.BG)])

        style.configure("TButton", padding=(10, 5), background="#e2e6ec", bordercolor=self.BORDER)
        style.map("TButton", background=[("active", "#d5dae2"), ("pressed", "#c8cdd6")])
        style.configure("Icon.TButton", padding=(4, 3))
        style.configure(
            "Accent.TButton",
            padding=(10, 5),
            background=self.ACCENT,
            foreground="#ffffff",
            bordercolor=self.ACCENT,
            focuscolor=self.ACCENT,
        )
        style.map(
            "Accent.TButton",
            background=[("active", self.ACCENT_DARK), ("pressed", self.ACCENT_DARK)],
            foreground=[("disabled", "#e8ecf4")],
        )

        style.configure(
            "TEntry", fieldbackground=self.CARD, bordercolor=self.BORDER, padding=3
        )
        style.map("TEntry", bordercolor=[("focus", self.ACCENT)])
        style.configure(
            "TCombobox", fieldbackground=self.CARD, background=self.CARD,
            bordercolor=self.BORDER, arrowcolor=self.SUBTEXT, padding=3,
        )
        style.map(
            "TCombobox",
            fieldbackground=[("readonly", self.CARD)],
            bordercolor=[("focus", self.ACCENT)],
        )

        style.configure(
            "TNotebook", background=self.BG, borderwidth=0, tabmargins=(8, 6, 8, 0)
        )
        style.configure(
            "TNotebook.Tab",
            padding=(12, 6),
            font=self.font_small,
            background=self.BG,
            foreground=self.SUBTEXT,
            bordercolor=self.BORDER,
        )
        style.map(
            "TNotebook.Tab",
            background=[("selected", self.CARD)],
            foreground=[("selected", self.ACCENT)],
            expand=[("selected", (1, 1, 1, 0))],
        )

        style.configure(
            "TLabelframe",
            background=self.BG,
            bordercolor=self.BORDER,
            relief="solid",
            borderwidth=1,
            padding=(8, 4, 8, 6),
        )
        style.configure(
            "TLabelframe.Label",
            background=self.BG,
            foreground=self.ACCENT,
            font=self.font_group,
        )

        style.configure(
            "Horizontal.TScale", background=self.BG, troughcolor="#dbe0e8",
            bordercolor=self.BORDER, lightcolor=self.ACCENT, darkcolor=self.ACCENT,
        )

        style.configure("TSeparator", background=self.BORDER)
        style.configure("Status.TFrame", background=self.STATUS_BG)
        style.configure(
            "Status.TLabel", background=self.STATUS_BG, foreground="#dfe4ec", font=self.font_small
        )
        style.configure(
            "StatusStrong.TLabel",
            background=self.STATUS_BG,
            foreground="#7fb0ff",
            font=(family, 10, "bold"),
        )

    def _build_layout(
        self,
        initial_map: Path | None,
        initial_output: Path | None,
        initial_values: dict[str, Any],
    ) -> None:
        self.root.columnconfigure(1, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.root.rowconfigure(1, weight=0)

        left = ttk.Frame(self.root, width=400)
        left.grid(row=0, column=0, sticky="nsw")
        left.grid_propagate(False)
        left.rowconfigure(0, weight=1)

        panel_canvas = tk.Canvas(left, highlightthickness=0, width=400, background=self.BG)
        panel_scroll = ttk.Scrollbar(left, orient="vertical", command=panel_canvas.yview)
        self.panel = ttk.Frame(panel_canvas)
        panel_window = panel_canvas.create_window((0, 0), window=self.panel, anchor="nw")
        self.panel.bind(
            "<Configure>",
            lambda event: panel_canvas.configure(scrollregion=panel_canvas.bbox("all")),
        )
        panel_canvas.bind(
            "<Configure>",
            lambda event: panel_canvas.itemconfigure(panel_window, width=event.width),
        )
        self._bind_panel_scroll(panel_canvas)
        panel_canvas.configure(yscrollcommand=panel_scroll.set)
        panel_canvas.grid(row=0, column=0, sticky="nsew")
        panel_scroll.grid(row=0, column=1, sticky="ns")

        viewer_frame = ttk.Frame(self.root)
        viewer_frame.grid(row=0, column=1, sticky="nsew")
        viewer_frame.rowconfigure(0, weight=1)
        viewer_frame.columnconfigure(0, weight=1)

        self.canvas = tk.Canvas(viewer_frame, background=self.VIEWER_BG, highlightthickness=0)
        self.canvas.grid(row=0, column=0, sticky="nsew", padx=(0, 0))
        self.canvas.bind("<Configure>", self._on_canvas_resize)

        status = ttk.Frame(self.root, style="Status.TFrame")
        status.grid(row=1, column=0, columnspan=2, sticky="ew")
        status.columnconfigure(0, weight=1)
        self.status_var = tk.StringVar(value="Ready")
        self.metrics_var = tk.StringVar(value="")
        ttk.Label(status, textvariable=self.status_var, anchor="w", style="Status.TLabel").grid(
            row=0, column=0, sticky="ew", padx=10, pady=5
        )
        ttk.Label(
            status, textvariable=self.metrics_var, anchor="e", style="StatusStrong.TLabel"
        ).grid(row=0, column=1, sticky="e", padx=10, pady=5)

        self._build_controls(initial_map, initial_output, initial_values)

    def _bind_panel_scroll(self, panel_canvas: tk.Canvas) -> None:
        def pointer_inside_canvas(event: tk.Event) -> bool:
            x0 = panel_canvas.winfo_rootx()
            y0 = panel_canvas.winfo_rooty()
            x1 = x0 + panel_canvas.winfo_width()
            y1 = y0 + panel_canvas.winfo_height()
            return x0 <= event.x_root <= x1 and y0 <= event.y_root <= y1

        def on_mousewheel(event: tk.Event) -> str | None:
            if not pointer_inside_canvas(event):
                return None
            if getattr(event, "num", None) == 4:
                delta = -3
            elif getattr(event, "num", None) == 5:
                delta = 3
            else:
                delta = -int(event.delta / 120) if event.delta else 0
            if delta:
                panel_canvas.yview_scroll(delta, "units")
            return "break"

        self.root.bind_all("<MouseWheel>", on_mousewheel, add="+")
        self.root.bind_all("<Button-4>", on_mousewheel, add="+")
        self.root.bind_all("<Button-5>", on_mousewheel, add="+")

    # Notebook tab -> NUMERIC_SPECS groups shown inside it (in spec order).
    TAB_GROUPS = (
        ("Track / speed", ("Sampling", "Track & safety", "Speed profile")),
        ("Map / centerline", ("Map cleanup", "Centerline")),
        ("Optimizer", ("Min-curvature", "D-ratio", "Straightening")),
    )

    def _build_controls(
        self,
        initial_map: Path | None,
        initial_output: Path | None,
        initial_values: dict[str, Any],
    ) -> None:
        pad = {"padx": 10, "pady": 4}
        row = 0

        ttk.Label(self.panel, text="Offline Trajectory Generator", style="Section.TLabel").grid(
            row=row, column=0, columnspan=3, sticky="w", padx=10, pady=(10, 2)
        )
        row += 1

        map_value = str(initial_map) if initial_map is not None else str(initial_values.get("map_yaml", ""))
        self.variables["map_yaml"] = tk.StringVar(value=map_value)
        self._entry_with_button(row, "Map", self.variables["map_yaml"], self.browse_map)
        row += 1

        saved_output = str(initial_values.get("output_dir", ""))
        if initial_output is not None:
            default_output = str(initial_output)
        elif initial_map is not None:
            default_output = saved_output or str(default_output_dir(initial_map))
        else:
            default_output = saved_output
        self.variables["output_dir"] = tk.StringVar(value=default_output)
        self._entry_with_button(row, "Output", self.variables["output_dir"], self.browse_output)
        row += 1

        self.variables["velocity_limits_csv"] = tk.StringVar(
            value=str(initial_values.get("velocity_limits_csv", DEFAULT_VELOCITY_LIMITS_CSV))
        )
        self._entry_with_button(
            row, "Velocity limits", self.variables["velocity_limits_csv"], self.browse_velocity_limits
        )
        row += 1

        buttons = ttk.Frame(self.panel)
        buttons.grid(row=row, column=0, columnspan=3, sticky="ew", padx=10, pady=(6, 4))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        ttk.Button(
            buttons, text="Rebuild", style="Accent.TButton",
            command=lambda: self.schedule_generate(0),
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(buttons, text="Save outputs", command=self.save_outputs).grid(
            row=0, column=1, sticky="ew", padx=(4, 0)
        )
        row += 1

        optimizer_value = str(initial_values.get("optimizer", "centerline"))
        if optimizer_value not in ("centerline", "mincurv", "d_ratio"):
            optimizer_value = "mincurv"  # saved YAML may hold a removed optimizer
        self.variables["optimizer"] = tk.StringVar(value=optimizer_value)
        self._combo(row, "Optimizer", self.variables["optimizer"], ("centerline", "mincurv", "d_ratio"))
        row += 1

        self.variables["width_mode"] = tk.StringVar(value=str(initial_values.get("width_mode", "distance")))
        self._combo(row, "Width mode", self.variables["width_mode"], ("hybrid", "distance", "raycast"))
        row += 1

        checks = ttk.Frame(self.panel)
        checks.grid(row=row, column=0, columnspan=3, sticky="ew", padx=10, pady=(4, 2))
        checks.columnconfigure(0, weight=1)
        checks.columnconfigure(1, weight=1)
        for key, default in (
            ("show_centerline", True), ("show_rt_lane", True), ("debug_image", True),
            ("reverse", False), ("straighten_straights", True), ("no_flip_y", False),
            ("unknown_as_free", False),
        ):
            self.variables[key] = tk.BooleanVar(value=bool(initial_values.get(key, default)))
        check_items = (
            ("Show centerline", "show_centerline"),
            ("Show RT lane", "show_rt_lane"),
            ("Save debug image", "debug_image"),
            ("Reverse direction", "reverse"),
            ("Straighten straights", "straighten_straights"),
            ("No flip Y", "no_flip_y"),
            ("Unknown as free", "unknown_as_free"),
        )
        for index, (label, key) in enumerate(check_items):
            ttk.Checkbutton(checks, text=label, variable=self.variables[key]).grid(
                row=index // 2, column=index % 2, sticky="w", padx=(0, 6), pady=1
            )
        row += 1

        notebook = ttk.Notebook(self.panel)
        notebook.grid(row=row, column=0, columnspan=3, sticky="nsew", padx=10, pady=(8, 10))
        row += 1

        specs_by_group: dict[str, list[NumericSpec]] = {}
        for spec in NUMERIC_SPECS:
            specs_by_group.setdefault(spec.group, []).append(spec)
            value = normalize_numeric_value(spec, initial_values.get(spec.key, spec.default))
            self.variables[spec.key] = tk.DoubleVar(value=float(value))

        for tab_title, groups in self.TAB_GROUPS:
            tab = ttk.Frame(notebook, padding=(6, 8, 6, 8))
            tab.columnconfigure(0, weight=1)
            notebook.add(tab, text=tab_title)
            tab_row = 0
            for group in groups:
                frame = ttk.Labelframe(tab, text=group)
                frame.grid(row=tab_row, column=0, sticky="ew", pady=(0, 8))
                frame.columnconfigure(0, weight=1)
                for group_row, spec in enumerate(specs_by_group.get(group, ())):
                    self._scale(frame, group_row, spec)
                tab_row += 1
        # Any group not listed in TAB_GROUPS lands on a trailing tab so new
        # NUMERIC_SPECS groups can never silently disappear from the GUI.
        known = {group for _, groups in self.TAB_GROUPS for group in groups}
        leftover = [g for g in specs_by_group if g not in known]
        if leftover:
            tab = ttk.Frame(notebook, padding=(6, 8, 6, 8))
            tab.columnconfigure(0, weight=1)
            notebook.add(tab, text="Other")
            for tab_row, group in enumerate(leftover):
                frame = ttk.Labelframe(tab, text=group)
                frame.grid(row=tab_row, column=0, sticky="ew", pady=(0, 8))
                frame.columnconfigure(0, weight=1)
                for group_row, spec in enumerate(specs_by_group[group]):
                    self._scale(frame, group_row, spec)

        self.panel.columnconfigure(1, weight=1)
        self.panel.columnconfigure(2, weight=0)
        for key, variable in self.variables.items():
            variable.trace_add("write", lambda *_args, changed_key=key: self.on_variable_changed(changed_key))

    def _entry_with_button(self, row: int, label: str, variable: tk.Variable, command: Any) -> None:
        ttk.Label(self.panel, text=label).grid(row=row, column=0, sticky="w", padx=(10, 4), pady=4)
        ttk.Entry(self.panel, textvariable=variable, width=26).grid(row=row, column=1, sticky="ew", pady=4)
        ttk.Button(self.panel, text="…", width=2, style="Icon.TButton", command=command).grid(
            row=row, column=2, sticky="e", padx=(4, 10), pady=4
        )

    def _combo(self, row: int, label: str, variable: tk.Variable, values: tuple[str, ...]) -> None:
        ttk.Label(self.panel, text=label).grid(row=row, column=0, sticky="w", padx=(10, 4), pady=4)
        combo = ttk.Combobox(
            self.panel, textvariable=variable, values=list(values), state="readonly"
        )
        if variable.get() not in values:
            variable.set(values[0])
        combo.grid(row=row, column=1, columnspan=2, sticky="ew", padx=(0, 10), pady=4)

    def _scale(self, parent: ttk.Widget, row: int, spec: NumericSpec) -> None:
        variable = self.variables[spec.key]
        slider_var = tk.DoubleVar(
            value=float(normalize_numeric_value(spec, variable.get(), clamp_to_slider=True))
        )
        self.scale_variables[spec.key] = slider_var
        label_var = tk.StringVar()
        entry_var = tk.StringVar()
        self.label_variables[spec.key] = label_var
        self.entry_variables[spec.key] = entry_var
        syncing_slider = False

        def update_widgets(*_: Any, force_entry: bool = False) -> None:
            nonlocal syncing_slider
            formatted = format_numeric_value(spec, variable.get())
            label_var.set(spec.label)
            if force_entry or self.root.focus_get() is not entry:
                entry_var.set(formatted)
            slider_value = normalize_numeric_value(spec, variable.get(), clamp_to_slider=True)
            if abs(float(slider_var.get()) - float(slider_value)) > 1e-12:
                syncing_slider = True
                try:
                    slider_var.set(float(slider_value))
                finally:
                    syncing_slider = False

        def apply_entry(_event: tk.Event | None = None) -> str:
            text = entry_var.get().strip()
            try:
                value = normalize_numeric_value(spec, text)
            except (TypeError, ValueError):
                update_widgets(force_entry=True)
                self.status_var.set(f"Invalid {spec.label}: {text}")
                return "break"
            variable.set(float(value))
            entry_var.set(format_numeric_value(spec, value))
            return "break"

        def apply_scale(value_text: str) -> None:
            if syncing_slider:
                return
            value = normalize_numeric_value(spec, value_text, clamp_to_slider=True)
            if abs(float(variable.get()) - float(value)) > 1e-12:
                variable.set(float(value))

        variable.trace_add("write", update_widgets)
        control = ttk.Frame(parent)
        control.grid(row=row, column=0, sticky="ew", pady=(2, 2))
        control.columnconfigure(2, weight=1)

        ttk.Label(control, textvariable=label_var, width=17, anchor="w").grid(
            row=0, column=0, sticky="w"
        )
        entry = ttk.Entry(control, textvariable=entry_var, width=8, justify="right")
        entry.grid(row=0, column=1, sticky="ew", padx=(4, 8))
        scale = ttk.Scale(
            control,
            from_=spec.min_value,
            to=spec.max_value,
            variable=slider_var,
            command=apply_scale,
        )
        scale.grid(row=0, column=2, sticky="ew")
        entry.bind("<Return>", apply_entry)
        entry.bind("<KP_Enter>", apply_entry)
        entry.bind("<FocusOut>", apply_entry)
        update_widgets(force_entry=True)

    def browse_map(self) -> None:
        filename = filedialog.askopenfilename(
            title="Select ROS map YAML",
            initialdir=str(first_existing_map().parent),
            filetypes=(("Map YAML", "*.yaml *.yml"), ("All files", "*")),
        )
        if filename:
            self.variables["map_yaml"].set(filename)
            if not self.variables["output_dir"].get():
                self.variables["output_dir"].set(str(default_output_dir(Path(filename))))

    def prompt_initial_map(self) -> None:
        self.browse_map()
        if not self.variables["map_yaml"].get().strip():
            self._show_canvas_message("Select a map YAML with the Map … button.")

    def browse_output(self) -> None:
        dirname = filedialog.askdirectory(initialdir=str(REPO_ROOT))
        if dirname:
            self.variables["output_dir"].set(dirname)

    def browse_velocity_limits(self) -> None:
        filename = filedialog.askopenfilename(
            title="Select velocity limits CSV",
            initialdir=str(DEFAULT_VELOCITY_LIMITS_CSV.parent),
            filetypes=(("Velocity limits CSV", "*.csv"), ("All files", "*")),
        )
        if filename:
            self.variables["velocity_limits_csv"].set(filename)

    def collect_values(self) -> dict[str, Any]:
        values: dict[str, Any] = {}
        for key, variable in self.variables.items():
            if isinstance(variable, tk.BooleanVar):
                values[key] = bool(variable.get())
            else:
                values[key] = variable.get()
        for spec in NUMERIC_SPECS:
            values[spec.key] = normalize_numeric_value(spec, values[spec.key])
        return values

    def save_current_params(self) -> None:
        try:
            save_gui_params(self.params_path, self.collect_values())
        except OSError as exc:
            self.status_var.set(f"Parameter save failed: {exc}")

    def on_variable_changed(self, key: str) -> None:
        self.save_current_params()
        if key in {"show_centerline", "show_rt_lane"}:
            self.render_current()
            return
        if key not in NO_GENERATE_KEYS:
            self.schedule_generate()

    def schedule_generate(self, delay_ms: int = 450) -> None:
        if self.pending_after is not None:
            self.root.after_cancel(self.pending_after)
        self.pending_after = self.root.after(delay_ms, self.start_generate)

    def start_generate(self) -> None:
        self.pending_after = None
        values = self.collect_values()
        try:
            args = make_namespace(values)
        except (TypeError, ValueError) as exc:
            self.status_var.set(f"Invalid parameter: {exc}")
            return

        self.generation_id += 1
        generation_id = self.generation_id
        self.running = True
        self.status_var.set("Generating...")

        def worker() -> None:
            started = time.perf_counter()
            try:
                result = generate_trajectory(args)
            except Exception as exc:  # noqa: BLE001 - GUI reports calculation failures to the user.
                # Bind `exc` as a default argument: Python clears the `except`
                # target when the block exits, so a plain closure would raise
                # NameError once Tk actually runs the callback.
                traceback.print_exc()
                self.root.after(0, lambda error=exc: self.finish_error(generation_id, error))
                return
            elapsed = time.perf_counter() - started
            self.root.after(0, lambda: self.finish_generate(generation_id, args, result, elapsed))

        threading.Thread(target=worker, daemon=True).start()

    def finish_error(self, generation_id: int, exc: Exception) -> None:
        if generation_id != self.generation_id:
            return
        self.running = False
        self.status_var.set(f"Error: {exc}")
        self.metrics_var.set("")
        self._show_canvas_message(str(exc))

    def finish_generate(
        self,
        generation_id: int,
        args: argparse.Namespace,
        result: GenerationResult,
        elapsed: float,
    ) -> None:
        if generation_id != self.generation_id:
            return
        self.running = False
        self.current_args = args
        self.current_result = result
        off_map = int(getattr(result, "off_map_wpnts", 0))
        kappa_bad = int(getattr(result, "kappa_violations", 0))
        if off_map:
            self.status_var.set(
                f"Generated in {elapsed:.2f}s — ⚠ {off_map} waypoints off the map "
                "(noise region picked? raise Min track width)"
            )
        elif kappa_bad:
            self.status_var.set(
                f"Generated in {elapsed:.2f}s — ⚠ {kappa_bad} waypoints exceed Max "
                f"curvature (max |κ| {float(getattr(result, 'max_abs_kappa', 0.0)):.2f}) "
                "— not drivable as-is"
            )
        else:
            self.status_var.set(f"Generated in {elapsed:.2f}s")
        self.metrics_var.set(
            f"lap {result.lap_time:.3f} s   ·   {len(result.global_traj.points_xy)} wpts"
            f"   ·   max |κ| {float(getattr(result, 'max_abs_kappa', 0.0)):.2f}"
        )
        self.render_current()

    def _on_canvas_resize(self, _event: tk.Event) -> None:
        # Debounce: window drags fire many <Configure> events; re-render once,
        # at the final size, so the preview always fits the current canvas.
        if self._resize_after is not None:
            self.root.after_cancel(self._resize_after)
        self._resize_after = self.root.after(120, self._refit_after_resize)

    def _refit_after_resize(self) -> None:
        self._resize_after = None
        size = (self.canvas.winfo_width(), self.canvas.winfo_height())
        if size == self.last_render_size:
            return
        if self.current_result is not None:
            self.render_current()
        else:
            self._show_canvas_message("Select a map YAML with the Map … button.")
            self.last_render_size = size

    def _show_canvas_message(self, text: str) -> None:
        self.canvas.delete("all")
        self.canvas.create_text(
            max(1, self.canvas.winfo_width()) // 2,
            max(1, self.canvas.winfo_height()) // 2,
            text=text,
            fill="#aeb6c2",
            font=self.font_group,
            width=max(300, self.canvas.winfo_width() - 80),
            justify="center",
        )

    def render_current(self) -> None:
        if self.current_result is None:
            return
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        image_rgb = render_preview_rgb(
            self.current_result,
            (width - 24, height - 24),
            show_centerline=bool(self.variables["show_centerline"].get()),
            show_rt_lane=bool(self.variables["show_rt_lane"].get()),
        )
        self.photo = rgb_to_photoimage(image_rgb)
        self.canvas.delete("all")
        x = max(0, (width - image_rgb.shape[1]) // 2)
        y = max(0, (height - image_rgb.shape[0]) // 2)
        self.canvas.create_image(x, y, image=self.photo, anchor="nw")
        legend = []
        if bool(self.variables["show_centerline"].get()):
            legend.append(("● centerline", "#4da3ff"))
        if bool(self.variables["show_rt_lane"].get()):
            legend.append(("● raceline", "#ff5a3c"))
        legend.append(("● start", "#2fd47a"))
        lx = 12
        for text, color in legend:
            item = self.canvas.create_text(
                lx, height - 12, text=text, fill=color, anchor="w", font=self.font_small
            )
            lx = self.canvas.bbox(item)[2] + 14
        self.last_render_size = (width, height)

    def save_outputs(self) -> None:
        if self.current_result is None or self.current_args is None:
            messagebox.showwarning("Save", "No trajectory has been generated yet.")
            return
        output_dir_text = self.variables["output_dir"].get().strip()
        output_dir = (
            Path(output_dir_text).expanduser()
            if output_dir_text
            else default_output_dir(self.current_args.map_yaml)
        )
        args = self.current_args
        args.output_dir = output_dir
        args.debug_image = bool(self.variables["debug_image"].get())
        try:
            write_outputs(
                output_dir,
                self.current_result.map_info,
                self.current_result.center_traj,
                self.current_result.global_traj,
                self.current_result.lap_time,
                args,
            )
            if args.debug_image:
                write_debug_image(
                    output_dir,
                    self.current_result.image,
                    self.current_result.map_info,
                    self.current_result.center_traj.points_xy,
                    self.current_result.global_traj.points_xy,
                    self.current_result.flip_y,
                )
        except Exception as exc:  # noqa: BLE001 - GUI reports filesystem failures.
            messagebox.showerror("Save failed", str(exc))
            return
        self.status_var.set(f"Saved to {output_dir}")


def parse_cli() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GUI for offline trajectory generation.")
    parser.add_argument(
        "--map-yaml",
        type=Path,
        default=None,
        help="Initial map YAML. If omitted, the GUI opens a file picker on startup.",
    )
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--params-yaml",
        type=Path,
        default=DEFAULT_PARAMS_YAML,
        help="GUI parameter YAML that is loaded and updated automatically.",
    )
    parser.add_argument(
        "--render-test",
        type=Path,
        default=None,
        help="Headless test mode: render preview PNG and exit.",
    )
    return parser.parse_args()


def resolve_initial_map(cli_map: Path | None, saved_values: dict[str, Any]) -> Path | None:
    if cli_map is not None:
        return cli_map
    saved_map = str(saved_values.get("map_yaml", "")).strip()
    if saved_map:
        saved_path = Path(saved_map)
        if saved_path.is_file():
            return saved_path
    return None


def run_render_test(map_yaml: Path, output_png: Path) -> int:
    values = {spec.key: spec.default for spec in NUMERIC_SPECS}
    values.update(
        {
            "map_yaml": str(map_yaml),
            "output_dir": "",
            "velocity_limits_csv": str(DEFAULT_VELOCITY_LIMITS_CSV),
            "optimizer": "centerline",
            "width_mode": "distance",
            "reverse": False,
            "straighten_straights": True,
            "no_flip_y": False,
            "unknown_as_free": False,
            "debug_image": False,
        }
    )
    result = generate_trajectory(make_namespace(values))
    image_rgb = render_preview_rgb(result, None, True, True)
    output_png.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_png), cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR))
    print(f"Rendered {output_png}")
    print(f"Waypoints: {len(result.global_traj.points_xy)}, estimated lap time: {result.lap_time:.3f}s")
    return 0


def main() -> int:
    args = parse_cli()
    if args.render_test is not None:
        return run_render_test(args.map_yaml or first_existing_map(), args.render_test)

    saved_values = load_gui_params(args.params_yaml)
    initial_map = resolve_initial_map(args.map_yaml, saved_values)
    root = tk.Tk()
    TrajectoryGui(root, initial_map, args.output_dir, args.params_yaml, saved_values)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
