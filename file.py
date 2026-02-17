"""
AI Pathfinder — Uninformed Search Visualizer
=============================================
Library : Matplotlib  (pip install matplotlib)
Run     : python pathfinder.py

CONTROLS
--------
Left-panel radio buttons → choose algorithm
Speed slider             → animation delay
▶ Run                    → start search
■ Stop                   → pause
↺ Reset                  → clear visualization
Clear Grid               → wipe all obstacles

EDIT MODE (click a mode button, then click/drag on grid)
---------------------------------------------------------
  Wall    → left-click to add wall, right-click to erase
  Start   → left-click any open cell to move Start (S)
  Target  → left-click any open cell to move Target (T)

Movement Order (clockwise + main diagonal only per assignment):
  1. Up   2. Right   3. Bottom   4. Bottom-Right (diag)
  5. Left   6. Top-Left (diag)
  * Top-Right and Bottom-Left diagonals are EXCLUDED
"""

import math
import heapq
import copy
from collections import deque

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Button, RadioButtons, Slider
from matplotlib.colors import ListedColormap
import numpy as np

# ─────────────────────────────────────────────────────────────
#  HARDCODED MAZE  (0 = open, -1 = wall)
# ─────────────────────────────────────────────────────────────
DEFAULT_GRID = [
    [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 0,  0,  0, -1,  0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 0,  0, -1, -1,  0, -1,  0, -1,  0, -1, -1, -1,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 0, -1, -1, -1, -1, -1,  0, -1, -1, -1,  0, -1,  0, -1, -1,  0,  0,  0,  0,  0],
    [ 0,  0,  0,  0,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0, -1,  0, -1,  0,  0,  0],
    [ 0,  0, -1,  0, -1,  0, -1,  0,  0, -1,  0, -1, -1,  0, -1,  0, -1,  0,  0,  0],
    [ 0,  0, -1,  0, -1,  0,  0,  0,  0,  0,  0, -1,  0,  0,  0,  0,  0,  0,  0,  0],
    [ 0,  0,  0,  0,  0,  0, -1,  0, -1, -1, -1,  0,  0,  0,  0, -1,  0,  0,  0,  0],
    [ 0, -1, -1, -1,  0, -1,  0,  0,  0,  0,  0,  0, -1, -1,  0, -1,  0,  0,  0,  0],
    [ 0,  0,  0,  0,  0,  0,  0, -1,  0, -1,  0, -1,  0,  0,  0, -1,  0,  0,  0,  0],
    [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
]

DEFAULT_START  = (1, 1)
DEFAULT_TARGET = (10, 18)

ROWS = len(DEFAULT_GRID)
COLS = len(DEFAULT_GRID[0])
DLS_LIMIT = 20

# Movement order exactly as per assignment spec
MOVES = [
    (-1,  0),   # 1. Up
    ( 0, +1),   # 2. Right
    (+1,  0),   # 3. Bottom
    (+1, +1),   # 4. Bottom-Right (main diagonal)
    ( 0, -1),   # 5. Left
    (-1, -1),   # 6. Top-Left    (main diagonal)
    # Top-Right (-1,+1) and Bottom-Left (+1,-1) EXCLUDED
]

# ─────────────────────────────────────────────────────────────
#  VIVID COLOR PALETTE
# ─────────────────────────────────────────────────────────────
BG_DARK    = "#0d0d1a"
PANEL_BG   = "#111128"
GRID_BG    = "#080816"

COL_EMPTY    = "#12122a"  # dark open cell
COL_WALL     = "#ff4757"  # vivid red
COL_START    = "#2ed573"  # neon green
COL_TARGET   = "#1e90ff"  # electric blue
COL_FRONTIER = "#ffa502"  # bright orange
COL_EXPLORED = "#7c4dff"  # vivid purple
COL_PATH     = "#ffdd59"  # bright yellow

COL_GRID_LINE = "#1a1a38"

ACC_CYAN    = "#00d2d3"
ACC_PINK    = "#ff6b81"
ACC_GREEN   = "#2ed573"
ACC_ORANGE  = "#ffa502"
TEXT_BRIGHT = "#ffffff"
TEXT_DIM    = "#aaaacc"


# ─────────────────────────────────────────────────────────────
#  HELPER
# ─────────────────────────────────────────────────────────────
def get_neighbors(grid, node):
    r, c = node
    result = []
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS and grid[nr][nc] != -1:
            result.append((nr, nc))
    return result


# ─────────────────────────────────────────────────────────────
#  SEARCH ALGORITHMS  (generators)
#  yield: (explored_set, frontier_set, path_or_None)
#  path=[] means no path found; path=None means still running
# ─────────────────────────────────────────────────────────────

def algo_bfs(grid, start, target):
    queue   = deque([[start]])
    visited = {start}
    while queue:
        path = queue.popleft()
        node = path[-1]
        frontier = set()
        for n in get_neighbors(grid, node):
            if n not in visited:
                visited.add(n)
                new_path = path + [n]
                if n == target:
                    yield set(visited), set(), new_path
                    return
                frontier.add(n)
                queue.append(new_path)
        yield set(visited), frontier, None
    yield set(visited), set(), []


def algo_dfs(grid, start, target):
    stack   = [[start]]
    visited = {start}
    while stack:
        path = stack.pop()
        node = path[-1]
        if node == target:
            yield set(visited), set(), path
            return
        frontier = set()
        for n in reversed(get_neighbors(grid, node)):
            if n not in visited:
                visited.add(n)
                frontier.add(n)
                stack.append(path + [n])
        yield set(visited), frontier, None
    yield set(visited), set(), []


def algo_ucs(grid, start, target):
    counter = 0
    heap    = [(0, counter, [start])]
    visited = {}
    while heap:
        cost, _, path = heapq.heappop(heap)
        node = path[-1]
        if node in visited:
            continue
        visited[node] = cost
        if node == target:
            yield set(visited), set(), path
            return
        frontier = set()
        for n in get_neighbors(grid, node):
            if n not in visited:
                dr, dc    = abs(n[0]-node[0]), abs(n[1]-node[1])
                step_cost = math.sqrt(2) if (dr == 1 and dc == 1) else 1.0
                counter  += 1
                heapq.heappush(heap, (cost+step_cost, counter, path+[n]))
                frontier.add(n)
        yield set(visited), frontier, None
    yield set(visited), set(), []


def algo_dls(grid, start, target):
    visited_all = set()
    result      = [None]
    stack = [([start], DLS_LIMIT)]
    while stack:
        path, depth = stack.pop()
        node = path[-1]
        visited_all.add(node)
        yield set(visited_all), set(), None
        if node == target:
            result[0] = path
            break
        if depth == 0:
            continue
        path_set = set(path)
        for n in reversed(get_neighbors(grid, node)):
            if n not in path_set:
                stack.append((path + [n], depth - 1))
    yield set(visited_all), set(), result[0] if result[0] else []


def algo_iddfs(grid, start, target):
    visited_all = set()
    for limit in range(ROWS * COLS + 1):
        found = [None]
        stack = [([start], limit)]
        while stack:
            path, depth = stack.pop()
            node = path[-1]
            visited_all.add(node)
            if node == target:
                found[0] = path
                break
            if depth == 0:
                continue
            path_set = set(path)
            for n in reversed(get_neighbors(grid, node)):
                if n not in path_set:
                    stack.append((path + [n], depth - 1))
        yield set(visited_all), set(), None
        if found[0]:
            yield set(visited_all), set(), found[0]
            return
    yield set(visited_all), set(), []


def algo_bidirectional(grid, start, target):
    fwd_queue   = deque([[start]])
    fwd_visited = {start: [start]}
    bwd_queue   = deque([[target]])
    bwd_visited = {target: [target]}

    def all_exp():
        return set(fwd_visited) | set(bwd_visited)

    while fwd_queue or bwd_queue:
        if fwd_queue:
            path = fwd_queue.popleft()
            node = path[-1]
            for n in get_neighbors(grid, node):
                if n not in fwd_visited:
                    np = path + [n]
                    fwd_visited[n] = np
                    fwd_queue.append(np)
                    if n in bwd_visited:
                        back = list(reversed(bwd_visited[n]))[1:]
                        yield all_exp(), set(), np + back
                        return
        if bwd_queue:
            path = bwd_queue.popleft()
            node = path[-1]
            for n in get_neighbors(grid, node):
                if n not in bwd_visited:
                    np = path + [n]
                    bwd_visited[n] = np
                    bwd_queue.append(np)
                    if n in fwd_visited:
                        back = list(reversed(np))[1:]
                        yield all_exp(), set(), fwd_visited[n] + back
                        return
        yield all_exp(), set(), None
    yield all_exp(), set(), []


ALGORITHMS = {
    "BFS":           algo_bfs,
    "DFS":           algo_dfs,
    "UCS":           algo_ucs,
    "DLS":           algo_dls,
    "IDDFS":         algo_iddfs,
    "Bidirectional": algo_bidirectional,
}


# ─────────────────────────────────────────────────────────────
#  APPLICATION
# ─────────────────────────────────────────────────────────────

class PathfinderApp:

    MODE_NONE   = "none"
    MODE_WALL   = "wall"
    MODE_START  = "start"
    MODE_TARGET = "target"

    def __init__(self):
        self.grid      = copy.deepcopy(DEFAULT_GRID)
        self.start     = DEFAULT_START
        self.target    = DEFAULT_TARGET
        self.algorithm = "BFS"
        self.speed     = 60
        self.edit_mode = self.MODE_NONE

        self._running   = False
        self._timer     = None
        self._gen       = None
        self._explored  = set()
        self._frontier  = set()
        self._path      = None
        self._dragging  = False
        self._drag_erase = False
        self._status    = "Select algorithm  →  click  ▶ Run"

        self._build_figure()
        self._refresh()
        plt.show()

    # ── Figure setup ──────────────────────────────────────────

    def _build_figure(self):
        self.fig = plt.figure(figsize=(19, 9.5), facecolor=BG_DARK)
        self.fig.canvas.manager.set_window_title("AI Pathfinder — Uninformed Search Visualizer")

        # Grid axes (right portion)
        self.ax = self.fig.add_axes([0.26, 0.03, 0.73, 0.94])
        self.ax.set_facecolor(GRID_BG)
        self.ax.set_xlim(-0.5, COLS - 0.5)
        self.ax.set_ylim(ROWS - 0.5, -0.5)   # row 0 at top
        self.ax.set_aspect("equal")
        self.ax.tick_params(left=False, bottom=False,
                            labelleft=False, labelbottom=False)
        for spine in self.ax.spines.values():
            spine.set_edgecolor(ACC_CYAN)
            spine.set_linewidth(2.5)

        # Colormap: index → cell color
        cmap_colors = [
            COL_EMPTY,    # 0 – open
            COL_WALL,     # 1 – wall
            COL_START,    # 2 – start
            COL_TARGET,   # 3 – target
            COL_FRONTIER, # 4 – frontier
            COL_EXPLORED, # 5 – explored
            COL_PATH,     # 6 – path
        ]
        self.cmap = ListedColormap(cmap_colors)
        init_data = np.zeros((ROWS, COLS), dtype=float)
        self.im = self.ax.imshow(
            init_data, cmap=self.cmap,
            vmin=0, vmax=6, interpolation="nearest", aspect="equal",
        )

        # Grid lines
        for r in range(ROWS + 1):
            self.ax.axhline(r - 0.5, color=COL_GRID_LINE, linewidth=0.6, zorder=1)
        for c in range(COLS + 1):
            self.ax.axvline(c - 0.5, color=COL_GRID_LINE, linewidth=0.6, zorder=1)

        # Path overlay line
        self.path_line, = self.ax.plot(
            [], [], color=COL_PATH, linewidth=3,
            zorder=10, solid_capstyle="round",
        )

        # S / T labels
        self.start_txt  = self.ax.text(
            0, 0, "S", color="#050510",
            ha="center", va="center",
            fontsize=12, fontweight="bold", zorder=12,
        )
        self.target_txt = self.ax.text(
            0, 0, "T", color="#050510",
            ha="center", va="center",
            fontsize=12, fontweight="bold", zorder=12,
        )

        # Status bar inside grid
        self.status_txt = self.ax.text(
            COLS / 2 - 0.5, -0.42, self._status,
            color=ACC_CYAN, ha="center", va="bottom",
            fontsize=10, fontweight="bold", zorder=12,
        )

        self._build_panel()

        self.fig.canvas.mpl_connect("button_press_event",   self._on_press)
        self.fig.canvas.mpl_connect("motion_notify_event",  self._on_drag)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)

    def _build_panel(self):
        fig = self.fig

        # Title
        fig.text(0.005, 0.975, "🧭  AI PATHFINDER",
                 color=ACC_CYAN, fontsize=14, fontweight="bold",
                 va="top", transform=fig.transFigure)
        fig.text(0.005, 0.945, "Uninformed Search Visualizer",
                 color=TEXT_DIM, fontsize=9, va="top",
                 transform=fig.transFigure)

        # Algorithm selector
        ax_radio = fig.add_axes([0.005, 0.60, 0.235, 0.31],
                                facecolor=PANEL_BG)
        ax_radio.set_title("  ALGORITHM", color=ACC_ORANGE,
                            fontsize=9, fontweight="bold", pad=5, loc="left")
        self.radio = RadioButtons(ax_radio, list(ALGORITHMS.keys()),
                                  activecolor=ACC_ORANGE)
        for lbl in self.radio.labels:
            lbl.set_color(TEXT_BRIGHT)
            lbl.set_fontsize(10)
        self.radio.on_clicked(lambda v: setattr(self, "algorithm", v))

        # Speed slider
        ax_spd = fig.add_axes([0.012, 0.535, 0.21, 0.042], facecolor=PANEL_BG)
        ax_spd.set_title("  SPEED  (ms / step)", color=ACC_ORANGE,
                          fontsize=8, fontweight="bold", pad=3, loc="left")
        self.slider = Slider(ax_spd, "", 10, 400, valinit=self.speed,
                             color=ACC_ORANGE, initcolor=ACC_CYAN)
        self.slider.label.set_color(TEXT_DIM)
        self.slider.valtext.set_color(ACC_CYAN)
        self.slider.on_changed(lambda v: setattr(self, "speed", int(v)))

        # Control buttons
        def make_btn(rect, label, fc, tc="#050510"):
            ax = fig.add_axes(rect)
            b  = Button(ax, label, color=fc, hovercolor="#ffffff")
            b.label.set_fontsize(10)
            b.label.set_fontweight("bold")
            b.label.set_color(tc)
            return b

        self.btn_run   = make_btn([0.012, 0.480, 0.21, 0.047],
                                  "▶  Run",      ACC_GREEN)
        self.btn_stop  = make_btn([0.012, 0.428, 0.21, 0.047],
                                  "■  Stop",     ACC_PINK)
        self.btn_reset = make_btn([0.012, 0.376, 0.21, 0.047],
                                  "↺  Reset",    ACC_ORANGE)
        self.btn_clear = make_btn([0.012, 0.324, 0.21, 0.047],
                                  "⬛  Clear Grid", "#555577", TEXT_BRIGHT)

        self.btn_run.on_clicked(self._on_run)
        self.btn_stop.on_clicked(self._on_stop)
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_clear.on_clicked(self._on_clear)

        # Edit mode section
        fig.text(0.005, 0.318, "✏  EDIT MODE  (click grid after selecting)",
                 color=ACC_CYAN, fontsize=8, fontweight="bold",
                 transform=fig.transFigure)

        self.btn_wall   = make_btn([0.012, 0.260, 0.21, 0.047],
                                   "🧱  Wall  (L=add  R=erase)", COL_WALL)
        self.btn_estart = make_btn([0.012, 0.208, 0.21, 0.047],
                                   "🟢  Set Start",              COL_START)
        self.btn_etargt = make_btn([0.012, 0.156, 0.21, 0.047],
                                   "🔵  Set Target",             COL_TARGET)

        self.btn_wall.on_clicked(self._on_mode_wall)
        self.btn_estart.on_clicked(self._on_mode_start)
        self.btn_etargt.on_clicked(self._on_mode_target)

        # Legend
        legend_items = [
            (COL_START,    "Start  (S)"),
            (COL_TARGET,   "Target  (T)"),
            (COL_WALL,     "Wall / Obstacle"),
            (COL_FRONTIER, "Frontier (queue)"),
            (COL_EXPLORED, "Explored (visited)"),
            (COL_PATH,     "Final Path"),
        ]
        patches = [mpatches.Patch(color=c, label=l) for c, l in legend_items]
        ax_leg = fig.add_axes([0.000, 0.00, 0.255, 0.15], facecolor=PANEL_BG)
        ax_leg.axis("off")
        leg = ax_leg.legend(
            handles=patches, loc="center", fontsize=8.5,
            facecolor=PANEL_BG, edgecolor=ACC_CYAN,
            labelcolor=TEXT_BRIGHT, framealpha=1,
            ncol=1, handlelength=1.4,
        )
        leg.get_title().set_color(ACC_CYAN)

        fig.text(0.005, 0.003, f"DLS depth limit: {DLS_LIMIT}  |  "
                 "Moves: Up · Right · Down · Down-Right · Left · Up-Left",
                 color=TEXT_DIM, fontsize=7, transform=fig.transFigure)

    # ── Display ───────────────────────────────────────────────

    def _build_data(self, explored=None, frontier=None, path=None):
        explored = explored or set()
        frontier = frontier or set()
        path_set = set(path) if path else set()
        d = np.zeros((ROWS, COLS), dtype=float)
        for r in range(ROWS):
            for c in range(COLS):
                cell = (r, c)
                if self.grid[r][c] == -1:   d[r, c] = 1
                elif cell in path_set:      d[r, c] = 6
                elif cell in frontier:      d[r, c] = 4
                elif cell in explored:      d[r, c] = 5
        # start / target always visible
        d[self.start[0],  self.start[1]]  = 2
        d[self.target[0], self.target[1]] = 3
        return d

    def _refresh(self, explored=None, frontier=None, path=None):
        self.im.set_data(self._build_data(explored, frontier, path))
        self.im.set_clim(0, 6)

        if path and len(path) > 1:
            self.path_line.set_data([p[1] for p in path], [p[0] for p in path])
        else:
            self.path_line.set_data([], [])

        self.start_txt.set_position((self.start[1],  self.start[0]))
        self.target_txt.set_position((self.target[1], self.target[0]))
        self.status_txt.set_text(self._status)
        self.fig.canvas.draw_idle()

    def _set_status(self, msg, color=ACC_CYAN):
        self._status = msg
        self.status_txt.set_text(msg)
        self.status_txt.set_color(color)

    # ── Edit mode ─────────────────────────────────────────────

    def _reset_btn_borders(self):
        for btn in [self.btn_wall, self.btn_estart, self.btn_etargt]:
            for spine in btn.ax.spines.values():
                spine.set_linewidth(1)

    def _highlight_btn(self, btn):
        self._reset_btn_borders()
        for spine in btn.ax.spines.values():
            spine.set_linewidth(3)
            spine.set_edgecolor(TEXT_BRIGHT)

    def _on_mode_wall(self, _):
        if self._running: return
        self.edit_mode = self.MODE_WALL
        self._highlight_btn(self.btn_wall)
        self._set_status("EDIT WALL — left-click: add  |  right-click: erase  |  drag supported",
                         ACC_ORANGE)
        self._refresh()

    def _on_mode_start(self, _):
        if self._running: return
        self.edit_mode = self.MODE_START
        self._highlight_btn(self.btn_estart)
        self._set_status("EDIT START — click any open cell to move  S", ACC_GREEN)
        self._refresh()

    def _on_mode_target(self, _):
        if self._running: return
        self.edit_mode = self.MODE_TARGET
        self._highlight_btn(self.btn_etargt)
        self._set_status("EDIT TARGET — click any open cell to move  T", "#1e90ff")
        self._refresh()

    # ── Mouse events on grid ──────────────────────────────────

    def _grid_cell(self, event):
        if event.inaxes is not self.ax:
            return None
        c = int(round(event.xdata))
        r = int(round(event.ydata))
        if 0 <= r < ROWS and 0 <= c < COLS:
            return (r, c)
        return None

    def _apply_edit(self, cell, button):
        if cell is None or self._running:
            return
        r, c = cell

        if self.edit_mode == self.MODE_WALL:
            if cell in (self.start, self.target):
                return
            self.grid[r][c] = -1 if button != 3 else 0

        elif self.edit_mode == self.MODE_START:
            if self.grid[r][c] == -1 or cell == self.target:
                return
            self.start     = cell
            self.edit_mode = self.MODE_NONE
            self._reset_btn_borders()
            self._set_status(f"Start moved → {cell}.  Now click ▶ Run")

        elif self.edit_mode == self.MODE_TARGET:
            if self.grid[r][c] == -1 or cell == self.start:
                return
            self.target    = cell
            self.edit_mode = self.MODE_NONE
            self._reset_btn_borders()
            self._set_status(f"Target moved → {cell}.  Now click ▶ Run")

        self._refresh()

    def _on_press(self, event):
        if event.inaxes is not self.ax:
            return
        cell = self._grid_cell(event)
        if self.edit_mode == self.MODE_WALL and cell:
            self._dragging   = True
            self._drag_erase = (event.button == 3)
        self._apply_edit(cell, event.button)

    def _on_drag(self, event):
        if not self._dragging or self.edit_mode != self.MODE_WALL:
            return
        self._apply_edit(self._grid_cell(event), 3 if self._drag_erase else 1)

    def _on_release(self, _):
        self._dragging = False

    # ── Control buttons ───────────────────────────────────────

    def _on_run(self, _):
        if self._running:
            return
        self._stop_timer()
        self._explored = set()
        self._frontier = set()
        self._path     = None
        self.edit_mode = self.MODE_NONE
        self._reset_btn_borders()
        self._set_status(f"Running  {self.algorithm}…", ACC_CYAN)
        self._refresh()
        self._running = True
        self._gen     = ALGORITHMS[self.algorithm](
            self.grid, self.start, self.target
        )
        self._schedule_step()

    def _on_stop(self, _):
        self._running = False
        self._stop_timer()
        self._set_status("Stopped.  Press ↺ Reset to clear.", ACC_PINK)
        self._refresh(self._explored, self._frontier, self._path)

    def _on_reset(self, _):
        self._running = False
        self._stop_timer()
        self._gen      = None
        self._explored = set()
        self._frontier = set()
        self._path     = None
        self.edit_mode = self.MODE_NONE
        self._reset_btn_borders()
        self._set_status("Select algorithm  →  click  ▶ Run")
        self._refresh()

    def _on_clear(self, _):
        if self._running:
            return
        self.grid = [[0] * COLS for _ in range(ROWS)]
        self._on_reset(None)

    # ── Animation ─────────────────────────────────────────────

    def _stop_timer(self):
        if self._timer is not None:
            try:
                self._timer.stop()
            except Exception:
                pass
            self._timer = None

    def _schedule_step(self):
        self._timer = self.fig.canvas.new_timer(interval=self.speed)
        self._timer.add_callback(self._step)
        self._timer.single_shot = True
        self._timer.start()

    def _step(self):
        if not self._running or self._gen is None:
            return
        try:
            explored, frontier, path = next(self._gen)
            self._explored = explored
            self._frontier = frontier
            self._path     = path

            if path is not None:
                self._running = False
                if path:
                    self._set_status(
                        f"✓ {self.algorithm} — path found!   "
                        f"Length: {len(path)-1} steps   "
                        f"Explored: {len(explored)} nodes",
                        ACC_GREEN,
                    )
                    self._refresh(explored, set(), path)
                else:
                    self._set_status(
                        f"✗ {self.algorithm} — no path exists   "
                        f"(explored {len(explored)} nodes)",
                        ACC_PINK,
                    )
                    self._refresh(explored)
            else:
                self._refresh(explored, frontier)
                if self._running:
                    self._schedule_step()

        except StopIteration:
            self._running = False


# ─────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    PathfinderApp()