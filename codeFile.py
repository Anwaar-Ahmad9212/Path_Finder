"""
=============================================================
  AI Pathfinder - Uninformed Search Visualizer
  Library : Matplotlib + NumPy   (pip install matplotlib numpy)
  Run     : python pathfinder.py
=============================================================

  ALGORITHMS              EDIT MODE
  ---------------------   ------------------------------------------
  BFS                     Wall   - left-click=add, right-click=erase
  DFS                     Start  - click any open cell to move S
  UCS                     Target - click any open cell to move T
  DLS  (depth limit=20)   Clear  - wipe all walls
  IDDFS
  Bidirectional

  Movement order (assignment spec - clockwise + main diagonal):
    1.Up  2.Right  3.Down  4.Down-Right  5.Left  6.Up-Left
    Top-Right and Bottom-Left diagonals are EXCLUDED.
=============================================================
"""

# -- pick the best available interactive backend
import matplotlib
for _b in ["TkAgg", "Qt5Agg", "GTK3Agg", "WXAgg", "MacOSX"]:
    try:
        matplotlib.use(_b)
        import matplotlib.pyplot as _pt
        _pt.figure(); _pt.close("all")
        break
    except Exception:
        pass

import math, heapq, copy
from collections import deque

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.widgets import Button, RadioButtons, Slider
from matplotlib.colors import ListedColormap
import numpy as np

# 
#  MAZE   0=open   -1=wall
#
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

# Strict movement order per assignment (clockwise + main diagonal only)
MOVES = [
    (-1,  0),   # 1. Up
    ( 0, +1),   # 2. Right
    (+1,  0),   # 3. Down
    (+1, +1),   # 4. Down-Right  (main diagonal)
    ( 0, -1),   # 5. Left
    (-1, -1),   # 6. Up-Left
    # Top-Right (-1,+1) and Bottom-Left (+1,-1) are EXCLUDED
]

# ---------------------------------------------------------------
#  VIVID COLOR PALETTE
# ---------------------------------------------------------------
C = {
    # cell types
    "empty":    "#0e0e1f",
    "wall":     "#ff2d55",   # hot red
    "start":    "#00ff88",   # neon green
    "target":   "#00aaff",   # electric blue
    "frontier": "#ff9f00",   # bright amber
    "explored": "#8b5cf6",   # vivid violet
    "path":     "#ffe600",   # laser yellow
    "grid_ln":  "#1c1c3a",   # subtle grid lines
    # UI
    "bg":       "#07071a",   # very dark navy
    "panel":    "#0f0f2d",   # panel bg
    "accent":   "#00d2ff",   # cyan
    "txt":      "#e0e0ff",   # bright white-blue
    "dim":      "#5555aa",   # muted purple
    # buttons
    "btn_run":   "#00c853",  # green
    "btn_stop":  "#ff1744",  # red
    "btn_reset": "#ff6d00",  # orange
    "btn_clear": "#455a64",  # grey-blue
    "btn_wall":  "#ff2d55",  # hot red
    "btn_start": "#00ff88",  # neon green
    "btn_tgt":   "#00aaff",  # electric blue
}

# Colormap - ordered by index
_CMAP_KEYS = ["empty", "wall", "start", "target", "frontier", "explored", "path"]
IDX = {k: i for i, k in enumerate(_CMAP_KEYS)}
CMAP = ListedColormap([C[k] for k in _CMAP_KEYS])


# ---------------------------------------------------------------
#  HELPERS
# ---------------------------------------------------------------
def neighbors(grid, node):
    r, c = node
    return [
        (r+dr, c+dc)
        for dr, dc in MOVES
        if 0 <= r+dr < ROWS and 0 <= c+dc < COLS and grid[r+dr][c+dc] != -1
    ]


# ---------------------------------------------------------------
#  SEARCH ALGORITHMS  - generators
#  Yield: (explored_set, frontier_set, path_or_None)
#    path=None  - still searching
#    path=[]    - no path exists
#    path=[...] - done, path found
# ---------------------------------------------------------------

def algo_bfs(grid, S, T):
    queue = deque([[S]])
    vis   = {S}
    while queue:
        path = queue.popleft()
        node = path[-1]
        front = set()
        for n in neighbors(grid, node):
            if n not in vis:
                vis.add(n)
                np_ = path + [n]
                if n == T:
                    yield set(vis), set(), np_
                    return
                front.add(n)
                queue.append(np_)
        yield set(vis), front, None
    yield set(vis), set(), []


def algo_dfs(grid, S, T):
    stack = [[S]]
    vis   = {S}
    while stack:
        path = stack.pop()
        node = path[-1]
        if node == T:
            yield set(vis), set(), path
            return
        front = set()
        for n in reversed(neighbors(grid, node)):
            if n not in vis:
                vis.add(n)
                front.add(n)
                stack.append(path + [n])
        yield set(vis), front, None
    yield set(vis), set(), []


def algo_ucs(grid, S, T):
    ctr  = 0
    heap = [(0.0, ctr, [S])]
    vis  = {}
    while heap:
        cost, _, path = heapq.heappop(heap)
        node = path[-1]
        if node in vis:
            continue
        vis[node] = cost
        if node == T:
            yield set(vis), set(), path
            return
        front = set()
        for n in neighbors(grid, node):
            if n not in vis:
                dr, dc    = abs(n[0]-node[0]), abs(n[1]-node[1])
                step_cost = math.sqrt(2) if (dr == 1 and dc == 1) else 1.0
                ctr      += 1
                heapq.heappush(heap, (cost + step_cost, ctr, path + [n]))
                front.add(n)
        yield set(vis), front, None
    yield set(vis), set(), []


def algo_dls(grid, S, T):
    vis    = set()
    result = [None]
    stack  = [([S], DLS_LIMIT)]
    while stack:
        path, depth = stack.pop()
        node = path[-1]
        vis.add(node)
        yield set(vis), set(), None        # one frame per expansion
        if node == T:
            result[0] = path
            break
        if depth == 0:
            continue
        path_set = set(path)
        for n in reversed(neighbors(grid, node)):
            if n not in path_set:
                stack.append((path + [n], depth - 1))
    yield set(vis), set(), result[0] or []


def algo_iddfs(grid, S, T):
    vis_all = set()
    for limit in range(ROWS * COLS + 1):
        found = [None]
        stack = [([S], limit)]
        while stack:
            path, depth = stack.pop()
            node = path[-1]
            vis_all.add(node)
            if node == T:
                found[0] = path
                break
            if depth == 0:
                continue
            path_set = set(path)
            for n in reversed(neighbors(grid, node)):
                if n not in path_set:
                    stack.append((path + [n], depth - 1))
        yield set(vis_all), set(), None    # one frame per depth pass
        if found[0]:
            yield set(vis_all), set(), found[0]
            return
    yield set(vis_all), set(), []


def algo_bidir(grid, S, T):
    fq = deque([[S]])
    bq = deque([[T]])
    fv = {S: [S]}   # node -> path from S
    bv = {T: [T]}   # node -> path from T

    def explored():
        return set(fv) | set(bv)

    while fq or bq:
        if fq:
            path = fq.popleft()
            node = path[-1]
            for n in neighbors(grid, node):
                if n not in fv:
                    np_ = path + [n]
                    fv[n] = np_
                    fq.append(np_)
                    if n in bv:
                        back = list(reversed(bv[n]))[1:]
                        yield explored(), set(), np_ + back
                        return
        if bq:
            path = bq.popleft()
            node = path[-1]
            for n in neighbors(grid, node):
                if n not in bv:
                    np_ = path + [n]
                    bv[n] = np_
                    bq.append(np_)
                    if n in fv:
                        back = list(reversed(np_))[1:]
                        yield explored(), set(), fv[n] + back
                        return
        yield explored(), set(), None
    yield explored(), set(), []


ALGOS = {
    "BFS":           algo_bfs,
    "DFS":           algo_dfs,
    "UCS":           algo_ucs,
    "DLS":           algo_dls,
    "IDDFS":         algo_iddfs,
    "Bidirectional": algo_bidir,
}


# ---------------------------------------------------------------
#  APPLICATION
# ---------------------------------------------------------------

class App:
    # edit modes
    NONE   = "none"
    WALL   = "wall"
    START  = "start"
    TARGET = "target"

    # Panel sits in figure-fraction x: [0, PW]
    # Grid sits in figure-fraction x: [PW, 1]
    PW = 0.21   # panel width fraction

    def __init__(self):
        self.grid   = copy.deepcopy(DEFAULT_GRID)
        self.start  = DEFAULT_START
        self.target = DEFAULT_TARGET
        self.algo   = "BFS"
        self.speed  = 60         # ms between animation steps
        self.mode   = self.NONE

        self._running  = False
        self._timer    = None
        self._gen      = None
        self._exp      = set()
        self._front    = set()
        self._path     = None
        self._dragging = False
        self._drag_rm  = False

        self._build()
        self._redraw()
        plt.show()

    # -- figure setup --

    def _build(self):
        self.fig = plt.figure(figsize=(21, 10.5), facecolor=C["bg"])
        try:
            self.fig.canvas.manager.set_window_title(
                "Uninformed Search Visualizer")
        except Exception:
            pass

        self._build_panel()
        self._build_grid()

        self.fig.canvas.mpl_connect("button_press_event",   self._press)
        self.fig.canvas.mpl_connect("motion_notify_event",  self._motion)
        self.fig.canvas.mpl_connect("button_release_event", self._release)

    # -- panel (left strip) ────────────────────────────────
    # All axes positioned with absolute figure fractions [left, bottom, w, h]

    def _panel_ax(self, bottom, height, fc=None):
        """Create a clean axes in the left panel strip."""
        pad_l = 0.01
        pad_r = 0.01
        ax = self.fig.add_axes([pad_l, bottom, self.PW - pad_l - pad_r, height],
                               facecolor=fc or C["panel"])
        ax.tick_params(left=False, bottom=False,
                       labelleft=False, labelbottom=False)
        for sp in ax.spines.values():
            sp.set_visible(False)
        return ax

    def _build_panel(self):
        PW = self.PW

        # -- Title────────────────────────────────────────────
        ax = self._panel_ax(0.93, 0.065, fc=C["bg"])
        ax.text(0.5, 0.70, "PATHFINDER",
                color=C["accent"], fontsize=13, fontweight="bold",
                ha="center", va="center", transform=ax.transAxes)
        ax.text(0.5, 0.18, "Uninformed Search Visualizer",
                color=C["dim"], fontsize=8,
                ha="center", va="center", transform=ax.transAxes)

        # -- Algorithm radio────────────────────────────────
        ax_r = self._panel_ax(0.59, 0.32, fc=C["panel"])
        ax_r.text(0.04, 0.98, "ALGORITHM", color=C["accent"],
                  fontsize=8, fontweight="bold", va="top",
                  transform=ax_r.transAxes)
        _n = len(ALGOS)
        self.radio = RadioButtons(
            ax_r,
            labels=list(ALGOS.keys()),
            active=0,
            label_props={
                "color":    [C["txt"]] * _n,
                "fontsize": [9.5]      * _n,
            },
            radio_props={
                "facecolor": [C["frontier"]] + [C["dim"]] * (_n - 1),
                "edgecolor": [C["txt"]] * _n,
                "s":         [64] * _n,
            },
        )
        self.radio.on_clicked(lambda v: setattr(self, "algo", v))

        # -- Speed slider
        ax_s = self._panel_ax(0.515, 0.055, fc=C["panel"])
        ax_s.text(0.04, 0.97, "SPEED  (ms / step)", color=C["accent"],
                  fontsize=7.5, fontweight="bold", va="top",
                  transform=ax_s.transAxes)
        self.slider = Slider(
            ax_s, "", 10, 500,
            valinit=self.speed,
            color=C["frontier"],
            initcolor=C["accent"],
            track_color=C["bg"],
        )
        self.slider.label.set_visible(False)
        self.slider.valtext.set_color(C["accent"])
        self.slider.valtext.set_fontsize(8)
        self.slider.on_changed(lambda v: setattr(self, "speed", int(v)))

        # -- Control buttons
        btn_h  = 0.052
        btn_gap = 0.006

        def mkbtn(bottom, label, fc, tc="#07071a"):
            a = self._panel_ax(bottom, btn_h, fc=fc)
            b = Button(a, label, color=fc, hovercolor="#ffffff")
            b.label.set_fontsize(9.5)
            b.label.set_fontweight("bold")
            b.label.set_color(tc)
            return b

        b0 = 0.515 - btn_h - btn_gap
        self.btn_run   = mkbtn(b0,                            "Run",           C["btn_run"])
        self.btn_stop  = mkbtn(b0 -   (btn_h+btn_gap),       "Stop",          C["btn_stop"])
        self.btn_reset = mkbtn(b0 - 2*(btn_h+btn_gap),       "Reset",         C["btn_reset"])
        self.btn_clear = mkbtn(b0 - 3*(btn_h+btn_gap),       "Clear Grid",    C["btn_clear"], C["txt"])

        self.btn_run.on_clicked(self._run)
        self.btn_stop.on_clicked(self._stop)
        self.btn_reset.on_clicked(self._reset)
        self.btn_clear.on_clicked(self._clear)

        # -- Edit mode header
        edit_top = b0 - 4*(btn_h+btn_gap)
        ax_eh = self._panel_ax(edit_top, 0.030, fc=C["bg"])
        ax_eh.text(0.04, 0.55, "EDIT MODE  (click button -> click grid)",
                   color=C["accent"], fontsize=7.5, fontweight="bold",
                   va="center", transform=ax_eh.transAxes)

        # -- Edit buttons
        e0 = edit_top - btn_h - btn_gap
        self.btn_wall   = mkbtn(e0,                      "Wall  (L=add   R=erase)", C["btn_wall"])
        self.btn_estart = mkbtn(e0 -   (btn_h+btn_gap), "Set Start",               C["btn_start"])
        self.btn_etargt = mkbtn(e0 - 2*(btn_h+btn_gap), "Set Target",              C["btn_tgt"])

        self.btn_wall.on_clicked(self._mode_wall)
        self.btn_estart.on_clicked(self._mode_start)
        self.btn_etargt.on_clicked(self._mode_target)

        # -- Legend
        leg_bot = e0 - 3*(btn_h+btn_gap) - 0.005
        ax_leg  = self._panel_ax(leg_bot, 0.175, fc=C["panel"])
        ax_leg.text(0.04, 0.97, "LEGEND", color=C["accent"],
                    fontsize=8, fontweight="bold", va="top",
                    transform=ax_leg.transAxes)
        legend_items = [
            (C["start"],    "Start  (S)"),
            (C["target"],   "Target  (T)"),
            (C["wall"],     "Wall / Obstacle"),
            (C["frontier"], "Frontier  (queue)"),
            (C["explored"], "Explored  (visited)"),
            (C["path"],     "Final Path"),
        ]
        patches = [mpatches.Patch(color=col, label=lbl)
                   for col, lbl in legend_items]
        ax_leg.legend(
            handles=patches,
            loc="upper left",
            bbox_to_anchor=(0.01, 0.88),
            fontsize=8,
            facecolor=C["panel"],
            edgecolor=C["accent"],
            labelcolor=C["txt"],
            framealpha=1,
            handlelength=1.4,
            handleheight=1.1,
        )

        # -- Footer note
        ax_ft = self._panel_ax(0.01, 0.025, fc=C["bg"])
        ax_ft.text(
            0.04, 0.5,
            f"DLS depth limit={DLS_LIMIT}  |  Moves: UP RIGHT DOWN DIAG-DOWN LEFT DIAG-UP",
            color=C["dim"], fontsize=7, va="center",
            transform=ax_ft.transAxes,
        )

    # -- grid canvas

    def _build_grid(self):
        margin = 0.02
        self.gax = self.fig.add_axes(
            [self.PW + margin, 0.03, 1.0 - self.PW - margin - 0.01, 0.94]
        )
        self.gax.set_facecolor(C["bg"])
        self.gax.set_xlim(-0.5, COLS - 0.5)
        self.gax.set_ylim(ROWS - 0.5, -0.5)   # row 0 at top
        self.gax.set_aspect("equal")
        self.gax.tick_params(left=False, bottom=False,
                             labelleft=False, labelbottom=False)
        for sp in self.gax.spines.values():
            sp.set_edgecolor(C["accent"])
            sp.set_linewidth(2.5)

        # Cell colours via imshow
        data = np.zeros((ROWS, COLS), dtype=float)
        self.im = self.gax.imshow(
            data, cmap=CMAP, vmin=0, vmax=6,
            interpolation="nearest", aspect="equal",
            extent=[-0.5, COLS-0.5, ROWS-0.5, -0.5],
        )

        # Subtle grid lines
        for r in range(ROWS + 1):
            self.gax.axhline(r - 0.5, color=C["grid_ln"], lw=0.8, zorder=1)
        for c in range(COLS + 1):
            self.gax.axvline(c - 0.5, color=C["grid_ln"], lw=0.8, zorder=1)

        # Path overlay (drawn on top of cells)
        self.path_line, = self.gax.plot(
            [], [], color=C["path"], lw=3.5,
            solid_capstyle="round", zorder=10,
        )

        # S / T text labels
        self.s_lbl = self.gax.text(
            0, 0, "S", color=C["bg"],
            ha="center", va="center",
            fontsize=13, fontweight="bold", zorder=12,
        )
        self.t_lbl = self.gax.text(
            0, 0, "T", color=C["bg"],
            ha="center", va="center",
            fontsize=13, fontweight="bold", zorder=12,
        )

        # Status bar at bottom of grid
        self.status_txt = self.gax.text(
            COLS / 2 - 0.5, ROWS - 0.25,
            "Select algorithm  ->  click  Run",
            color=C["accent"], ha="center", va="bottom",
            fontsize=10, fontweight="bold", zorder=12,
        )

    # ── redraw ────────────────────────────────────────────────

    def _make_data(self, exp=None, front=None, path=None):
        exp   = exp   or set()
        front = front or set()
        pset  = set(path) if path else set()
        d = np.zeros((ROWS, COLS), dtype=float)
        for r in range(ROWS):
            for c in range(COLS):
                cell = (r, c)
                if   self.grid[r][c] == -1: d[r, c] = IDX["wall"]
                elif cell in pset:          d[r, c] = IDX["path"]
                elif cell in front:         d[r, c] = IDX["frontier"]
                elif cell in exp:           d[r, c] = IDX["explored"]
        d[self.start[0],  self.start[1]]  = IDX["start"]
        d[self.target[0], self.target[1]] = IDX["target"]
        return d

    def _redraw(self, exp=None, front=None, path=None):
        self.im.set_data(self._make_data(exp, front, path))

        if path and len(path) > 1:
            self.path_line.set_data([p[1] for p in path],
                                    [p[0] for p in path])
        else:
            self.path_line.set_data([], [])

        self.s_lbl.set_position((self.start[1],  self.start[0]))
        self.t_lbl.set_position((self.target[1], self.target[0]))
        self.fig.canvas.draw_idle()

    def _set_status(self, msg, color=None):
        self.status_txt.set_text(msg)
        self.status_txt.set_color(color or C["accent"])

    # ── edit mode buttons ─────────────────────────────────────

    def _highlight_edit(self, active=None):
        """Put a white border around the active edit button."""
        for btn in (self.btn_wall, self.btn_estart, self.btn_etargt):
            for sp in btn.ax.spines.values():
                sp.set_visible(False)
        if active:
            for sp in active.ax.spines.values():
                sp.set_visible(True)
                sp.set_linewidth(3)
                sp.set_edgecolor("#ffffff")
        self.fig.canvas.draw_idle()

    def _mode_wall(self, _):
        if self._running: return
        self.mode = self.WALL
        self._highlight_edit(self.btn_wall)
        self._set_status(
            "WALL — left-click to add  |  right-click to erase  |  drag to paint",
            C["frontier"])
        self._redraw(self._exp, self._front, self._path)

    def _mode_start(self, _):
        if self._running: return
        self.mode = self.START
        self._highlight_edit(self.btn_estart)
        self._set_status("START — click any open cell to move  S", C["start"])
        self._redraw(self._exp, self._front, self._path)

    def _mode_target(self, _):
        if self._running: return
        self.mode = self.TARGET
        self._highlight_edit(self.btn_etargt)
        self._set_status("TARGET — click any open cell to move  T", C["target"])
        self._redraw(self._exp, self._front, self._path)

    # ── mouse on grid ─────────────────────────────────────────

    def _cell_at(self, event):
        """Convert a mouse event to (row, col) or None."""
        if event.inaxes is not self.gax:
            return None
        c = int(round(event.xdata))
        r = int(round(event.ydata))
        return (r, c) if (0 <= r < ROWS and 0 <= c < COLS) else None

    def _apply_edit(self, cell, mouse_btn):
        if cell is None or self._running:
            return
        r, c = cell

        if self.mode == self.WALL:
            if cell in (self.start, self.target):
                return
            self.grid[r][c] = -1 if mouse_btn != 3 else 0
            self._redraw(self._exp, self._front, self._path)

        elif self.mode == self.START:
            if self.grid[r][c] == -1 or cell == self.target:
                return
            self.start = cell
            self.mode  = self.NONE
            self._highlight_edit(None)
            self._set_status(f"Start moved -> {cell}   Click  Run to search")
            self._redraw()

        elif self.mode == self.TARGET:
            if self.grid[r][c] == -1 or cell == self.start:
                return
            self.target = cell
            self.mode   = self.NONE
            self._highlight_edit(None)
            self._set_status(f"Target moved -> {cell}   Click  Run to search")
            self._redraw()

    def _press(self, event):
        if event.inaxes is not self.gax:
            return
        cell = self._cell_at(event)
        if self.mode == self.WALL and cell:
            self._dragging = True
            self._drag_rm  = (event.button == 3)
        self._apply_edit(cell, event.button)

    def _motion(self, event):
        if not self._dragging or self.mode != self.WALL:
            return
        self._apply_edit(self._cell_at(event), 3 if self._drag_rm else 1)

    def _release(self, _):
        self._dragging = False

    # ── control buttons ───────────────────────────────────────

    def _run(self, _):
        if self._running:
            return
        self._kill_timer()
        self._exp   = set()
        self._front = set()
        self._path  = None
        self.mode   = self.NONE
        self._highlight_edit(None)
        self._set_status(f"Running  {self.algo} …", C["accent"])
        self._redraw()
        self._running = True
        self._gen = ALGOS[self.algo](self.grid, self.start, self.target)
        self._step()

    def _stop(self, _):
        self._running = False
        self._kill_timer()
        self._set_status("Stopped - press  Reset to clear", C["btn_stop"])
        self._redraw(self._exp, self._front, self._path)

    def _reset(self, _):
        self._running = False
        self._kill_timer()
        self._gen   = None
        self._exp   = set()
        self._front = set()
        self._path  = None
        self.mode   = self.NONE
        self._highlight_edit(None)
        self._set_status("Select algorithm  ->  click  Run")
        self._redraw()

    def _clear(self, _):
        if self._running:
            return
        self.grid = [[0] * COLS for _ in range(ROWS)]
        self._reset(None)

    # ── animation timer ───────────────────────────────────────

    def _kill_timer(self):
        if self._timer is not None:
            try:
                self._timer.stop()
            except Exception:
                pass
            self._timer = None

    def _step(self):
        """Advance the generator one step, then reschedule."""
        if not self._running or self._gen is None:
            return
        try:
            exp, front, path = next(self._gen)
            self._exp   = exp
            self._front = front
            self._path  = path

            if path is not None:
                # algorithm finished
                self._running = False
                if path:
                    self._set_status(
                        f"[OK]  {self.algo}  -  path found!     "
                        f"Length: {len(path)-1} steps     "
                        f"Explored: {len(exp)} nodes",
                        C["start"],
                    )
                    self._redraw(exp, set(), path)
                else:
                    self._set_status(
                        f"[FAIL]  {self.algo}  -  no path exists     "
                        f"({len(exp)} nodes explored)",
                        C["btn_stop"],
                    )
                    self._redraw(exp)
            else:
                self._redraw(exp, front)
                if self._running:
                    self._timer = self.fig.canvas.new_timer(interval=self.speed)
                    self._timer.single_shot = True
                    self._timer.add_callback(self._step)
                    self._timer.start()

        except StopIteration:
            self._running = False


# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    App()