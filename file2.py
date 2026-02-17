"""
AI Pathfinder - Uninformed Search Visualization
Implements: BFS, DFS, UCS, DLS, IDDFS, Bidirectional Search
Movement Order (Clockwise + Main Diagonal only):
  1. Up  2. Right  3. Bottom  4. Bottom-Right  5. Left  6. Top-Left
"""

import tkinter as tk
from tkinter import ttk, messagebox
import heapq
from collections import deque

# ─────────────────────────────────────────────
#  HARDCODED MAZE
#  0  = open cell
# -1  = wall / obstacle
#  S  = start (row 1, col 1)
#  T  = target (row 10, col 18)
# ─────────────────────────────────────────────
GRID = [
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

START  = (1, 1)
TARGET = (10, 18)

ROWS = len(GRID)
COLS = len(GRID[0])

# Movement order (clockwise + main diagonal only - as per assignment)
# Up, Right, Bottom, Bottom-Right (diagonal), Left, Top-Left (diagonal)
MOVES = [(-1, 0), (0, 1), (1, 0), (1, 1), (0, -1), (-1, -1)]

# ─────────────────────────────────────────────
#  COLORS
# ─────────────────────────────────────────────
COLOR_EMPTY    = "#1e1e2e"   # dark background
COLOR_WALL     = "#45475a"   # obstacle
COLOR_START    = "#a6e3a1"   # green - start
COLOR_TARGET   = "#89b4fa"   # blue  - target
COLOR_FRONTIER = "#f9e2af"   # yellow - in queue/stack
COLOR_EXPLORED = "#6c7086"   # grey  - visited
COLOR_PATH     = "#f38ba8"   # red/pink - final path
COLOR_BG       = "#181825"
COLOR_PANEL    = "#1e1e2e"
COLOR_TEXT     = "#cdd6f4"
COLOR_ACCENT   = "#cba6f7"
COLOR_BTN      = "#313244"
COLOR_BTN_ACT  = "#cba6f7"

CELL_SIZE = 42
CELL_PAD  = 2

DLS_LIMIT = 20  # depth limit for DLS (set < 18 to show "no path found" worst case)


# ─────────────────────────────────────────────
#  SEARCH ALGORITHMS
#  Each yields: (frontier_set, explored_set, path_or_None)
#  so we can animate step-by-step.
# ─────────────────────────────────────────────

def bfs():
    """Breadth-First Search"""
    queue = deque([[START]])
    visited = {START}
    while queue:
        path = queue.popleft()
        node = path[-1]
        frontier = set()
        for next_path in [path + [n] for n in get_neighbors(node) if n not in visited]:
            frontier.add(next_path[-1])
        yield set(visited), frontier, None
        for next_path in [path + [n] for n in get_neighbors(node) if n not in visited]:
            n = next_path[-1]
            if n == TARGET:
                yield set(visited), set(), next_path
                return
            visited.add(n)
            queue.append(next_path)
    yield set(visited), set(), []


def dfs():
    """Depth-First Search (iterative)"""
    stack = [[START]]
    visited = {START}
    while stack:
        path = stack.pop()
        node = path[-1]
        if node == TARGET:
            yield set(visited), set(), path
            return
        frontier = set()
        for n in reversed(get_neighbors(node)):
            if n not in visited:
                frontier.add(n)
                visited.add(n)
                stack.append(path + [n])
        yield set(visited), frontier, None
    yield set(visited), set(), []


def ucs():
    """Uniform-Cost Search (cost = 1 straight, sqrt(2) diagonal)"""
    import math
    heap = [(0, [START])]
    visited = {}
    while heap:
        cost, path = heapq.heappop(heap)
        node = path[-1]
        if node in visited:
            continue
        visited[node] = cost
        if node == TARGET:
            yield set(visited), set(), path
            return
        frontier = set()
        for n in get_neighbors(node):
            if n not in visited:
                dr = abs(n[0] - node[0])
                dc = abs(n[1] - node[1])
                step_cost = math.sqrt(2) if (dr == 1 and dc == 1) else 1
                heapq.heappush(heap, (cost + step_cost, path + [n]))
                frontier.add(n)
        yield set(visited), frontier, None
    yield set(visited), set(), []


def dls():
    """Depth-Limited Search"""
    limit = DLS_LIMIT
    result = []
    visited_all = set()

    def _dls(path, depth):
        node = path[-1]
        visited_all.add(node)
        if node == TARGET:
            result.append(path[:])
            return True
        if depth == 0:
            return False
        for n in get_neighbors(node):
            if n not in set(path):
                path.append(n)
                if _dls(path, depth - 1):
                    return True
                path.pop()
        return False

    _dls([START], limit)
    yield set(visited_all), set(), result[0] if result else []


def iddfs():
    """Iterative Deepening DFS"""
    visited_all = set()
    for depth in range(ROWS * COLS):
        found = []
        visited_this = set()

        def _dls_id(path, d):
            node = path[-1]
            visited_this.add(node)
            visited_all.add(node)
            if node == TARGET:
                found.append(path[:])
                return True
            if d == 0:
                return False
            for n in get_neighbors(node):
                if n not in set(path):
                    path.append(n)
                    if _dls_id(path, d - 1):
                        return True
                    path.pop()
            return False

        _dls_id([START], depth)
        yield set(visited_all), set(), None
        if found:
            yield set(visited_all), set(), found[0]
            return
    yield set(visited_all), set(), []


def bidirectional():
    """Bidirectional BFS"""
    front_queue = deque([[START]])
    back_queue  = deque([[TARGET]])
    front_visited = {START: [START]}
    back_visited  = {TARGET: [TARGET]}

    while front_queue or back_queue:
        # Expand forward
        if front_queue:
            path = front_queue.popleft()
            node = path[-1]
            for n in get_neighbors(node):
                if n not in front_visited:
                    new_path = path + [n]
                    front_visited[n] = new_path
                    front_queue.append(new_path)
                    if n in back_visited:
                        full = new_path + list(reversed(back_visited[n][1:]))
                        yield set(front_visited) | set(back_visited), set(), full
                        return
        # Expand backward
        if back_queue:
            path = back_queue.popleft()
            node = path[-1]
            for n in get_neighbors(node):
                if n not in back_visited:
                    new_path = path + [n]
                    back_visited[n] = new_path
                    back_queue.append(new_path)
                    if n in front_visited:
                        full = front_visited[n] + list(reversed(new_path[1:]))
                        yield set(front_visited) | set(back_visited), set(), full
                        return
        yield set(front_visited) | set(back_visited), set(), None
    yield set(front_visited) | set(back_visited), set(), []


def get_neighbors(node):
    r, c = node
    result = []
    for dr, dc in MOVES:
        nr, nc = r + dr, c + dc
        if 0 <= nr < ROWS and 0 <= nc < COLS and GRID[nr][nc] != -1:
            result.append((nr, nc))
    return result


# ─────────────────────────────────────────────
#  GUI
# ─────────────────────────────────────────────

class PathfinderApp:
    def __init__(self, root):
        self.root = root
        self.root.title("AI Pathfinder — Uninformed Search Visualizer")
        self.root.configure(bg=COLOR_BG)
        self.root.resizable(False, False)

        self.algorithm = tk.StringVar(value="BFS")
        self.speed_ms  = tk.IntVar(value=80)
        self.status    = tk.StringVar(value="Select an algorithm and press  ▶  Run")
        self._job      = None       # after() handle
        self._gen      = None       # current generator
        self._running  = False
        self._path_cells = []

        self._build_ui()
        self._draw_grid()

    # ── UI Construction ──────────────────────

    def _build_ui(self):
        # Left panel
        panel = tk.Frame(self.root, bg=COLOR_PANEL, width=220)
        panel.pack(side=tk.LEFT, fill=tk.Y, padx=0, pady=0)
        panel.pack_propagate(False)

        tk.Label(panel, text="🧭 AI Pathfinder", bg=COLOR_PANEL, fg=COLOR_ACCENT,
                 font=("Courier New", 16, "bold")).pack(pady=(22, 4))
        tk.Label(panel, text="Uninformed Search", bg=COLOR_PANEL, fg=COLOR_TEXT,
                 font=("Courier New", 9)).pack(pady=(0, 18))

        # Algorithm selector
        tk.Label(panel, text="ALGORITHM", bg=COLOR_PANEL, fg=COLOR_ACCENT,
                 font=("Courier New", 9, "bold")).pack(anchor="w", padx=18)

        algos = ["BFS", "DFS", "UCS", "DLS", "IDDFS", "Bidirectional"]
        for alg in algos:
            rb = tk.Radiobutton(panel, text=alg, variable=self.algorithm, value=alg,
                                bg=COLOR_PANEL, fg=COLOR_TEXT, selectcolor=COLOR_BG,
                                activebackground=COLOR_PANEL, activeforeground=COLOR_ACCENT,
                                font=("Courier New", 11), indicatoron=True)
            rb.pack(anchor="w", padx=24, pady=2)

        # Speed slider
        tk.Label(panel, text="\nSPEED (ms/step)", bg=COLOR_PANEL, fg=COLOR_ACCENT,
                 font=("Courier New", 9, "bold")).pack(anchor="w", padx=18)
        slider = tk.Scale(panel, from_=10, to=500, orient=tk.HORIZONTAL,
                          variable=self.speed_ms, bg=COLOR_PANEL, fg=COLOR_TEXT,
                          troughcolor=COLOR_BG, highlightthickness=0,
                          activebackground=COLOR_ACCENT, font=("Courier New", 9))
        slider.pack(fill=tk.X, padx=18, pady=4)

        # Buttons
        btn_cfg = dict(bg=COLOR_BTN, fg=COLOR_TEXT, activebackground=COLOR_ACCENT,
                       activeforeground=COLOR_BG, font=("Courier New", 11, "bold"),
                       relief=tk.FLAT, cursor="hand2", pady=8)

        tk.Button(panel, text="▶  Run", command=self._start, **btn_cfg).pack(
            fill=tk.X, padx=18, pady=(18, 4))
        tk.Button(panel, text="■  Stop", command=self._stop, **btn_cfg).pack(
            fill=tk.X, padx=18, pady=4)
        tk.Button(panel, text="↺  Reset", command=self._reset, **btn_cfg).pack(
            fill=tk.X, padx=18, pady=4)

        # Legend
        tk.Label(panel, text="\nLEGEND", bg=COLOR_PANEL, fg=COLOR_ACCENT,
                 font=("Courier New", 9, "bold")).pack(anchor="w", padx=18)
        self._legend(panel, COLOR_START,    "Start")
        self._legend(panel, COLOR_TARGET,   "Target")
        self._legend(panel, COLOR_WALL,     "Wall")
        self._legend(panel, COLOR_FRONTIER, "Frontier")
        self._legend(panel, COLOR_EXPLORED, "Explored")
        self._legend(panel, COLOR_PATH,     "Final Path")

        # Status bar
        tk.Label(panel, textvariable=self.status, bg=COLOR_PANEL, fg="#a6e3a1",
                 font=("Courier New", 8), wraplength=190, justify=tk.LEFT).pack(
            anchor="w", padx=18, pady=(18, 6))

        # DLS note
        tk.Label(panel, text=f"DLS limit: {DLS_LIMIT}  (edit in code)", bg=COLOR_PANEL, fg="#6c7086",
                 font=("Courier New", 8)).pack(anchor="w", padx=18)

        # Canvas (right side)
        canvas_w = COLS * CELL_SIZE + 2 * CELL_PAD
        canvas_h = ROWS * CELL_SIZE + 2 * CELL_PAD
        self.canvas = tk.Canvas(self.root, width=canvas_w, height=canvas_h,
                                bg=COLOR_BG, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)

    def _legend(self, parent, color, label):
        row = tk.Frame(parent, bg=COLOR_PANEL)
        row.pack(anchor="w", padx=18, pady=1)
        tk.Label(row, bg=color, width=2, height=1).pack(side=tk.LEFT, padx=(0, 6))
        tk.Label(row, text=label, bg=COLOR_PANEL, fg=COLOR_TEXT,
                 font=("Courier New", 9)).pack(side=tk.LEFT)

    # ── Grid Drawing ──────────────────────────

    def _draw_grid(self, explored=None, frontier=None, path=None):
        self.canvas.delete("all")
        explored  = explored  or set()
        frontier  = frontier  or set()
        path_set  = set(path) if path else set()

        for r in range(ROWS):
            for c in range(COLS):
                x1 = c * CELL_SIZE + CELL_PAD
                y1 = r * CELL_SIZE + CELL_PAD
                x2 = x1 + CELL_SIZE - CELL_PAD * 2
                y2 = y1 + CELL_SIZE - CELL_PAD * 2

                cell = (r, c)
                if GRID[r][c] == -1:
                    color = COLOR_WALL
                elif cell == START:
                    color = COLOR_START
                elif cell == TARGET:
                    color = COLOR_TARGET
                elif cell in path_set:
                    color = COLOR_PATH
                elif cell in frontier:
                    color = COLOR_FRONTIER
                elif cell in explored:
                    color = COLOR_EXPLORED
                else:
                    color = COLOR_EMPTY

                self.canvas.create_rectangle(x1, y1, x2, y2,
                                             fill=color, outline="", tags="cell")

                # Draw labels for start/target
                if cell == START:
                    self.canvas.create_text((x1+x2)//2, (y1+y2)//2,
                                            text="S", fill=COLOR_BG,
                                            font=("Courier New", 11, "bold"))
                elif cell == TARGET:
                    self.canvas.create_text((x1+x2)//2, (y1+y2)//2,
                                            text="T", fill=COLOR_BG,
                                            font=("Courier New", 11, "bold"))

        # Draw path line on top
        if path and len(path) > 1:
            for i in range(len(path) - 1):
                r1, c1 = path[i]
                r2, c2 = path[i+1]
                cx1 = c1 * CELL_SIZE + CELL_SIZE // 2
                cy1 = r1 * CELL_SIZE + CELL_SIZE // 2
                cx2 = c2 * CELL_SIZE + CELL_SIZE // 2
                cy2 = r2 * CELL_SIZE + CELL_SIZE // 2
                self.canvas.create_line(cx1, cy1, cx2, cy2,
                                        fill="white", width=2, tags="path_line")

    # ── Controls ─────────────────────────────

    def _start(self):
        if self._running:
            return
        self._reset(clear_only=False)
        alg = self.algorithm.get()
        self.status.set(f"Running {alg}...")
        self._running = True

        generators = {
            "BFS":           bfs,
            "DFS":           dfs,
            "UCS":           ucs,
            "DLS":           dls,
            "IDDFS":         iddfs,
            "Bidirectional": bidirectional,
        }
        self._gen = generators[alg]()
        self._step()

    def _step(self):
        if not self._running or self._gen is None:
            return
        try:
            explored, frontier, path = next(self._gen)
            if path is not None:
                # Algorithm finished
                self._running = False
                if path:
                    self._draw_grid(explored, set(), path)
                    alg = self.algorithm.get()
                    self.status.set(
                        f"✓ {alg} found path!\n"
                        f"Path length: {len(path)-1} steps\n"
                        f"Nodes explored: {len(explored)}"
                    )
                else:
                    self._draw_grid(explored)
                    self.status.set("✗ No path found!")
            else:
                self._draw_grid(explored, frontier)
                self._job = self.root.after(self.speed_ms.get(), self._step)
        except StopIteration:
            self._running = False

    def _stop(self):
        self._running = False
        if self._job:
            self.root.after_cancel(self._job)
        self.status.set("Stopped. Press Reset to try again.")

    def _reset(self, clear_only=True):
        self._running = False
        if self._job:
            self.root.after_cancel(self._job)
            self._job = None
        self._gen = None
        self._draw_grid()
        if clear_only:
            self.status.set("Select an algorithm and press  ▶  Run")


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────

if __name__ == "__main__":
    root = tk.Tk()
    app  = PathfinderApp(root)
    root.mainloop()