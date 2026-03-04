# 🧭 AI Pathfinder — Uninformed Search Visualizer

> **Artificial Intelligence**
> Visualize six blind search algorithms navigating a grid maze step-by-step in real time.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Demo](#demo)
- [Requirements](#requirements)
- [Installation](#installation)
- [How to Run](#how-to-run)
- [Algorithms](#algorithms)
- [Movement Order](#movement-order)
- [GUI Guide](#gui-guide)
- [Color Legend](#color-legend)
- [Grid Configuration](#grid-configuration)
- [Project Structure](#project-structure)

---

## Overview

AI Pathfinder is an interactive GUI application that visualizes how six different **uninformed (blind) search algorithms** explore a grid maze to find a path from a **Start point S** to a **Target point T** while avoiding static wall obstacles.

The goal is not just to find the path — it's to **visualize the process**. The GUI animates every step of the search so you can watch each algorithm "think": which cells it checks first, how its frontier grows, and which final path it chooses.

| Property | Value |
|---|---|
| Grid size | 12 rows × 20 columns (240 cells) |
| Start (S) | Row 1, Column 1 |
| Target (T) | Row 10, Column 18 |
| Library | Python — Matplotlib + NumPy |
| Algorithms | BFS, DFS, UCS, DLS, IDDFS, Bidirectional |

---

## Requirements

| Package | Version | Purpose |
|---|---|---|
| Python | ≥ 3.8 | Runtime |
| matplotlib | ≥ 3.7 | GUI, rendering, widgets |
| numpy | ≥ 1.21 | Grid array operations |

> **GUI backend:** The script auto-detects the best available interactive backend in this order:
> `TkAgg → Qt5Agg → GTK3Agg → WXAgg → MacOSX`
>
> On **Windows**, `TkAgg` (bundled with most Python installs) is used automatically — no extra setup needed.

---

## Installation

```bash
# 1. Clone the repository
git clone https://github.com/your-username/ai-pathfinder.git
cd ai-pathfinder

# 2. Install dependencies
pip install matplotlib numpy
```

**Windows users** — if you get a backend error, make sure tkinter is available:
```bash
# Check if tkinter is installed
python -m tkinter
```
If the window opens, you're good. If not, reinstall Python and tick "tcl/tk and IDLE" during setup.

**Linux users** — install tkinter separately if needed:
```bash
sudo apt-get install python3-tk      # Ubuntu / Debian
sudo dnf install python3-tkinter     # Fedora
```

---

## How to Run

```bash
python pathfinder.py
```

The application window opens immediately. No additional configuration is needed.

---

## Algorithms

Six uninformed search strategies are implemented, each as a Python **generator** that yields step-by-step state updates to drive the animation.

### 1. 🟢 Breadth-First Search (BFS)
Uses a **FIFO queue**. Explores nodes level by level, expanding all neighbours at depth *d* before moving to depth *d+1*.
- ✅ **Complete** — always finds a solution if one exists
- ✅ **Optimal** — guaranteed shortest path (unit cost)
- ❌ High memory — stores all frontier nodes simultaneously
- **Result on test maze:** 18 steps, 97 nodes explored

### 2. 🔴 Depth-First Search (DFS)
Uses an **explicit stack**. Dives as deep as possible along one branch before backtracking.
- ✅ **Complete** — finds a solution (no infinite loops with visited set)
- ❌ **Not optimal** — may return a much longer path
- ✅ Low memory — only stores the current path stack
- **Result on test maze:** 29 steps, 131 nodes explored

### 3. 🔵 Uniform-Cost Search (UCS)
Uses a **min-heap priority queue** ordered by cumulative path cost.
- Straight moves cost **1.0**; diagonal moves cost **√2 ≈ 1.414**
- ✅ **Complete** and ✅ **Cost-optimal**
- ❌ Higher overhead than BFS (O(log n) heap ops)
- **Result on test maze:** 18 steps, 102 nodes explored

### 4. 🟠 Depth-Limited Search (DLS)
A **bounded DFS** that refuses to explore beyond a fixed depth limit (default: **20**).
- ❌ **Incomplete** — returns failure if limit < solution depth
- ❌ **Not optimal**
- ✅ Prevents infinite loops; predictable memory use
- **Result on test maze:** 20 steps, 89 nodes explored

### 5. 🟣 Iterative Deepening DFS (IDDFS)
Calls DLS repeatedly with limits **0, 1, 2, 3, …** until the target is found.
- ✅ **Complete** and ✅ **Optimal** (unit cost)
- ✅ DFS-level memory: O(b·d)
- ❌ Re-expands shallow nodes on every iteration
- **Result on test maze:** 18 steps, 209 nodes explored

### 6. 🔴 Bidirectional Search
Two **simultaneous BFS frontiers** expand from S and T, meeting in the middle.
- ✅ **Complete** and ✅ **Optimal**
- ✅ Most node-efficient — explores ~half the nodes of BFS
- ❌ More complex path stitching; roughly double the memory
- **Result on test maze:** 18 steps, **74 nodes explored** (fewest of all)

### Comparison Table

| Algorithm | Time | Space | Complete | Optimal | Steps | Explored |
|---|---|---|---|---|---|---|
| BFS | O(b^d) | O(b^d) | ✅ Yes | ✅ Yes | 18 | 97 |
| DFS | O(b^m) | O(b·m) | ✅ Yes | ❌ No | 29 | 131 |
| UCS | O(b^C*/ε) | O(b^C*/ε) | ✅ Yes | ✅ Yes | 18 | 102 |
| DLS | O(b^l) | O(b·l) | ❌ No | ❌ No | 20 | 89 |
| IDDFS | O(b^d) | O(b·d) | ✅ Yes | ✅ Yes | 18 | 209 |
| Bidirectional | O(b^d/2) | O(b^d/2) | ✅ Yes | ✅ Yes | **18** | **74** |

---

## Movement Order

When expanding any node, all algorithms follow this exact **clockwise order using only the Main Diagonal** — as specified in the assignment:

| # | Direction | (Δrow, Δcol) |
|---|---|---|
| 1 | ⬆ Up | (-1, 0) |
| 2 | ➡ Right | (0, +1) |
| 3 | ⬇ Down | (+1, 0) |
| 4 | ↘ Down-Right *(diagonal)* | (+1, +1) |
| 5 | ⬅ Left | (0, -1) |
| 6 | ↖ Up-Left *(diagonal)* | (-1, -1) |

> ⚠️ **Top-Right (-1, +1)** and **Bottom-Left (+1, -1)** diagonals are explicitly **excluded** per the assignment specification. This is enforced in the shared `neighbors()` function used by all six algorithms.

---

## GUI Guide

```
┌────────────────────┬────────────────────────────────────────────┐
│   CONTROL PANEL    │                                            │
│                    │                                            │
│  ALGORITHM         │                                            │
│  ○ BFS             │                                            │
│  ○ DFS             │          G R I D   C A N V A S            │
│  ○ UCS             │                                            │
│  ○ DLS             │     12 rows × 20 cols  (animated)          │
│  ○ IDDFS           │                                            │
│  ○ Bidirectional   │                                            │
│                    │                                            │
│  SPEED (ms/step)   │                                            │
│  [====slider====]  │                                            │
│                    ├────────────────────────────────────────────┤
│  [ ▶  Run       ]  │  ✓ BFS — path found!  Length: 18  Explored: 97  │
│  [ ■  Stop      ]  └────────────────────────────────────────────┘
│  [ ↺  Reset     ]
│  [⬛ Clear Grid ]
│
│  ✏ EDIT MODE
│  [[ ] Wall (L=add R=erase)]
│  [[S]  Set Start          ]
│  [[T]  Set Target         ]
│
│  LEGEND
│  █ Start (S)
│  █ Target (T)
│  █ Wall
│  █ Frontier (queue)
│  █ Explored
│  █ Final Path
└────────────────────
```

### Controls

| Control | Action |
|---|---|
| **Algorithm radio buttons** | Select which algorithm to run |
| **Speed slider** | Set animation delay (10 ms = fast, 500 ms = slow) |
| **▶ Run** | Start the selected algorithm from scratch |
| **■ Stop** | Pause the animation mid-search |
| **↺ Reset** | Clear the visualization (keeps maze walls) |
| **⬛ Clear Grid** | Remove all walls, reset to empty maze |

### Edit Mode

Click an edit button to activate it, then interact with the grid:

| Button | How to use |
|---|---|
| **[ ] Wall** | Left-click to add a wall · Right-click to erase · Click-and-drag to paint |
| **[S] Set Start** | Click any open cell to move the Start point |
| **[T] Set Target** | Click any open cell to move the Target point |

> Edit mode is disabled while the algorithm is running. Press **■ Stop** or wait for completion first.

---

## Color Legend

| Color | Meaning |
|---|---|
| 🟢 Neon green | Start point (S) |
| 🔵 Electric blue | Target point (T) |
| 🔴 Hot red | Wall / obstacle |
| 🟠 Bright amber | Frontier — nodes in queue/stack, not yet expanded |
| 🟣 Vivid violet | Explored — nodes already visited/expanded |
| 🟡 Laser yellow | Final path from S to T |
| ⬛ Dark navy | Empty open cell |

---

## Grid Configuration

The maze is defined as a hardcoded 2D array at the top of `pathfinder.py`:

```python
DEFAULT_GRID = [
    [ 0,  0,  0,  0, ...],   # 0  = open cell
    [ 0,  0,  0, -1, ...],   # -1 = wall
    ...
]
DEFAULT_START  = (1, 1)     # (row, col)
DEFAULT_TARGET = (10, 18)   # (row, col)
DLS_LIMIT = 20              # depth limit for DLS and IDDFS
```

To change the maze, edit `DEFAULT_GRID` directly — or use the **Wall edit mode** in the GUI at runtime. To permanently save a layout, update `DEFAULT_GRID` in the source file.

---

## Project Structure

```
ai-pathfinder/
│
├── pathfinder.py          # Main application (all code in one file)
├── README.md              # This file
└── AI_Pathfinder_Report.pdf  # Comprehensive assignment report
```

### Key sections inside `pathfinder.py`

| Lines | Content |
|---|---|
| `DEFAULT_GRID` | Hardcoded 12×20 maze |
| `MOVES` | Strict clockwise movement order (6 directions) |
| `neighbors()` | Shared neighbour generator used by all algorithms |
| `algo_bfs()` | BFS generator |
| `algo_dfs()` | DFS generator |
| `algo_ucs()` | UCS generator |
| `algo_dls()` | DLS generator |
| `algo_iddfs()` | IDDFS generator |
| `algo_bidir()` | Bidirectional Search generator |
| `class App` | Matplotlib GUI — panel, grid, animation, mouse events |

---

## Troubleshooting

**`AttributeError: 'RadioButtons' object has no attribute 'circles'`**
→ You need matplotlib ≥ 3.7. Upgrade with:
```bash
pip install --upgrade matplotlib
```

**`TypeError: GridSpecFromSubplotSpec.__init__() got an unexpected keyword argument 'left'`**
→ Same fix — upgrade matplotlib to ≥ 3.7.

**Window doesn't open / blank screen**
→ The script auto-selects a backend. If none work, install tkinter (see [Installation](#installation)).

**Animation is too fast/slow**
→ Adjust the **Speed slider** in the GUI (10 ms = fastest, 500 ms = slowest).

---

*Built with Python · Matplotlib · NumPy*
