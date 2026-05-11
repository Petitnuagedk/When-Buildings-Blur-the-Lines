import os
import glob
import argparse
import math
import numpy as np
import pandas as pd
import networkx as nx
from typing import List, Tuple, Dict
import matplotlib.pyplot as plt
import matplotlib
from sympy import false

def load_adj_matrices_from_csv(path: str) -> np.ndarray:
    """
    Load adjacency matrices stored as stacked NxN blocks separated by one or more empty lines.
    Each block is N rows (comma-separated) representing one adjacency matrix.
    Returns array of shape (T, N, N) with dtype=int.
    """
    with open(path, "r") as f:
        raw_lines = [line.rstrip("\n") for line in f]

    blocks = []
    cur = []
    for line in raw_lines:
        if line.strip() == "":
            if cur:
                blocks.append(cur)
                cur = []
        else:
            cur.append(line)
    if cur:
        blocks.append(cur)

    if not blocks:
        raise ValueError("No matrix blocks found in file")

    mats = []
    shape = None
    for b in blocks:
        rows = []
        for row in b:
            # allow trailing commas
            parts = [p.strip() for p in row.split(",") if p.strip() != ""]
            rows.append([int(p) for p in parts])
        arr = np.array(rows, dtype=int)
        if arr.ndim != 2 or arr.shape[0] != arr.shape[1]:
            raise ValueError(f"Each block must be a square matrix; found shape {arr.shape}")
        if shape is None:
            shape = arr.shape
        else:
            if arr.shape != shape:
                raise ValueError(f"Inconsistent block shapes found: {arr.shape} vs {shape}")
        mats.append(arr)
    return np.stack(mats, axis=0)

def graph_from_adj(adj: np.ndarray, directed: bool = False) -> nx.Graph:
    """
    Build a NetworkX Graph (or DiGraph if directed=True) from adjacency matrix (N,N).
    Assumes adjacency entries are 0/1 (or can be nonzero).
    """
    if directed:
        G = nx.DiGraph()
    else:
        G = nx.Graph()
    N = adj.shape[0]
    G.add_nodes_from(range(N))
    rows, cols = np.where(adj != 0)
    edges = [(int(i), int(j)) for i, j in zip(rows, cols)]
    # If undirected and adjacency has both (i,j) and (j,i), adding both is harmless
    G.add_edges_from(edges)
    return G

def canonical_shortest_path(G: nx.Graph, u: int, v: int) -> List[int]:
    """
    Returns a deterministic shortest path between u and v in graph G.
    networkx.shortest_path is deterministic given adjacency node ordering,
    so this suffices. If you prefer tie-breaking differently, change this.
    If no path, raises nx.NetworkXNoPath.
    """
    return nx.shortest_path(G, source=u, target=v)

def compute_pair_metrics(adj_matrices: np.ndarray,
                         pairs: List[Tuple[int,int]],
                         directed: bool = False) -> Dict[Tuple[int,int], Dict]:
    """
    All metrics returned are normalized to [0,1].
    - lifetime: fraction of frames where a path exists (normalized by T)
    - stability: normalized switch rate (switches / (T-1))
    - avail_to_unavail_rate: normalized (count / (T-1))
    - persistency_exact: fraction of consecutive available-frame pairs with identical path (0 if undefined)
    - persistency_jaccard: average Jaccard similarity across consecutive available frames (0 if undefined)
    - avg_path_length_norm: average shortest-path length normalized by maximum possible (N-1). 0 if never available.
    """
    T, N, _ = adj_matrices.shape
    results = {}
    graphs = [graph_from_adj(adj_matrices[t], directed=directed) for t in range(T)]

    for (u, v) in pairs:
        reachable = np.zeros(T, dtype=bool)
        paths = [None] * T
        for t in range(T):
            G = graphs[t]
            if u in G and v in G:
                try:
                    p = canonical_shortest_path(G, u, v)
                    reachable[t] = True
                    paths[t] = p
                except nx.NetworkXNoPath:
                    reachable[t] = False
                    paths[t] = None
            else:
                reachable[t] = False
                paths[t] = None

        # lifetime normalized
        lifetime_frac = float(reachable.sum()) / float(T) if T > 0 else 0.0

        # switches normalized (normalized by T-1)
        if T <= 1:
            switches_rate = 0.0
            avail_to_unavail_rate = 0.0
        else:
            transitions = reachable[:-1] != reachable[1:]
            switches_rate = float(int(transitions.sum())) / float(T-1)
            avail_to_unavail = (reachable[:-1] & (~reachable[1:]))
            avail_to_unavail_rate = float(int(avail_to_unavail.sum())) / float(T-1)

        # persistency between consecutive available frames
        identical_count = 0
        jaccard_sum = 0.0
        common_pairs = 0
        path_lengths = []
        for t in range(T):
            if paths[t] is not None:
                path_lengths.append(len(paths[t]) - 1)  # edges
        for t in range(T-1):
            if reachable[t] and reachable[t+1]:
                p1 = paths[t]
                p2 = paths[t+1]
                common_pairs += 1
                if p1 == p2:
                    identical_count += 1
                s1 = set(p1)
                s2 = set(p2)
                inter = len(s1 & s2)
                union = len(s1 | s2)
                jaccard = (inter / union) if union > 0 else 1.0
                jaccard_sum += jaccard

        persistency_exact = (identical_count / common_pairs) if common_pairs > 0 else 0.0
        persistency_jaccard = (jaccard_sum / common_pairs) if common_pairs > 0 else 0.0

        # average path length normalized by max possible (N-1)
        if path_lengths:
            avg_len = float(np.mean(path_lengths))
            max_len = float(max(1, N-1))
            #print(f"Pair ({u},{v}): avg_len={avg_len:.3f}")
            avg_path_length_norm = avg_len / max_len
            avg_inverse_path_length = float(np.mean([1.0 / l for l in path_lengths if l > 0]))
            avg_path_length = avg_len
        else:
            avg_path_length_norm = 0.0
            avg_inverse_path_length = 0.0
            avg_path_length = 0.0

        results[(u, v)] = {
            "lifetime": lifetime_frac,
            "stability": 1.0 - switches_rate,            # stability higher when fewer switches
            "switches_rate": switches_rate,               # kept normalized in [0,1]
            "avail_to_unavail_rate": avail_to_unavail_rate,
            "persistency_exact": persistency_exact,
            "persistency_jaccard": persistency_jaccard,
            "avg_path_length_norm": avg_path_length_norm,
            "avg_inverse_path_length": avg_inverse_path_length,
            "avg_path_length": avg_path_length,
            # debug series (not normalized, kept for inspection)
            "reachability_series": reachable,
            "paths": paths
        }

    return results


def discover_scenarios(results_dir: str) -> List[str]:
    """Return sorted scenario subdirs containing connectivity_matrices.csv."""
    if not os.path.isdir(results_dir):
        raise FileNotFoundError(f"Results directory not found: {results_dir}")
    paths = sorted(
        [d for d in glob.glob(os.path.join(results_dir, '*'))
         if os.path.isdir(d) and os.path.isfile(os.path.join(d, 'connectivity_matrices.csv'))]
    )
    if not paths:
        raise FileNotFoundError(f"No scenario folders with connectivity_matrices.csv found in {results_dir}")
    return paths


def discover_scenarios_nested(results_dir: str) -> List[str]:
    """
    Discover scenarios in the nested structure:
        results_dir/
          Loss model A/
            numNodes/
              10/
                connectivity_matrices.csv
              20/
              ...
          Loss model B/
            ...
    Returns sorted list of leaf paths containing connectivity_matrices.csv.
    """
    if not os.path.isdir(results_dir):
        raise FileNotFoundError(f"Results directory not found: {results_dir}")
    paths = []
    for loss_model in sorted(os.listdir(results_dir)):
        loss_model_path = os.path.join(results_dir, loss_model)
        if not os.path.isdir(loss_model_path):
            continue
        num_nodes_path = os.path.join(loss_model_path, 'numNodes')
        if not os.path.isdir(num_nodes_path):
            continue
        for node_count in sorted(os.listdir(num_nodes_path),
                                  key=lambda x: int(x) if x.isdigit() else x):
            leaf = os.path.join(num_nodes_path, node_count)
            if os.path.isdir(leaf) and os.path.isfile(os.path.join(leaf, 'connectivity_matrices.csv')):
                paths.append(leaf)
    if not paths:
        raise FileNotFoundError(
            f"No connectivity_matrices.csv found in nested loss_model/numNodes/N structure under {results_dir}"
        )
    return paths


def _is_nested_structure(results_dir: str) -> bool:
    """Return True if results_dir follows the loss_model/numNodes/N nested layout."""
    # Check if 'all' appears as a path component
    parts = os.path.normpath(results_dir).replace('\\', '/').split('/')
    if 'all' in [p.lower() for p in parts]:
        return True
    # Also detect by structure: any immediate subdir has a 'numNodes' child
    if os.path.isdir(results_dir):
        for entry in os.listdir(results_dir):
            subdir = os.path.join(results_dir, entry)
            if os.path.isdir(subdir) and os.path.isdir(os.path.join(subdir, 'numNodes')):
                return True
    return False


def save_metrics_csv(metrics: Dict[Tuple[int,int], Dict], out_file: str):
    with open(out_file, 'w') as f:
        f.write('src,dst,lifetime,stability,switches_rate,avail_to_unavail_rate,persistency_exact,persistency_jaccard,avg_path_length_norm,avg_inverse_path_length,avg_path_length\n')
        for (u, v), m in sorted(metrics.items()):
            f.write(f"{u},{v},{m['lifetime']:.6f},{m['stability']:.6f},{m['switches_rate']:.6f},{m['avail_to_unavail_rate']:.6f},{m['persistency_exact']:.6f},{m['persistency_jaccard']:.6f},{m['avg_path_length_norm']:.6f},{m['avg_inverse_path_length']:.6f},{m['avg_path_length']:.6f}\n")


def plot_pair_metrics_heatmap(metrics: Dict[Tuple[int,int], Dict],
                              N: int,
                              *,
                              show_keys: Tuple[str,str,str] = ("lifetime","stability","persistency_exact"),
                              figsize=(12,4),
                              out_file: str = None):
    """
    Create a 3-panel heatmap (lifetime | stability | persistency) for node pairs (i,j).
    - metrics: dict keyed by (i,j) with metric entries in [0,1]
    - N: number of nodes (matrix dimension)
    - show_keys: tuple of three metric keys to display in order
    - out_file: if provided, save figure to path
    """
    # prepare matrices
    mats = [np.full((N,N), np.nan) for _ in range(3)]
    for (i,j), vals in metrics.items():
        if 0 <= i < N and 0 <= j < N:
            for k,key in enumerate(show_keys):
                v = vals.get(key, None)
                if v is None:
                    mats[k][i,j] = np.nan
                else:
                    mats[k][i,j] = float(v)

    # plotting
    fig, axes = plt.subplots(1, 3, figsize=figsize)
    cmaps = ['Reds','Blues','Greens']
    titles = ['Lifetime', 'Stability', 'Persistency (exact)']
    for ax, mat, cmap, title in zip(axes, mats, cmaps, titles):
        im = ax.imshow(mat, vmin=0.0, vmax=1.0, cmap=cmap, origin='lower')
        ax.set_title(title)
        ax.set_xlabel("dst j")
        ax.set_ylabel("src i")
        # annotate each cell with value if non-nan
        if annot == True:
            for (i, j), _ in np.ndenumerate(mat):
                val = mat[i,j]
                if np.isfinite(val):
                    txt = f"{val:.2f}"
                    ax.text(j, i, txt, ha='center', va='center', color='black', fontsize=6)
                else:
                    # dim diagonal or undefined cells
                    if i == j:
                        ax.text(j, i, "-", ha='center', va='center', color='gray', fontsize=6)
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.02)

    plt.tight_layout()
    if out_file:
        plt.savefig(out_file, dpi=200, bbox_inches='tight')
    else:
        plt.show()

# Example usage:
annot = False
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute path metrics over connectivity matrices from results-con')
    parser.add_argument('--results-dir', default='results-con-SB-FOBA-alt', help='Root results directory containing scenario subdirectories')
    parser.add_argument('--scenario', default=None, help='Specific scenario subdirectory (uses all if omitted)')
    parser.add_argument('--out-dir', default=None, help='Directory to store outputs (default: {results-dir}-output)')
    parser.add_argument('--pair-samples', default=None, help='Optional list of fixed pairs as i,j; format: "0,1;2,3"')
    parser.add_argument('--annot', action='store_true', help='Enable annotation in heatmap')
    args = parser.parse_args()
    heatmap = False
    if args.out_dir is None:
        args.out_dir = args.results_dir.rstrip('/\\') + '-output'
    os.makedirs(args.out_dir, exist_ok=True)

    nested = _is_nested_structure(args.results_dir)
    if nested:
        print(f"Detected nested loss_model/numNodes/N structure under '{args.results_dir}'")

    if args.scenario:
        scenario_path = os.path.join(args.results_dir, args.scenario)
        if not os.path.isdir(scenario_path):
            raise FileNotFoundError(f"Scenario not found: {scenario_path}")
        scenarios = [scenario_path]
    else:
        if nested:
            scenarios = discover_scenarios_nested(args.results_dir)
        else:
            scenarios = discover_scenarios(args.results_dir)

    for scenario_path in scenarios:
        if nested:
            # Build a flat name like "LossModelA_numNodes_10" from the relative path
            rel = os.path.relpath(scenario_path, args.results_dir)
            scenario_name = rel.replace(os.sep, '_').replace(' ', '_')
        else:
            scenario_name = os.path.basename(scenario_path)
        connectivity_path = os.path.join(scenario_path, 'connectivity_matrices.csv')
        print(f"Processing scenario {scenario_name} -> {connectivity_path}")

        adj3 = load_adj_matrices_from_csv(connectivity_path)   # shape (T,N,N)
        T, N, _ = adj3.shape

        if args.pair_samples:
            pairs = []
            for part in args.pair_samples.split(';'):
                tokens = [x.strip() for x in part.split(',') if x.strip() != '']
                if len(tokens) == 2:
                    pairs.append((int(tokens[0]), int(tokens[1])))
        else:
            pairs = [(0, 1), (2, 5), (1, min(59, N - 1))] if N > 1 else []

        metrics = compute_pair_metrics(adj3, pairs)
        print(f"sample pairs metrics for {scenario_name}:")
        for pair, m in metrics.items():
            print(pair,
                  f"lifetime={m['lifetime']:.3f}",
                  f"stability={m['stability']:.3f}",
                  f"switches_rate={m['switches_rate']:.3f}",
                  f"persistency_exact={m['persistency_exact']:.3f}",
                  f"persistency_jaccard={m['persistency_jaccard']:.3f}",
                  f"avg_path_length_norm={m['avg_path_length_norm']:.3f}",
                  f"avg_inverse_path_length={m['avg_inverse_path_length']:.3f}",
                  f"avg_path_length={m['avg_path_length']:.3f}")

        all_pairs = [(i, j) for i in range(N) for j in range(N) if i != j]
        metrics_all = compute_pair_metrics(adj3, all_pairs)

        csv_out = os.path.join(args.out_dir, f'{scenario_name}_pair_metrics.csv')
        save_metrics_csv(metrics_all, csv_out)
        print(f"Saved metrics CSV: {csv_out}")
        if heatmap == True :
            heatmap_out = os.path.join(args.out_dir, f'{scenario_name}_heatmap_metrics.png')
            annot = args.annot
            plot_pair_metrics_heatmap(metrics_all, N, out_file=heatmap_out)
            print(f"Saved heatmap: {heatmap_out}")
