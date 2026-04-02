import os
import glob
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


def read_frames(filename):
    """Read a csv file where frames are separated by blank lines.

    Returns a list of numpy arrays, one per frame.
    """
    frames = []
    with open(filename, 'r') as f:
        current = []
        for line in f:
            line = line.strip()
            if line == '':
                if current:
                    frames.append(np.array(current, dtype=float))
                    current = []
            else:
                parts = line.split(',')
                if parts[-1] == '':
                    parts = parts[:-1]
                current.append([float(p) for p in parts])
        if current:
            frames.append(np.array(current, dtype=float))
    return frames


def read_matrix_file(filename):
    """Read CSV matrix data in row-major format, ignoring blank lines."""
    rows = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if parts[-1] == '':
                parts = parts[:-1]
            rows.append([float(x) for x in parts])
    return np.array(rows, dtype=float)


def load_scenario_from_results(results_dir, scenario=None):
    """Load frames from results-con/<scenario> directory."""
    if scenario:
        scenario_dir = os.path.join(results_dir, scenario)
    else:
        subdirs = [d for d in glob.glob(os.path.join(results_dir, '*')) if os.path.isdir(d)]
        if not subdirs:
            raise FileNotFoundError(f'No scenarios found in {results_dir}')
        scenario_dir = sorted(subdirs)[0]

    if not os.path.isdir(scenario_dir):
        raise FileNotFoundError(f'Scenario directory not found: {scenario_dir}')

    conn_file = os.path.join(scenario_dir, 'connectivity_matrices.csv')
    mobility_file = os.path.join(scenario_dir, 'mobility.csv')
    nodepos_file = os.path.join(scenario_dir, 'node_positions.csv')

    if not os.path.isfile(conn_file):
        raise FileNotFoundError(f'Connectivity matrix file not found in {scenario_dir}')

    conn_raw = read_matrix_file(conn_file)
    N = conn_raw.shape[1]

    # Convert connectivity to frames
    if conn_raw.shape[0] == N and conn_raw.shape[1] == N:
        conn_frames = [conn_raw]
    elif conn_raw.shape[0] % N == 0:
        nframes_conn = conn_raw.shape[0] // N
        conn_frames = [conn_raw[i * N:(i + 1) * N, :] for i in range(nframes_conn)]
    else:
        raise ValueError('Connectivity file has unexpected shape: {} (with N={})'.format(conn_raw.shape, N))

    def load_node_frames():
        node_data = read_matrix_file(nodepos_file)
        if node_data.shape[1] < 2:
            raise ValueError('node_positions.csv must contain at least x and y columns')

        if node_data.shape[0] == N:
            # static positions for all frames
            base = node_data[:, :3] if node_data.shape[1] >= 3 else np.column_stack((node_data[:, :2], np.zeros(N)))
            return [base] * len(conn_frames)

        if node_data.shape[0] % N == 0:
            nframes_pos = node_data.shape[0] // N
            frames = []
            for i in range(nframes_pos):
                block = node_data[i * N:(i + 1) * N, :]
                if block.shape[1] >= 3:
                    frames.append(block[:, :3])
                else:
                    frames.append(np.column_stack((block[:, :2], np.zeros(N))))
            return frames

        raise ValueError('node_positions.csv has unexpected row count {} for N={}'.format(node_data.shape[0], N))

    mob_frames = []
    if os.path.isfile(mobility_file):
        mob_frames = read_frames(mobility_file)
        if len(mob_frames) == 0:
            mob_frames = []

    if not mob_frames:
        if os.path.isfile(nodepos_file):
            mob_frames = load_node_frames()
        else:
            raise FileNotFoundError('No mobility data found in scenario; expected mobility.csv or node_positions.csv')

    # Align frame counts
    if len(mob_frames) != len(conn_frames):
        n = min(len(mob_frames), len(conn_frames))
        mob_frames = mob_frames[:n]
        conn_frames = conn_frames[:n]

    return mob_frames, conn_frames, scenario_dir


def animate_connectivity(mob_frames, conn_frames, interval=200, save=None):
    """Create an animation of the nodes and their connectivity."""
    assert len(mob_frames) == len(conn_frames), 'mobility and connectivity frames count mismatch'
    nframes = len(mob_frames)
    N = mob_frames[0].shape[0]

    fig, ax = plt.subplots()
    scat = ax.scatter([], [], s=50, c='blue')
    lines = []

    def init():
        ax.set_xlim(-750, 750)
        ax.set_ylim(-750, 750)
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title('t=0 (frame 0)')
        return scat,

    def update(frame_index):
        pos = mob_frames[frame_index]
        adj = conn_frames[frame_index].astype(int)
        scat.set_offsets(pos[:, :2])
        for ln in lines:
            ln.remove()
        lines.clear()

        for i in range(N):
            for j in range(i + 1, N):
                if adj[i, j] != 0:
                    x = [pos[i, 0], pos[j, 0]]
                    y = [pos[i, 1], pos[j, 1]]
                    ln, = ax.plot(x, y, 'k-', lw=0.5, alpha=0.5)
                    lines.append(ln)

        ax.set_title(f'frame={frame_index}')
        return scat, *lines

    ani = animation.FuncAnimation(fig, update, frames=nframes,
                                  init_func=init, blit=False, interval=interval)

    if save:
        ext = os.path.splitext(save)[1].lower().lstrip('.')
        try:
            if ext in ('mp4', 'avi'):
                if 'ffmpeg' in animation.writers.list():
                    writer = animation.FFMpegWriter(fps=1000 / interval)
                    ani.save(save, writer=writer, dpi=150)
                else:
                    raise RuntimeError('ffmpeg writer not available, please install ffmpeg or choose .gif output')
            elif ext == 'gif':
                ani.save(save, writer='pillow', dpi=150)
            else:
                ani.save(save, dpi=150)
        except Exception as e:
            print(f'Error saving animation: {e}')
            print('Attempting to display instead...')
            plt.show()
    else:
        plt.show()


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Plot mobility and connectivity animation')
    parser.add_argument('--results-dir', default='results-con', help='root results directory containing scenario subfolders')
    parser.add_argument('--scenario', help='scenario subfolder name under results-dir')
    parser.add_argument('--mob', default='mobility-rt.csv', help='legacy mobility csv file path (fallback)')
    parser.add_argument('--conn', default='connectivityM-rt.csv', help='legacy connectivity csv file path (fallback)')
    parser.add_argument('--interval', type=int, default=200, help='frame interval in ms')
    parser.add_argument('--save', help='output animation file (mp4 or gif)')
    parser.add_argument('--save-all', action='store_true', help='save animations for all scenarios in results-dir')
    parser.add_argument('--save-dir', default='plot_results', help='output directory for save-all mode')
    args = parser.parse_args()

    if args.results_dir and os.path.isdir(args.results_dir):
        scenario_dirs = []
        if args.scenario:
            scenario_dirs = [args.scenario]
        else:
            scenario_dirs = sorted([os.path.basename(d) for d in glob.glob(os.path.join(args.results_dir, '*')) if os.path.isdir(d)])

        if not scenario_dirs:
            raise FileNotFoundError(f'No scenario folders found under {args.results_dir}')

        if args.save_all:
            os.makedirs(args.save_dir, exist_ok=True)

        for scenario in scenario_dirs:
            mob_frames, conn_frames, chosen = load_scenario_from_results(args.results_dir, scenario)
            print(f'Loaded scenario: {chosen}')
            print(f'Frames: mobility={len(mob_frames)}, connectivity={len(conn_frames)}')

            if args.save_all:
                if args.save:
                    ext = os.path.splitext(args.save)[1].lower().lstrip('.')
                else:
                    ext = 'gif'
                out_file = os.path.join(args.save_dir, f'{scenario}.{ext}')
            else:
                out_file = args.save

            animate_connectivity(mob_frames, conn_frames, interval=args.interval, save=out_file)
            if out_file:
                print(f'Saved animation for {scenario}: {out_file}')
            plt.close('all')

    else:
        print('Using legacy paths: mob=%s conn=%s' % (args.mob, args.conn))
        mob_frames = read_frames(args.mob)
        conn_frames = read_frames(args.conn)
        print(f'Read {len(mob_frames)} mobility frames and {len(conn_frames)} connectivity frames')
        conn_mats = []
        for f in conn_frames:
            total = f.size
            N = int(np.sqrt(total))
            conn_mats.append(f.reshape((N, N)))
        conn_frames = conn_mats

        animate_connectivity(mob_frames, conn_frames, interval=args.interval, save=args.save)
