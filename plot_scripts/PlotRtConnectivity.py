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
                # ignore trailing empty element if any
                if parts[-1] == '':
                    parts = parts[:-1]
                current.append([float(p) for p in parts])
        if current:
            frames.append(np.array(current, dtype=float))
    return frames


def animate_connectivity(mob_frames, conn_frames, interval=200, save=None):
    """Create an animation of the nodes and their connectivity.

    mob_frames : list of (N,3) arrays
    conn_frames : list of (N,N) arrays (0/1)
    interval : ms between frames
    save : filename to save the animation (mp4 or gif), or None to show.
    """
    assert len(mob_frames) == len(conn_frames), "mobility and connectivity frames count mismatch"
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
        ax.set_title('t=0.0 s')
        return scat,

    def update(frame):
        pos = mob_frames[frame]
        adj = conn_frames[frame].astype(int)
        scat.set_offsets(pos[:, :2])
        # remove old lines
        for ln in lines:
            ln.remove()
        lines.clear()
        # draw new connections
        for i in range(N):
            for j in range(i+1, N):
                if adj[i,j]:
                    x = [pos[i,0], pos[j,0]]
                    y = [pos[i,1], pos[j,1]]
                    ln, = ax.plot(x, y, 'k-', lw=0.5, alpha=0.5)
                    lines.append(ln)
        ax.set_title(f't={frame} (step)')
        return scat, *lines

    ani = animation.FuncAnimation(fig, update, frames=nframes,
                                  init_func=init, blit=False, interval=interval)
    if save:
        ext = save.split('.')[-1].lower()
        try:
            if ext in ('mp4', 'avi'):
                # try ffmpeg writer
                if 'ffmpeg' in animation.writers.list():
                    writer = animation.FFMpegWriter(fps=1000/interval)
                    ani.save(save, writer=writer, dpi=150)
                else:
                    raise RuntimeError('ffmpeg writer not available, please install ffmpeg or choose .gif output')
            elif ext in ('gif',):
                # Pillow can write gif
                ani.save(save, writer='pillow', dpi=150)
            else:
                # let matplotlib guess
                ani.save(save, dpi=150)
        except Exception as e:
            print(f"Error saving animation: {e}")
            print("Attempting to display instead...")
            plt.show()
    else:
        plt.show()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Plot mobility and connectivity animation')
    parser.add_argument('--mob', default='mobility-rt.csv', help='mobility csv file')
    parser.add_argument('--conn', default='connectivityM-rt.csv', help='connectivity csv file')
    parser.add_argument('--interval', type=int, default=200, help='frame interval in ms')
    parser.add_argument('--save', help='output animation file (mp4 or gif)')
    args = parser.parse_args()

    mob_frames = read_frames(args.mob)
    conn_frames = read_frames(args.conn)
    print(f"Read {len(mob_frames)} mobility frames and {len(conn_frames)} connectivity frames")
    # connectivity file contains 0/1 entries; convert each frame to square matrix
    conn_mats = []
    for f in conn_frames:
        # determine N by square root of length
        total = f.size
        N = int(np.sqrt(total))
        conn_mats.append(f.reshape((N,N)))

    animate_connectivity(mob_frames, conn_mats, interval=args.interval, save=args.save)
