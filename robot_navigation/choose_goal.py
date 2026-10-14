"""Click a destination on a saved map; prints navigation goal arguments."""
import argparse
from .paths import DEFAULT_MAP
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from .navigate_map import Map

def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--map', type=Path, default=DEFAULT_MAP)
    p.add_argument('--clearance', type=float, default=0.4)
    args = p.parse_args()
    map_ = Map(args.map, args.clearance)
    print(f'Loaded map: {args.map.resolve()}')
    h,w = map_.free.shape
    image = np.where(map_.occupied, 0, np.where(map_.free, 254, 205))
    fig, ax = plt.subplots()
    ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=255,
              extent=[map_.origin[0], map_.origin[0]+w*map_.resolution,
                      map_.origin[1], map_.origin[1]+h*map_.resolution])
    ax.set(xlabel='World X (metres)', ylabel='World Y (metres)', title=f'{args.map}\nClick a white area well away from walls; close to cancel')
    while plt.fignum_exists(fig.number):
        points = plt.ginput(1, timeout=-1)
        if not points:
            break
        cell = map_.cell(points[0])
        if map_.valid(cell) and not map_.base_blocked[cell]:
            print(f'--goal {points[0][0]:.3f} {points[0][1]:.3f}')
            break
        print('Choose known free space farther from walls and gray areas.')
    plt.close(fig)


if __name__ == "__main__":
    main()
