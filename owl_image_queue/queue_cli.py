import argparse, os, json
from .sqlite_queue import ImageQueue

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--base-dir', default='/home/dev/bags')
    ap.add_argument('cmd', choices=['size', 'drain', 'requeue-missing'])
    ap.add_argument('--n', type=int, default=100)
    args = ap.parse_args()
    q = ImageQueue(os.path.join(args.base_dir, 'queue.db'))
    if args.cmd == 'size':
        print(q.size())
    elif args.cmd == 'drain':
        paths = q.dequeue_batch(args.n)
        print('\n'.join(paths))
    elif args.cmd == 'requeue-missing':
        n = q.requeue_missing()
        print(f'removed {n} missing entries')
