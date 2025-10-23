import sqlite3, os, pathlib
from typing import List, Optional

SCHEMA = """
CREATE TABLE IF NOT EXISTS queue(
  path TEXT PRIMARY KEY,
  mtime REAL
);
CREATE INDEX IF NOT EXISTS idx_mtime ON queue(mtime);
"""

class ImageQueue:
    def __init__(self, db_path: str):
        self.db_path = db_path
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        self._conn = sqlite3.connect(db_path, timeout=30)
        self._conn.execute('PRAGMA journal_mode=WAL;')
        self._conn.executescript(SCHEMA)
        self._conn.commit()

    def enqueue(self, path: str, mtime: Optional[float]=None):
        p = str(pathlib.Path(path))
        mt = mtime if mtime is not None else os.path.getmtime(p)
        try:
            self._conn.execute('INSERT OR IGNORE INTO queue(path, mtime) VALUES(?, ?)', (p, mt))
            self._conn.commit()
        except sqlite3.Error:
            self._conn.rollback()
            raise

    def dequeue_batch(self, n: int) -> List[str]:
        cur = self._conn.cursor()
        cur.execute('SELECT path FROM queue ORDER BY mtime LIMIT ?', (n,))
        rows = cur.fetchall()
        paths = [r[0] for r in rows]
        if not paths:
            return []
        cur.executemany('DELETE FROM queue WHERE path=?', [(p,) for p in paths])
        self._conn.commit()
        return paths

    def size(self) -> int:
        cur = self._conn.cursor()
        cur.execute('SELECT COUNT(*) FROM queue')
        return cur.fetchone()[0]

    def requeue_missing(self):
        cur = self._conn.cursor()
        cur.execute('SELECT path FROM queue')
        paths = [r[0] for r in cur.fetchall()]
        missing = [p for p in paths if not os.path.exists(p)]
        if missing:
            cur.executemany('DELETE FROM queue WHERE path=?', [(p,) for p in missing])
            self._conn.commit()
            return len(missing)
        return 0
