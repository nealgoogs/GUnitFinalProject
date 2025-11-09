# I2C_mutex.py  — minimal drop-in for GoPiGo3/easysensors
# Provides: class Mutex with acquire()/release() and context manager support.

import os
import sys
import time
import threading

try:
    import fcntl  # POSIX file locking
    _HAVE_FCNTL = True
except Exception:
    _HAVE_FCNTL = False

class Mutex:
    """
    Simple cross-process mutex for I²C access.
    Usage:
        m = Mutex()
        m.acquire()
        ... do i2c work ...
        m.release()
    Or:
        with Mutex():
            ... do i2c work ...
    """
    def __init__(self, debug=False, lockfile="/tmp/i2c_mutex.lock", timeout=None, poll_interval=0.05):
        self.debug = debug
        self.lockfile = lockfile
        self.timeout = timeout  # seconds or None
        self.poll_interval = poll_interval
        self._fh = None
        self._tlock = threading.Lock()  # fallback for no-fcntl case

    def acquire(self):
        if _HAVE_FCNTL:
            start = time.time()
            self._fh = open(self.lockfile, "w+")
            while True:
                try:
                    fcntl.flock(self._fh, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    if self.debug:
                        print("[I2C_mutex] acquired (fcntl)", file=sys.stderr)
                    return True
                except BlockingIOError:
                    if self.timeout is not None and (time.time() - start) > self.timeout:
                        if self.debug:
                            print("[I2C_mutex] acquire timeout", file=sys.stderr)
                        return False
                    time.sleep(self.poll_interval)
        else:
            # Fallback: per-process lock only (won’t protect across processes)
            got = self._tlock.acquire(timeout=self.timeout if self.timeout else -1)
            if self.debug and got:
                print("[I2C_mutex] acquired (threading)", file=sys.stderr)
            return got

    def release(self):
        try:
            if _HAVE_FCNTL and self._fh is not None:
                fcntl.flock(self._fh, fcntl.LOCK_UN)
                self._fh.close()
                self._fh = None
                if self.debug:
                    print("[I2C_mutex] released (fcntl)", file=sys.stderr)
            else:
                if self._tlock.locked():
                    self._tlock.release()
                    if self.debug:
                        print("[I2C_mutex] released (threading)", file=sys.stderr)
        except Exception:
            pass

    # Context manager support
    def __enter__(self):
        ok = self.acquire()
        if not ok:
            raise TimeoutError("I2C mutex acquire timed out")
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        # Do not suppress exceptions
        return False
