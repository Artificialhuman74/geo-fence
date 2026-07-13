"""Sound + desktop notification for geofence breach alerts.

The breach sound loops continuously for as long as the tracker stays
outside the fence (start_siren/stop_siren), running on its own background
thread so it never blocks the Firebase listener.
"""

import atexit
import os
import subprocess
import sys
import threading

_siren_thread = None
_siren_stop = threading.Event()
_siren_lock = threading.Lock()

# Kill any in-flight afplay on interpreter shutdown (e.g. Ctrl-C on
# server.py mid-breach) instead of letting the clip finish playing.
atexit.register(lambda: stop_siren())


def _play_once_blocking(sound_file: str) -> None:
    """Play sound_file start-to-finish, blocking the calling thread."""
    if sys.platform == "darwin":
        # afplay is stdlib-free, ships with macOS, and its handle can be
        # terminated instantly — unlike playsound, which offers no stop().
        proc = subprocess.Popen(["afplay", sound_file],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        while proc.poll() is None:
            if _siren_stop.wait(0.15):
                proc.terminate()
                return
        return
    from playsound import playsound
    playsound(sound_file)


def _siren_loop(sound_file: str) -> None:
    failures = 0
    while not _siren_stop.is_set():
        try:
            _play_once_blocking(sound_file)
            failures = 0
        except Exception as exc:
            failures += 1
            print(f"[alarm] siren playback failed ({exc}); falling back to terminal bell")
            sys.stdout.write("\a")
            sys.stdout.flush()
            if failures >= 3:
                # No audio device (headless/hosted host) — stop retrying
                # instead of spamming the logs once a second.
                print("[alarm] audio unavailable, giving up on server-side sound")
                return
            _siren_stop.wait(1.0)


def start_siren(sound_file: str) -> None:
    """Start looping sound_file in the background. Safe to call repeatedly
    (e.g. once per REPEAT_ALARM) — a loop already running is left alone."""
    global _siren_thread
    with _siren_lock:
        if _siren_thread is not None and _siren_thread.is_alive():
            return
        if not sound_file or not os.path.isfile(sound_file):
            print(f"[alarm] sound file not found: {sound_file!r}; "
                  "falling back to repeated terminal bell")
        _siren_stop.clear()
        _siren_thread = threading.Thread(
            target=_siren_loop, args=(sound_file,), daemon=True, name="siren")
        _siren_thread.start()


def stop_siren() -> None:
    global _siren_thread
    with _siren_lock:
        _siren_stop.set()
        thread, _siren_thread = _siren_thread, None
    if thread is not None:
        thread.join(timeout=2)


def notify(title: str, message: str) -> None:
    if sys.platform == "darwin":
        # plyer needs pyobjus on macOS; osascript is built in and reliable.
        script = f'display notification "{message}" with title "{title}"'
        try:
            subprocess.run(["osascript", "-e", script], check=False, timeout=5)
            return
        except Exception as exc:
            print(f"[alarm] osascript notification failed ({exc})")
    try:
        from plyer import notification
        notification.notify(title=title, message=message, timeout=10)
    except Exception as exc:
        print(f"[alarm] desktop notification failed ({exc})")


def trigger(sound_file: str, distance_m: float) -> None:
    """One-shot alert: used by the legacy headless dashboard.py. The web
    dashboard (server.py) uses start_siren/stop_siren instead so the sound
    loops for the whole breach instead of a single beep."""
    if sound_file and os.path.isfile(sound_file):
        _play_once_blocking(sound_file)
    else:
        sys.stdout.write("\a")
        sys.stdout.flush()
    notify(
        title="Geofence Alert",
        message=f"Device is {distance_m:.0f}m from home — outside geofence!",
    )
