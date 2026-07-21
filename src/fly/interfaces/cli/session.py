# Shared helpers

import re
from pathlib import Path
from sqlite3.dbapi2 import connect

from fly.core.dataManager import get_setting, pull_data, update_port_data, update_setting
from fly.core.drone import Drone
from fly.core.mission import Mission

DEFAULT_PORT = "udpin://0.0.0.0:1450"

_UDP_RE = re.compile(r"^udp(?:in|out)?://([0-9]{1,3}\.){3}[0-9]{1,3}:[0-9]{1,5}$")
_TCP_RE = re.compile(r"^udp(?:in|out)?://([0-9]{1,3}\.){3}[0-9]{1,3}:[0-9]{1,5}$")
_SERIAL_RE = re.compile(r"^serial://(/dev/[a-zA-Z0-9_-]+|COM[0-9]+)(:[0-9]+)?$")

def validate_port_format(port: str) -> bool:
    # mirrors the port validation the old CLI attempted
    return bool(_UDP_RE.match(port) or _TCP_RE.match(port) or _SERIAL_RE.match(port))

def resolve_port(port: str | None) -> str:
    # --port flag wins; otherwise fall back to the last-successful port saved
    # in fly/config/settings.json; otherwise the standard SITL
    if port:
        return port

    data = pull_data() or {}
    saved = data.get("port")
    if saved:
        print(f"-- No `--port` given, using last-connected port: {saved}")
        return saved

    print(f"-- No `--port` given and no saved port on record, defaulting to {DEFAULT_PORT}")
    return DEFAULT_PORT

def _remember_port(port: str) -> None:
    data = pull_data() or {}
    history = data.get("port-history", [])
    if port not in history:
        history.append(port)
    update_port_data(port=port, history=history)

def resolve_setting(value, key: str, default=None, *, quiet: bool = False):
    # generic version of resolve_port()
    if value is not None:
        return value

    saved = get_setting(key)
    if saved is not None:
        if not quiet:
            flag = key.replace("_", "-")
            print(f"-- No `--{flag}` given, using last-used value: {saved}")
        return saved

    return default

def remember_setting(key: str, value) -> None:
    # Persist a value under `key` in settings.json for resolve_setting() to find next time
    if value is None:
        return
    update_setting(key,value)

async def get_connected_drone(port: str | None, *, timeout: int=10) -> Drone | None:
    # resolves a port, validates, connects, remembers it on success
    resolved = resolve_port(port)

    if not validate_port_format(resolved):
        print(
            f"-- Invalid port format: {resolved!r}. Expected udp(in|out)://host:port, "
            "tcp(in|out)://host:port, or serial://dev/tty...[:baud]."
        )
        return None

    drone = Drone(resolved, connection_timeout=timeout)
    print(f"-- Connecting to {resolved} ...")
    connected = await drone.connect()
    if not connected:
        print("-- Connection failed; consider trying a different --port.")
        return None

    _remember_port(resolved)
    return drone

def load_mission(file: str | Path) -> Mission | None:
    # parses json into Mission
    path = Path(file)
    if not path.exists():
        print(f"-- Mission file not found: {path}")
        return None
    try:
        return Mission(file=str(path))
    except Exception as e:
        print(f"-- Failed to load mission {path}: {e}")
        return None
