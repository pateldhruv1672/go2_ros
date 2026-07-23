from __future__ import annotations

import argparse
import asyncio
import os
import signal
import subprocess
import sys
from typing import Optional


def _log(text: str) -> None:
    print(f"[omi_ble_preflight] {text}", flush=True)


def _run_bluetoothctl_disconnect(address: str) -> None:
    address = (address or "").strip()
    if not address:
        return

    try:
        subprocess.run(
            ["bluetoothctl", "disconnect", address],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=4.0,
            check=False,
        )
        _log(f"bluetoothctl_disconnect attempted address={address}")
    except Exception as exc:
        _log(f"bluetoothctl_disconnect_warning address={address} error={type(exc).__name__}: {exc}")


async def _find_device(address: str, name: str, scan_timeout_sec: float):
    from bleak import BleakScanner

    address = (address or "").strip()
    name = (name or "").strip().lower()

    # If an exact address is provided, ONLY use that address.
    # Do not fallback to random name matching.
    if address:
        _log(f"scanning_for_address address={address} timeout={scan_timeout_sec:.1f}s")
        device = await BleakScanner.find_device_by_address(
            address,
            timeout=scan_timeout_sec,
        )
        if device is not None:
            _log(f"found_by_address address={device.address} name={device.name}")
            return device

        _log(f"failed exact_address_not_found address={address}")
        return None

    # Only use name scan when no address is provided.
    if name:
        _log(f"scanning_for_name name='{name}' timeout={scan_timeout_sec:.1f}s")
        devices = await BleakScanner.discover(timeout=scan_timeout_sec)
        for device in devices:
            dev_name = (device.name or "").strip().lower()

            # Critical bug fix:
            # Do not allow empty/None device names to match.
            if not dev_name:
                continue

            if name == dev_name or name in dev_name or dev_name in name:
                _log(f"found_by_name address={device.address} name={device.name}")
                return device

    return None


async def _connect_once(device, connect_timeout_sec: float) -> bool:
    from bleak import BleakClient

    _log(
        f"connecting address={device.address} name={device.name} "
        f"timeout={connect_timeout_sec:.1f}s"
    )

    client = BleakClient(device, timeout=connect_timeout_sec)

    try:
        await asyncio.wait_for(client.connect(), timeout=connect_timeout_sec)

        connected = bool(client.is_connected)
        _log(f"connect_result connected={connected}")

        return connected

    except asyncio.TimeoutError:
        _log(
            f"failed connect_timeout address={device.address} "
            f"timeout={connect_timeout_sec:.1f}s"
        )
        return False

    except Exception as exc:
        _log(f"failed connect_exception address={device.address} error={type(exc).__name__}: {exc}")
        return False

    finally:
        try:
            if client.is_connected:
                _log(f"disconnecting_after_preflight address={device.address}")
                await asyncio.wait_for(client.disconnect(), timeout=5.0)
                _log(f"disconnected_after_preflight address={device.address}")
            else:
                _log(f"no_active_ble_connection_to_disconnect address={device.address}")
        except Exception as exc:
            _log(f"disconnect_warning address={device.address} error={type(exc).__name__}: {exc}")


async def _run_async(
    address: str,
    name: str,
    scan_timeout_sec: float,
    connect_timeout_sec: float,
    cleanup_before: bool,
    cleanup_after: bool,
) -> bool:
    address = (address or "").strip()

    try:
        if cleanup_before and address:
            _run_bluetoothctl_disconnect(address)

        device = await _find_device(address, name, scan_timeout_sec)
        if device is None:
            _log(
                "failed device_not_found "
                f"address='{address}' name='{name}'"
            )
            return False

        ok = await _connect_once(device, connect_timeout_sec)
        return bool(ok)

    except ModuleNotFoundError as exc:
        _log(f"failed missing_python_dependency error={exc}")
        _log("install with: python -m pip install bleak")
        return False

    except Exception as exc:
        _log(f"failed exception={type(exc).__name__}: {exc}")
        return False

    finally:
        if cleanup_after and address:
            _run_bluetoothctl_disconnect(address)


def _select_child_python() -> str:
    venv = os.environ.get("VIRTUAL_ENV", "").strip()
    if venv:
        candidate = os.path.join(venv, "bin", "python")
        if os.path.exists(candidate):
            return candidate

    env_python = os.environ.get("PYTHON", "").strip()
    if env_python and os.path.exists(env_python):
        return env_python

    return sys.executable


def _run_child_preflight(
    address: str,
    name: str,
    scan_timeout_sec: float,
    connect_timeout_sec: float,
    cleanup_before: bool,
    cleanup_after: bool,
) -> bool:
    child_python = _select_child_python()

    cmd = [
        child_python,
        "-m",
        "go2_omi_voice_bridge.omi_ble_preflight",
        "--child",
        "--address",
        str(address or ""),
        "--name",
        str(name or ""),
        "--scan-timeout-sec",
        str(float(scan_timeout_sec)),
        "--connect-timeout-sec",
        str(float(connect_timeout_sec)),
    ]

    if cleanup_before:
        cmd.append("--cleanup-before")
    if cleanup_after:
        cmd.append("--cleanup-after")

    _log(f"using_child_python python={child_python}")

    env = os.environ.copy()

    # Parent ROS launch may run outside the venv.
    # Keep package import available for child Python.
    workspace_install = os.path.abspath("install/go2_omi_voice_bridge/lib/python3.12/site-packages")
    existing_pythonpath = env.get("PYTHONPATH", "")
    if os.path.isdir(workspace_install) and workspace_install not in existing_pythonpath:
        env["PYTHONPATH"] = workspace_install + (":" + existing_pythonpath if existing_pythonpath else "")

    # Give the child enough time to cleanly disconnect.
    # The child itself enforces scan/connect/disconnect timeouts.
    parent_timeout = float(scan_timeout_sec) + float(connect_timeout_sec) + 25.0

    proc = None
    try:
        proc = subprocess.Popen(
            cmd,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )

        try:
            stdout, _ = proc.communicate(timeout=parent_timeout)
        except subprocess.TimeoutExpired:
            _log(f"failed child_parent_timeout after_sec={parent_timeout:.1f}; terminating_child_gracefully")

            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    stdout, _ = proc.communicate(timeout=8.0)
                except subprocess.TimeoutExpired:
                    _log("child_did_not_exit_after_sigint; killing_child")
                    proc.kill()
                    stdout, _ = proc.communicate(timeout=5.0)

            if stdout:
                print(stdout, end="", flush=True)

            # Always attempt target cleanup after a timed-out child.
            if address:
                _run_bluetoothctl_disconnect(address)

            return False

        if stdout:
            print(stdout, end="", flush=True)

        if proc.returncode == 0:
            _log("child_preflight_ok")
            return True

        _log(f"child_preflight_failed returncode={proc.returncode}")

        if address:
            _run_bluetoothctl_disconnect(address)

        return False

    except Exception as exc:
        _log(f"failed child_exception={type(exc).__name__}: {exc}")
        if address:
            _run_bluetoothctl_disconnect(address)
        return False


def run_omi_ble_preflight(
    address: str,
    name: str,
    scan_timeout_sec: float = 12.0,
    connect_timeout_sec: float = 10.0,
    cleanup_before: bool = True,
    cleanup_after: bool = True,
) -> bool:
    return _run_child_preflight(
        address=address,
        name=name,
        scan_timeout_sec=float(scan_timeout_sec),
        connect_timeout_sec=float(connect_timeout_sec),
        cleanup_before=bool(cleanup_before),
        cleanup_after=bool(cleanup_after),
    )


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--child", action="store_true")
    parser.add_argument("--address", default="")
    parser.add_argument("--name", default="Omi")
    parser.add_argument("--scan-timeout-sec", type=float, default=12.0)
    parser.add_argument("--connect-timeout-sec", type=float, default=10.0)
    parser.add_argument("--cleanup-before", action="store_true")
    parser.add_argument("--cleanup-after", action="store_true")
    args = parser.parse_args(argv)

    if args.child:
        ok = asyncio.run(
            _run_async(
                address=args.address,
                name=args.name,
                scan_timeout_sec=args.scan_timeout_sec,
                connect_timeout_sec=args.connect_timeout_sec,
                cleanup_before=args.cleanup_before,
                cleanup_after=args.cleanup_after,
            )
        )
        return 0 if ok else 2

    ok = run_omi_ble_preflight(
        address=args.address,
        name=args.name,
        scan_timeout_sec=args.scan_timeout_sec,
        connect_timeout_sec=args.connect_timeout_sec,
        cleanup_before=args.cleanup_before,
        cleanup_after=args.cleanup_after,
    )
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
