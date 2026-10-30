#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Run a real Ivy peer against server_cpp (requires python3-ivy and loopback networking)."""
import argparse
import json
from pathlib import Path
import signal
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from ivy.std_api import (
    IvyApplicationConnected, IvyBindMsg, IvyGetApplication, IvyGetApplicationMessages, IvyGetApplicationName,
    IvyInit, IvySendMsg, IvyStart, IvyStop,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("repository", type=Path)
    parser.add_argument("--server-first", action="store_true")
    args = parser.parse_args()
    executable, repository = args.executable, args.repository
    definitions = repository / "sw/ext/pprzlink/message_definitions/v1.0/messages.xml"
    schema = {m.attrib["name"]: m for m in ET.parse(definitions).iter("message")}

    def message(sender, name, **values):
        fields = []
        for field in schema[name].findall("field"):
            key, kind = field.attrib["name"], field.attrib["type"]
            value = values.get(key, "" if kind == "string" else 0)
            if kind == "string":
                value = '"' + str(value) + '"' if not value or " " in str(value) else str(value)
            fields.append(str(value))
        return " ".join([sender, name] + fields)

    received = {}
    changed = threading.Event()

    def capture(name):
        def callback(_app, body):
            received[name] = body
            changed.set()
        return callback

    def wait(name, send=None):
        deadline = time.monotonic() + 12
        while name not in received:
            if time.monotonic() > deadline:
                raise AssertionError(f"Timed out waiting for {name}; received {received}")
            if send:
                IvySendMsg(send)
            changed.wait(0.2)
            changed.clear()
        return received[name]

    def on_connection(app, event):
        # PprzGCS uses this exact application name to initiate discovery.
        if event == IvyApplicationConnected and IvyGetApplicationName(app) == "Paparazzi server":
            IvySendMsg("pprzcontrol startup-test AIRCRAFTS_REQ ")

    # Reserve an ephemeral UDP port for this isolated Ivy bus.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        bus_port = reservation.getsockname()[1]
    bus = f"127.255.255.255:{bus_port}"
    expected_messages = {"ALIVE", "AIRCRAFT_DIE", "INTRUDER", "MOVE_WAYPOINT", "DL_EMERGENCY_CMD",
                         "DL_SETTING", "GET_DL_SETTING", "JUMP_TO_BLOCK", "RAW_DATALINK", "INFO_MSG_GROUND",
                         "LINK_REPORT", "AIRCRAFTS_REQ", "CONFIG_REQ"}
    with tempfile.TemporaryDirectory(prefix="server-cpp-ivy-") as temporary:
        home = Path(temporary)
        (home / "conf").mkdir()
        aircraft = home / "var/aircrafts/Test"
        aircraft.mkdir(parents=True)
        (home / "conf/conf.xml").write_text('<conf><aircraft ac_id="6" name="Test" airframe="airframe.xml" radio="radio.xml"/></conf>')
        (home / "conf/airframe.xml").write_text('<airframe><section name="BAT"/></airframe>')
        (home / "conf/radio.xml").write_text('<radio/>')
        (aircraft / "flight_plan.xml").write_text('<dump><flight_plan lat0="43" lon0="1" ground_alt="100"/></dump>')
        (aircraft / "settings.xml").write_text('<settings/>')
        replay = home / "var/replay"
        shutil.copytree(home / "conf", replay / "conf")
        shutil.copytree(home / "var/aircrafts", replay / "var/aircrafts")
        shutil.copyfile(definitions, replay / "var/messages.xml")
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            udp.bind(("127.0.0.1", 0))
            udp.settimeout(5)
            IvyInit("server-cpp-test", "test READY", 0, on_connection, lambda *_: None)
            for name in ["NEW_AIRCRAFT", "FLIGHT_PARAM", "ENGINE_STATUS", "TELEMETRY_STATUS"]:
                IvyBindMsg(capture(name), rf"^ground {name} (.*)$")
            IvyBindMsg(capture("CONFIG"), r"^config-test ground CONFIG (.*)$")
            IvyBindMsg(capture("AIRCRAFTS"), r"^list-test ground AIRCRAFTS (.*)$")
            IvyBindMsg(capture("BATCH"), r"^batch-19 ground AIRCRAFTS (.*)$")
            IvyBindMsg(capture("STARTUP"), r"^startup-test ground AIRCRAFTS (.*)$")
            IvyBindMsg(capture("SETTING"), r"^ground_dl SETTING (.*)$")
            IvyBindMsg(capture("REPLAY"), r"^ground NEW_AIRCRAFT (replay6)$")
            IvyBindMsg(capture("REPLAY_CONFIG"), r"^replay-test ground CONFIG (.*)$")
            if not args.server_first:
                IvyStart(bus)
            output = home / "output.txt"
            try:
                with output.open("w") as stream:
                    process = subprocess.Popen([str(executable), "--home", str(home), "--messages", str(definitions),
                        "-b", bus, "--verbose", "--no-md5-check", "--udp-json-stream-port", str(udp.getsockname()[1])],
                        stdout=stream, stderr=subprocess.STDOUT)
                    try:
                        deadline = time.monotonic() + 12
                        if args.server_first:
                            while "READY waiting for ALIVE" not in output.read_text():
                                assert time.monotonic() < deadline, output.read_text()
                                time.sleep(0.1)
                            IvyStart(bus)
                        while True:
                            app = IvyGetApplication("Paparazzi server")
                            bindings = {regexp for _, regexp in IvyGetApplicationMessages(app)} if app else set()
                            missing = {name for name in expected_messages if not any(name in regexp for regexp in bindings)}
                            if not missing:
                                break
                            assert time.monotonic() < deadline, f"Missing advertised subscriptions: {missing}"
                            time.sleep(0.1)
                        assert not ({r"^([^ ]+) (.*)$", r"^([^ ]*) (.*)$"} & bindings), bindings
                        assert wait("STARTUP").strip() == ",", received
                        # With no aircraft yet, verbose mode must explain the idle state.
                        while "[server_cpp] WAIT no registered aircraft" not in output.read_text():
                            assert time.monotonic() < deadline, output.read_text()
                            time.sleep(0.1)
                        IvySendMsg(message("99", "ALIVE", md5sum=",".join(["0"] * 16)))
                        wait("NEW_AIRCRAFT", message("6", "ALIVE", md5sum=",".join(["0"] * 16)))
                        IvySendMsg(message("6", "GPS_LLA", lat=430000000, lon=10000000, speed=1234, course=900, mode=3, itow=1000))
                        IvySendMsg(message("6", "ESTIMATOR", z=150, z_dot=2))
                        wait("FLIGHT_PARAM")
                        # Confirm timers continue after the updates have been delivered.
                        time.sleep(0.7)
                        flight = received["FLIGHT_PARAM"].split()
                        fields = [f.attrib["name"] for f in schema["FLIGHT_PARAM"].findall("field")]
                        flight = dict(zip(fields, flight))
                        assert abs(float(flight["lat"]) - 43) < 1e-6, flight
                        assert abs(float(flight["alt"]) - 150) < 1e-5, flight
                        assert abs(float(flight["speed"]) - 12.34) < 1e-4, flight
                        config = wait("CONFIG", "gcs config-test CONFIG_REQ 6")
                        assert "file://" in config and "Test" in config, config
                        assert wait("AIRCRAFTS", "gcs list-test AIRCRAFTS_REQ").strip() == "6,"
                        setting = wait("SETTING", message("gcs", "DL_SETTING", ac_id="6", index=2, value=1.25))
                        index, aircraft_id, value = setting.split()
                        assert (int(index), int(aircraft_id), float(value)) == (2, 6, 1.25), setting
                        packet = json.loads(udp.recv(65535))
                        assert "Test (6)" in packet and "timestamp" in packet, packet
                        assert isinstance(packet["Test (6)"]["ALIVE"]["md5sum"], dict), packet
                        wait("REPLAY", message("replay6", "ALIVE", md5sum=",".join(["0"] * 16)))
                        replay_config = wait("REPLAY_CONFIG", "gcs replay-test CONFIG_REQ replay6")
                        assert "/var/replay/var/aircrafts/Test/" in replay_config, replay_config
                        # Consecutive synchronous answerers must hand work to the
                        # application thread without blocking its periodic jobs.
                        for index in range(20):
                            IvySendMsg(f"gcs batch-{index} AIRCRAFTS_REQ")
                        assert wait("BATCH").strip() == "6,replay6,"
                        # Shutdown while requests may still be queued must release
                        # the transport's pending reply and join promptly.
                        for index in range(50):
                            IvySendMsg(f"gcs shutdown-{index} AIRCRAFTS_REQ")
                        process.send_signal(signal.SIGINT if args.server_first else signal.SIGTERM)
                        assert process.wait(timeout=5) == 0
                        logs = list((home / "var/logs").glob("*.data"))
                        assert len(logs) == 1
                        estimator_logs = [line.split() for line in logs[0].read_text().splitlines()
                                          if line.split()[1:3] == ["6", "ESTIMATOR"]]
                        assert estimator_logs and float(estimator_logs[0][0]) >= 0, estimator_logs
                        metadata = list((home / "var/logs").glob("*.log"))
                        assert ET.parse(metadata[0]).getroot().tag == "configuration"
                    finally:
                        if process.poll() is None:
                            process.terminate()
                            try:
                                process.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait()
                assert "Message from" not in output.read_text(), output.read_text()
                diagnostics = output.read_text()
                for marker in ["START home=", "LOAD messages ", "BIND message ALIVE",
                               "RX 99 ALIVE", "Register 99:", "REGISTERED aircraft 6 (Test)",
                               "RX 6 GPS_LLA", "RX REQUEST gcs CONFIG_REQ",
                               "TX FLIGHT_PARAM", "STATUS rx=", "peers=[server-cpp-test]", "STOP rx="]:
                    assert marker in diagnostics, f"Missing diagnostic {marker}: {diagnostics}"
            except Exception:
                print(output.read_text() if output.exists() else "No server output", file=sys.stderr)
                raise
            finally:
                IvyStop()
    print("Ivy discovery, telemetry, timers, requests, uplink, JSON, logging, replay and shutdown passed")


if __name__ == "__main__":
    main()
