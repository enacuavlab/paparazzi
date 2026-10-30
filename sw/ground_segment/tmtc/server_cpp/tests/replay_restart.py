#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Exercise session replacement with a player started after server_cpp."""
import argparse
import json
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from ivy.std_api import (
    IvyApplicationConnected, IvyBindMsg, IvyGetApplication, IvyGetApplicationMessages,
    IvyGetApplicationName, IvyInit, IvySendMsg, IvyStart, IvyStop,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("repository", type=Path)
    parser.add_argument("--server-first", action="store_true")
    args = parser.parse_args()
    definitions = args.repository / "sw/ext/pprzlink/message_definitions/v1.0/messages.xml"
    protocol = ET.parse(definitions)
    schemas = {element.attrib["name"]: element for element in protocol.iter("message")}

    def message(sender, name, **values):
        fields = []
        for field in schemas[name].findall("field"):
            value = str(values.get(field.attrib["name"], 0))
            if field.attrib["type"] == "string":
                value = '"' + value + '"'
            fields.append(value)
        return " ".join([sender, name] + fields)

    changed = threading.Event()
    replies, flights, connections = {}, [], []

    def on_connection(app, event):
        if event == IvyApplicationConnected and IvyGetApplicationName(app) == "Paparazzi server":
            connections.append(app)
            changed.set()

    def reply(_app, request_id, body):
        replies[request_id] = body
        changed.set()

    def flight(_app, body):
        fields = [field.attrib["name"] for field in schemas["FLIGHT_PARAM"].findall("field")]
        flights.append(dict(zip(fields, body.split())))
        changed.set()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
        reservation.bind(("127.0.0.1", 0))
        bus = f"127.255.255.255:{reservation.getsockname()[1]}"
    with tempfile.TemporaryDirectory(prefix="server-replay-restart-") as temporary:
        home = Path(temporary)
        (home / "conf").mkdir()
        aircraft = home / "var/aircrafts/Live"
        aircraft.mkdir(parents=True)
        (home / "conf/conf.xml").write_text(
            '<conf><aircraft ac_id="6" name="Live" airframe="airframe.xml" radio="radio.xml"/></conf>')
        (home / "conf/airframe.xml").write_text('<airframe/>')
        (home / "conf/radio.xml").write_text('<radio/>')
        (aircraft / "flight_plan.xml").write_text('<dump><flight_plan lat0="43" lon0="1" ground_alt="100"/></dump>')
        (aircraft / "settings.xml").write_text('<settings/>')
        output = home / "output.txt"
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp, output.open("w") as stream:
            udp.bind(("127.0.0.1", 0))
            udp.settimeout(5)
            IvyInit("replay-session-test", "READY", 0, on_connection, lambda *_: None)
            IvyBindMsg(reply, r"^([^ ]*) ground AIRCRAFTS (.*)$")
            IvyBindMsg(reply, r"^([^ ]*) ground CONFIG (.*)$")
            IvyBindMsg(flight, r"^ground FLIGHT_PARAM (.*)$")
            if not args.server_first:
                IvyStart(bus)
            process = subprocess.Popen([
                str(args.executable), "--home", str(home), "--messages", str(definitions),
                "--bus", bus, "--verbose", "--no-md5-check", "--udp-json-stream-port",
                str(udp.getsockname()[1]),
            ], stdout=stream, stderr=subprocess.STDOUT)
            try:
                def wait_for(predicate, description):
                    deadline = time.monotonic() + 15
                    while not predicate():
                        assert process.poll() is None, output.read_text()
                        assert time.monotonic() < deadline, description + "\n" + output.read_text()
                        changed.wait(0.1)
                        changed.clear()

                def query(request_id, body):
                    def received():
                        if request_id in replies:
                            return True
                        IvySendMsg(f"gcs {request_id} {body}")
                        return False
                    wait_for(received, "No response for " + request_id)
                    return replies[request_id].strip()

                if args.server_first:
                    wait_for(lambda: "READY waiting for ALIVE" in output.read_text(), "Server not ready")
                    IvyStart(bus)
                wait_for(lambda: len(connections) == 1, "Initial connection")

                def bindings_ready():
                    app = IvyGetApplication("Paparazzi server")
                    bindings = {pattern for _, pattern in IvyGetApplicationMessages(app)} if app else set()
                    return r"^([^ ]*) (.*)$" in bindings and any("CONFIG_REQ" in pattern for pattern in bindings)
                wait_for(bindings_ready, "Generic bind and request answerers not advertised")
                assert query("empty", "AIRCRAFTS_REQ") == ","
                alive = {"md5sum": ",".join(["0"] * 16)}
                IvySendMsg(message("6", "ALIVE", **alive))
                wait_for(lambda: "REGISTERED aircraft 6 (Live)" in output.read_text(), "Live registration")
                assert query("live", "AIRCRAFTS_REQ") == "6,"
                pid = process.pid

                # Only now does the player prepare its files. Its GPS wire order
                # differs from the live schema, and it adds a telemetry message.
                replay = home / "var/replay"
                shutil.copytree(home / "conf", replay / "conf")
                shutil.copytree(home / "var/aircrafts", replay / "var/aircrafts")
                historical = ET.parse(definitions)
                telemetry = historical.find(".//msg_class[@name='telemetry']")
                gps = telemetry.find("message[@name='GPS_LLA']")
                lat, lon = gps.findall("field")[:2]
                gps.remove(lat)
                gps.remove(lon)
                gps.insert(0, lon)
                gps.insert(1, lat)
                used = {int(element.attrib["id"]) for element in telemetry.findall("message")}
                extra = ET.SubElement(telemetry, "message", name="REPLAY_ONLY",
                                      id=str(next(index for index in range(256) if index not in used)))
                ET.SubElement(extra, "field", name="value", type="float")
                historical.write(replay / "var/messages.xml")
                # One ALIVE only: the application must retain it over reconnect.
                IvySendMsg(message("replay6", "ALIVE", **alive))
                wait_for(lambda: "REPLAY session ready for replay6" in output.read_text(), "Replay restart")
                wait_for(lambda: len(connections) == 2, "Replay reconnection")
                assert query("replay-list", "AIRCRAFTS_REQ") == "replay6,"
                assert "/var/replay/var/aircrafts/Live/" in query("replay-config", "CONFIG_REQ replay6")
                schemas.update({element.attrib["name"]: element for element in historical.iter("message")})
                IvySendMsg(message("replay6", "GPS_LLA", lat=430000000, lon=10000000, mode=3))
                wait_for(lambda: any(item["ac_id"] == "replay6" and abs(float(item["lat"]) - 43) < 1e-6
                                     and abs(float(item["long"]) - 1) < 1e-6 for item in flights), "Historical GPS schema")
                IvySendMsg(message("replay6", "REPLAY_ONLY", value=42))
                while True:
                    packet = json.loads(udp.recv(65535))
                    if "REPLAY_ONLY" in packet.get("Live (replay6)", {}):
                        assert packet["Live (replay6)"]["REPLAY_ONLY"]["value"] == 42
                        break

                # Invalid replacement must leave the working session connected.
                archived = replay / "var/messages.xml"
                valid_xml = archived.read_text()
                archived.write_text("<protocol>")
                IvySendMsg(message("replay6", "ALIVE", **alive))
                wait_for(lambda: "Replay restart:" in output.read_text(), "Invalid replay definitions not rejected")
                assert query("still-running", "AIRCRAFTS_REQ") == "replay6,"
                assert len(connections) == 2
                archived.write_text(valid_xml)
                # Another log can reuse both the aircraft ID and message schema.
                (replay / "conf/conf.xml").write_text(
                    '<conf><aircraft ac_id="6" name="Second" airframe="airframe.xml" radio="radio.xml"/></conf>')
                second = replay / "var/aircrafts/Second"
                second.mkdir()
                (second / "flight_plan.xml").write_text('<dump><flight_plan lat0="55" lon0="20" ground_alt="200"/></dump>')
                (second / "settings.xml").write_text('<settings/>')
                flights.clear()
                IvySendMsg(message("replay6", "ALIVE", **alive))
                wait_for(lambda: len(connections) == 3, "Second-log reconnection")
                assert query("second-config", "CONFIG_REQ replay6").endswith(" Second")
                wait_for(lambda: any(item["ac_id"] == "replay6" and float(item["alt"]) == 200
                                     and float(item["speed"]) == 0 for item in flights), "Second log did not reset state/timers")
                # A config-only update must also trigger replacement; leave the
                # message file untouched, as many logs share the same protocol.
                message_time = archived.stat().st_mtime_ns
                (replay / "conf/conf.xml").write_text(
                    '<conf><aircraft ac_id="6" name="Third" airframe="airframe.xml" radio="radio.xml"/></conf>')
                third = replay / "var/aircrafts/Third"
                third.mkdir()
                (third / "flight_plan.xml").write_text('<dump><flight_plan lat0="56" lon0="21" ground_alt="300"/></dump>')
                (third / "settings.xml").write_text('<settings/>')
                flights.clear()
                IvySendMsg(message("replay6", "ALIVE", **alive))
                wait_for(lambda: len(connections) == 4, "Config-only reconnection")
                assert query("third-config", "CONFIG_REQ replay6").endswith(" Third")
                wait_for(lambda: any(item["ac_id"] == "replay6" and float(item["alt"]) == 300
                                     for item in flights), "Config-only restart lost timers/discovery")
                assert archived.stat().st_mtime_ns == message_time
                assert process.pid == pid and process.poll() is None
                for index in range(30):
                    IvySendMsg(f"gcs shutdown-{index} AIRCRAFTS_REQ")
                process.send_signal(signal.SIGINT if args.server_first else signal.SIGTERM)
                assert process.wait(timeout=5) == 0
                logs = list((home / "var/logs").glob("*.log"))
                assert len(logs) == 4, logs
                snapshots = [ET.parse(path).getroot() for path in logs]
                assert sum(root.find(".//message[@name='REPLAY_ONLY']") is not None for root in snapshots) == 3
                names = {root.find("conf/aircraft").attrib["name"] for root in snapshots}
                assert names == {"Live", "Second", "Third"}, names
                assert "STOP rx=" in output.read_text()
                print("Late replay, archived schemas, session reset, retry, log rotation and shutdown passed")
            except Exception:
                print(output.read_text(), flush=True)
                raise
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
                IvyStop()


if __name__ == "__main__":
    main()
