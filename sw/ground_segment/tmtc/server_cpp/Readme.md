
`cmake .. -Dpprzlink++_DIR=/home/fabien/dev/pprz/pprzlink/lib/v2.0/C++/build`

# C++ ground server

Standalone C++ implementation of the ground server. Aircraft state and periodic
work run on one application thread. Messaging uses PprzLink messages through
`PprzConnection`; its current backend uses `pprzlink::IvyLink`. The modem/transport
agent (`link`, `pprzlink_router`, etc.) handles the aircraft connection.
Run this server in place of the OCaml server on a bus, rather than running both
and duplicating ground publications and commands.

## Reading the code

Start with `main.cpp`, then `application.cpp`, then `server.cpp`.
`application.cpp` connects incoming messages and periodic jobs to `Server`.
`server.cpp` contains the receive-and-dispatch flow; its methods are implemented
in files named after their responsibility. These are ordinary methods of one
class sharing its aircraft map, without a hierarchy of subsystem objects.

| File | Responsibility |
| --- | --- |
| `main.cpp`, `options.h` | CLI options and defaults |
| `application.cpp` | Startup, subscriptions, periodic jobs, diagnostics and shutdown |
| `server.h`, `server.cpp` | Server entry points, shared state and incoming-message dispatch |
| `aircraft.h` | Plain aircraft, navigation, engine, radio and link state |
| `configuration.h`, `configuration.cpp` | Independent configuration lookup, paths, XML parameters, signatures and replay snapshots |
| `server_configuration.cpp` | Register aircraft and answer PprzLink aircraft/config requests |
| `telemetry.cpp` | Apply decoded aircraft telemetry to state |
| `commands.cpp` | Forward ground commands as datalink messages |
| `publishing.cpp` | Build periodic ground messages from aircraft state |
| `monitoring.cpp` | Battery/proximity alerts, intruder expiry and wind estimates |
| `logging.cpp`, `output_resources.h` | Flight data logs and owned output resources |
| `telemetry_json.cpp` | Convert decoded telemetry to JSON |
| `geodesy.cpp` | PROJ coordinate conversions and geodesic calculations |
| `pprz_connection.h`, `pprz_connection_ivy.cpp` | PprzLink messaging boundary and current transport backend |

For example, an `ALIVE` message reaches `Server::receive()`, which registers the
aircraft using `server_configuration.cpp` and the independent `Configuration`
object. One `bind(pprzlink::ALL, ...)` callback receives decoded messages;
registration announces `NEW_AIRCRAFT`. Subsequent telemetry
updates its `Aircraft` in `telemetry.cpp`. Every 500 ms, `broadcast()` in
`publishing.cpp` reads that state and sends ground messages such as `FLIGHT_PARAM`.
Ground commands take the shorter path through `commands.cpp` to datalink output.

### Configuration ownership

`Application` owns `paparazzi::Configuration` and passes a const reference to
`Server`. Future HTTP handlers can use the same object without going through
server state or PprzLink requests:

```cpp
paparazzi::Configuration configuration(home);
const auto aircraft = configuration.aircraft("6"); // Owned metadata and paths
const auto parameters = configuration.parameters(aircraft);
```

`aircraft()` resolves live/replay paths and GUI metadata. `parameters()` reads the
generated flight plan reference, literal battery/camera defines and autopilot
mode names. It prefers a built airframe copy when present, while the source
path stays available for CONFIG replies. Missing optional defines leave the
caller in charge of defaults. `signature()` reads the stored MD5; the server
compares it with ALIVE. `writeSnapshot()` creates the XML replay snapshot.

The component accepts a home path, uses only the standard library and tinyxml2,
and has no dependency on `Server`, `Options`, runtime `Aircraft`, PprzLink, Asio
or PROJ. It remains part of this program, with no separately packaged library.
Live conf.xml is loaded once; replay conf.xml is read on each lookup, because
replay files can appear or change after startup. File contents are read when
parameters, signatures or snapshots are requested. Returned metadata owns its
data and does not borrow pointers into XML documents.

This separates the existing file handling. Airframe include expansion and
flight-plan procedure resolution have not been added yet.

### Execution and transport boundary

```text
PprzLink Ivy thread -> SDK Asio dispatch -> application io_context
                                         | update aircraft
Asio steady_timer -----------------------| publish messages
Asio signal_set -------------------------| stop on SIGINT/SIGTERM
                                         | send via PprzConnection
```

`Application` owns one Boost.Asio `io_context`, run on the application thread.
Three `steady_timer` objects trigger broadcasts every 500 ms, alerts every
second, and wind/status every five seconds. The small `repeat()` helper re-arms
those timers, preserving cadence and skipping missed deadlines. There is no
custom event loop, scheduler, task queue or signal polling. Wall-clock time is
still used for flight logs and GPS time.

The backend passes the application context to PprzLink's Asio-aware `IvyLink`.
It owns one thread to service the native Ivy loop and report native loop errors.
PprzLink posts decoded messages and request answerers onto the application
context and sends correlated replies there, without blocking the Ivy thread.
All server callbacks and periodic jobs execute on the application thread;
aircraft state needs no mutex. The server has no callback-posting bridge,
request futures or request-cancellation mutex.

SIGINT/SIGTERM stop the context through an asynchronous `signal_set` handler.
Shutdown calls `IvyLink::close()` to suppress queued SDK callbacks and replies,
then joins the native thread before destroying server state and output
resources. Closing matters even after the context has stopped: Asio retains
queued handlers, and the SDK makes them inert. The same cleanup runs if a
handler throws.

Future HTTP or broadcasting components can share this context. They can read
state or call server methods on the application thread, with no extra state
synchronization. Keep their network operations asynchronous and handlers short.

`PprzConnection` exposes PprzLink messages, one generic `bind()` callback and
request answerers. `IvyLink::run()` activates the generic receiver's catch-all;
`RequestReply::bindRequest()` handles request correlation separately. The SDK
owns request handlers until the link closes; no Ivy subscription tokens are
retained by the application.
Only `pprz_connection_ivy.cpp` includes `IvyLink` or uses native Ivy APIs; replacing
the transport means replacing this backend and its build dependency. The current
`--bus`/`IVYBUS` setting and the peer name `Paparazzi server` remain for GCS
compatibility. `legacy_text.cpp` separately wraps the SDK's historical text codec
for `.data` files and `RAW_DATALINK`; that file grammar does not require an Ivy
connection. This change isolates Ivy; it does not provide a second transport.

## Build

Requires CMake 3.20+, a C++23 compiler, tinyxml2, CLI11, Boost headers (Asio), PROJ 8+ (`libproj-dev`),
and the recent PprzLink C++ SDK
with `pprzlink::core`, `pprzlink::ivy`, the Asio-aware `IvyLink` constructor,
and Ivy C++ 3.18+.
This is the same external SDK used by the original example in this directory;
the older SDK under `sw/ext/pprzlink` does not provide these APIs.

```sh
cmake -S sw/ground_segment/tmtc/server_cpp \
      -B sw/ground_segment/tmtc/server_cpp/build \
      -Dpprzlink++_DIR=/path/to/pprzlink/lib/v2.0/C++/build
cmake --build sw/ground_segment/tmtc/server_cpp/build -j
ctest --test-dir sw/ground_segment/tmtc/server_cpp/build --output-on-failure
```

On this workstation the SDK package directory is
`/home/fabien/dev/pprz/pprzlink/lib/v2.0/C++/build`.

## Run

```sh
PAPARAZZI_HOME="$PWD" sw/ground_segment/tmtc/server_cpp/build/server_cpp
```

`--home DIR` overrides `PAPARAZZI_HOME`. The default schema is
`HOME/var/messages.xml`; `--messages FILE` overrides it. Aircraft must be listed
in `conf/conf.xml` and have generated `var/aircrafts/NAME/flight_plan.xml` and
configuration files, just as for the OCaml server. Configuration signatures are
checked by default against `var/aircrafts/NAME/conf/aircraft.md5`.

CLI11 parses options and generates `--help`. Long options use double dashes
and hyphen-separated names: `--log-name`, `--no-md5-check`,
`--replay-old-log`, `--http`, `--hostname`, `--port`, and
`--udp-json-stream-{addr,port,disable}`. Short options are `-b` (`--bus`),
`-v` (`--verbose`), `-n` (`--no-log`), and `-h` (`--help`).
`IVYBUS` sets the default bus. `--http` returns URLs for an external HTTP server.
SIGINT/SIGTERM stop the event loop and close logs.
The Ivy application name is exactly `Paparazzi server`: PprzGCS uses this name
to recognize a server connection and request the existing aircraft list. This
works whether the server or GCS starts first.

## Diagnose communication

```sh
sw/ground_segment/tmtc/server_cpp/build/server_cpp --verbose -b 127.255.255.255:2010
```

`-v` is an alias for `--verbose`. Diagnostics go to stderr and include:

- `START`, `LOAD`, `LOG`, `JSON`: the actual paths, destinations and options used.
- `BIND`: the generic receiver and the two request answerers installed per session.
- `REPLAY`: scheduling, replacing and starting a replay session. Preparation
  errors are reported even without verbose mode; another discovery can retry.
- `RX` / `RX REQUEST`: decoded incoming messages. Request IDs are managed by
  `IvyLink` rather than the application.
- `REGISTER`, `CONFIG`, `MD5`, `REGISTERED`: registration progress, expected and
  received signatures, and the flight plan/airframe paths being loaded.
  Registration errors remain visible without verbose mode.
- `TX` / `TX RESPONSE`: outgoing message fields handed to `IvyLink`. Its send
  API reports errors but does not expose the number of matching subscribers.
- `STATUS`: receive/send counts, aircraft count and connected Ivy peers every
  five seconds. `WAIT` explains when no aircraft has registered; `AIRCRAFT`
  reports how long it has been since telemetry was last received.

If `peers=[]`, check the bus address and running link/GCS agents. With peers but
no `RX ... ALIVE`, check the link's aircraft telemetry. If `ALIVE` arrives but
registration fails, the following path, configuration or MD5 error identifies
the failure. Once registered, check telemetry age. The generic receiver decodes
all known bus messages; `Server` filters them by class, registration and command
name. Verbose mode changes only diagnostics, not reception or logging scope.

## Implemented

- Aircraft discovery on `ALIVE`, duplicate suppression, removal/re-registration,
  signature checks, and `NEW_AIRCRAFT` notifications.
- `AIRCRAFTS_REQ` / `CONFIG_REQ`, with Ivy request IDs and file/HTTP URLs.
- Fixed-wing and rotorcraft position, attitude, navigation, autopilot, energy,
  RC, satellite, setting and waypoint telemetry. PROJ supplies WGS84 UTM,
  ECEF/ENU and geographic offset conversions. UTM northings remain signed south
  of the equator, as required by Paparazzi. ECEF/ENU use ellipsoidal altitude;
  aircraft telemetry retains its HMSL correction and ground reference.
- Ground flight, engine, autopilot, RC, camera, navigation, survey, satellite,
  waypoint, calibration, settings and link-status broadcasts every 500 ms.
- Waypoint moves, settings, setting reads, block jumps, emergency commands,
  informational messages and raw datalink forwarding, using `ground_dl` as sender.
- Battery and proximity alerts every second; multi-aircraft traffic and external
  intruders (expired after 10 seconds); formation, TCAS and drawing forwarding.
- Wind estimates every five seconds and `WIND_CLEAR`.
- `.data` flight logs and `.log` XML snapshots, including protocol definitions,
  source airframe/radio, generated flight plans and settings for log replay.
  `replayID` aircraft use `var/replay` configuration and schemas. Log and JSON
  timestamps use elapsed time since the current session started at reception.
- UDP JSON telemetry, defaulting to `127.0.0.1:9870`. Strings are escaped and
  arrays retain the OCaml server's indexed JSON object convention. The socket is
  configured after option parsing, so destination overrides take effect.

## Replay sessions

The server can start before the player. On a `replayID ALIVE` (or `PPRZ_MODE`
with `--replay-old-log`), it checks the modification times of the player's
`var/replay/var/messages.xml` and `var/replay/conf/conf.xml`. The first replay
or a change to either file schedules a messaging-session restart. No filesystem
polling or player modification is required. Discovery messages must retain a
compatible schema so the current dictionary can decode the trigger.

A session groups its dictionary, flight log, `Server` state and `PprzConnection`.
The restart first validates replay definitions/configuration, then closes and
joins the old link, releases old state, and builds a session using the archived
dictionary. Invalid preparation leaves the working session intact for a later
retry. Recreating a session also clears registrations, settings and telemetry
from the previous live flight or replay log.

The restart is posted to Asio, so link destruction happens after the detecting
SDK callback returns. The triggering discovery message is copied and processed
in the new session, even if sent only once. Other telemetry during reconnection
may be missed. The GCS sees the exact `Paparazzi server` peer reconnect and can
request the aircraft list again; an early `NEW_AIRCRAFT` announcement may precede
peer reconnection, so that list is the reliable discovery path across a restart.

The application's Asio context, timers, JSON output socket and independent
configuration component stay alive. Logs rotate per session and replay snapshots
use the archived protocol and replay configuration. All replay telemetry is
decoded with one archived dictionary through `bind()`; request answerers use
`bindRequest()`. Live telemetry is ignored during a replay session.
Future HTTP handlers should obtain the current session's server for each request,
rather than retaining a reference across session replacement.
Returning to live operation currently requires restarting the program.

## Differences from the OCaml server

Messages are sent using `IvyLink`'s standard serialization, whose numeric
formatting can differ from the OCaml XML display formats. The legacy text codec
is used only for `.data` logs and `RAW_DATALINK` text; Ivy traffic and request
envelopes use `IvyLink` directly.

This is not a byte-for-byte compatibility port. KML output (`-kml` and
`-kml_no_http`) and SRTM terrain lookup are not implemented; AGL and waypoint
ground altitude use the aircraft's ground reference. Airframe XML includes and
expressions are not expanded: battery/camera defines and custom autopilot mode
names are read from literal XML. Log snapshots retain source airframes rather
than embedding the complete OCaml configuration expansion.

Wind uses a small algebraic circle fit to heading-binned ground-speed vectors
instead of the OCaml optimizer. Geographic navigation references and camera
offsets use PROJ's WGS84 topocentric transformations. The camera footprint still
assumes a flat ground plane. GPS time of week is mapped to the nearest week to
reception. Proximity alerts use PROJ's ellipsoidal geodesic distance and a short
geodesic velocity projection, including across the dateline.
Unrecognized telemetry is still logged/streamed but does not update state.
Malformed messages are reported on stderr; non-finite telemetry also produces
`TELEMETRY_ERROR`. Validation covers fixtures and a local Ivy peer, not flight
hardware or every historical log format.

## Integration test

A configuration CTest compiles the component with only tinyxml2, checking
paths, built/source airframes, parameters, replay isolation, signatures and
snapshots. The core CTest uses temporary aircraft fixtures and the repository's
real message definitions. A coordinate CTest checks PROJ reference values, signed
southern northings, ECEF/ENU axes, poles, dateline crossings and invalid inputs.
These tests need no network. The dedicated replay integration test starts the
player's file preparation after the server, changes telemetry field order,
adds a message absent from the live schema, checks another log with the same
aircraft ID, rejects invalid replacement files, verifies log rotation, and
stops with requests in flight:

```sh
python3 sw/ground_segment/tmtc/server_cpp/tests/replay_restart.py \
  sw/ground_segment/tmtc/server_cpp/build/server_cpp "$PWD"
```

It requires `python3-ivy` and local UDP/TCP networking. Add `--server-first` for
the other startup order. To include both in CTest, configure with
`-DSERVER_CPP_REPLAY_TEST=ON`.

The original `tests/ivy_smoke.py` is retained unchanged. Its subscription checks
expect the previous per-message bindings and reject the generic catch-all, so
it does not match the new reception layout. `SERVER_CPP_IVY_TEST` still selects
those original tests separately.
