# Distance control alongside an existing route

The route process keeps sending its usual twist, pose, or room-target commands.
The onboard PID adjusts their **final horizontal velocity request** to keep the
drones around a nominal **0.75 m** center-to-center XY distance, subject to
configured minimum and maximum distance limits.
It does not generate routes, fixed formation offsets, or takeoff/landing commands.

- Within the band: the follower actively restores nominal spacing, even when its
  route requests a different gap. The leader follows its route subject to limits.
- Approaching a boundary: reduce motion that would worsen the gap.
- Too close: request motion away from the peer.
- Too far: request motion toward the peer.
- Missing/stale/invalid peer or local position: request zero XY velocity, retaining
  the existing route target. Fresh valid data permits corrections to resume.

The alphabetically first drone ID leads (`drone1`); the other follows (`drone2`).
Both drones must configure the same two IDs, nominal separation and limits.
Along the line pointing from leader to follower, the follower requests
`leader_radial_velocity + target_gain * (nominal_separation - measured_distance)`.
This replaces the follower's radial route request, so opposing route commands
cannot cancel the nominal correction. Tangential route motion is preserved when
the speed budget allows. Route targets remain stored, but a conflicting follower
target may not be reached while spacing control is active.

Follower total desired XY speed is bounded by the existing room speed limit,
capped at 0.15 m/s. The leader gets two-thirds of that budget (at most 0.10 m/s),
leaving the follower headroom to catch up. With the main-branch defaults, the
follower limit is 0.10 m/s and the leader limit is about 0.067 m/s. The existing
room-target gain (0.20 by default) sets how quickly correction grows with distance
error. The three-sigma XY covariance radii are
added to the minimum and subtracted from the maximum. If uncertainty consumes
the usable band or excludes the nominal separation, XY motion is inhibited.

These are velocity-control limits, **not guaranteed physical distance bounds**:
latency, inertia, disturbances, localization error and saturation can cause
overshoot. Choose thresholds with measured rotor/downwash, braking and error
margins. This first version is for unloaded drones with horizontal separation in
a clear area; corrections do not plan around walls, obstacles or suspended cables.

## Configuration

The shared geometry is saved in
[`src/params/payload_geometry.yaml`](../../params/payload_geometry.yaml):
nominal drone separation **0.75 m**, payload attachment drop **0.5 m**. With level
drones and taut equal ropes meeting at a centered attachment, each effective rope
length is `sqrt(0.5^2 + (0.75 / 2)^2) = 0.625 m` (1.25 m total), plus material for knots/fittings.
This assumes drone attachment spacing equals drone separation. `payload_drop_m`
remains a planning input; it does not command altitude or enable payload flight.

At startup, if `/pidrone/coordination` is absent, the PID automatically loads this
geometry and [`src/params/coordination.yaml`](../../params/coordination.yaml).
Coordination is available by default, with **0.60 m minimum** and **0.90 m maximum**.
These are provisional unloaded-trial settings, not validated flight bounds or
payload settings. Position uncertainty must leave the nominal target inside the
usable band. The combined three-sigma XY margin must be below **0.15 m** for
this band to remain usable; do not reduce covariance estimates to force acceptance.

Identity comes from the hostname in `ROS_MASTER_URI`, which the existing Docker
build sets to the drone's own ROS master:

| Master hostname | Own ID | Peer ID | Role |
| --- | --- | --- | --- |
| `drone1` or `drone1.local` | `drone1` | `drone2` | Leader |
| `drone2` or `drone2.local` | `drone2` | `drone1` | Follower |

The Docker container hostname is not used. Unknown master hostnames (including
`localhost` and IP addresses) are not guessed: configure a complete
`/pidrone/coordination` dictionary explicitly with `drone_id`, `peer_id`,
`nominal_separation_m`, `min_distance`, and `max_distance` for such installations.
An existing parameter dictionary overrides automatic loading. Require
`0 < min_distance < nominal_separation_m < max_distance`.

The dashboard publishes its preflight selection on `/pidrone/coordination/paired`
(`std_msgs/Bool`). Once connected drones report fresh DISARMED modes, one connection
selects solo and two select paired. A selection is retained while either connected
drone is armed/flying, so staggered arming or a late connection does not change the
other drone's choice. Connect both drones before arming for a paired flight; use
one dashboard session to make this selection.

Each PID accepts selections only while DISARMED and latches the choice upon
ARMED (or a direct transition to FLYING). It keeps that choice until disarmed:
connecting the second drone during a solo flight does not turn on the guard, and
disconnecting during a paired flight does not turn it off. Paired flights brake
in XY when peer telemetry expires. A selection older than two seconds, or no
selection at all, keeps the guard on rather than assuming solo. Without the
dashboard, explicitly force solo or supply a fresh preflight selection.

To force solo regardless of dashboard connections, set `enabled: false` in
`coordination.yaml` before a fresh node session, or set this parameter before
restarting only the PID:

```sh
rosparam set /pidrone/coordination/enabled false
```

Make configuration changes and restart only while disarmed. The PID reads settings
once at startup. Existing parameters persist when restarting only the PID; to
reload the YAML files, delete `/pidrone/coordination` before that restart. A fresh
ROS master loads the files again, so the ROS-only solo override is not persistent.
Missing peer data never switches to solo mode. Invalid configuration or an unknown
automatic identity inhibits XY motion while preserving altitude and watchdogs.
The laptop peer relay still needs to be started separately.

Dashboard Execute now waits for fresh readiness from every selected routine before
sending its start releases. This is independent of the distance guard and does not
start the peer relay. Launch failure, disconnect, Stop or Disarm cancels a pending
release. Later route stages and takeoff times are not guaranteed synchronized.

Rebuild with `catkin_make` on both drones to generate the new `PeerState` message.
Use their ROS Kinetic/Python 2.7 environment. Both drones must use the same surveyed
anchor IDs/coordinates and common map frame. Align heading at fusion startup while
disarmed, stationary and facing map +Y; ongoing yaw drift is still possible.
Synchronize laptop/drone clocks. The relay currently expects odometry's body frame
to be `pidrone_body`.

## Peer telemetry relay

Use the existing laptop environment (`src/web/requirements.txt`, websockets 15):

```sh
.venv/bin/python tools/peer_relay.py
```

Override `--drone1 ws://HOST:9090` and `--drone2 ws://HOST:9090` if needed. Start the
relay before flight. It waits for both drones to be flying with fresh localization
and matching anchor maps, then forwards telemetry every 0.1 s. Starting the relay
can enable spacing corrections on already-flying, configured drones.

The relay only publishes `/pidrone/coordination/peer` (`pidrone_pkg/PeerState`).
It converts body velocity into map axes, preserves the oldest source measurement
timestamp, and includes the recipient's localization generation. It sends no
flight modes, route targets, altitude commands, motor commands or heartbeats.

Each PID rejects wrong identities, wrong frames, obsolete reference generations,
and data older than 0.5 s, with a local monotonic receipt watchdog as well. The relay
pauses both outgoing streams when either input becomes invalid or stale; there
is no automatic network reconnect. Ctrl-C stops the relay. The onboard response
is XY braking when data expires, not a new disarm or target capture. The original
FC watchdogs can still disarm for their existing fault conditions.

The route process can read `/pidrone/coordination/status` (`std_msgs/String`):
`clear`, `tracking_spacing`, `limited`, `too_close`, `too_far`, or a reason such as `peer_state_stale`,
`local_position_unavailable`, `reference_mismatch`, or `uncertainty_too_large`.
`disabled` means coordination is explicitly disabled or unconfigured; `solo`
means the flight was selected as solo; `inactive` means paired control is not flying.
`tracking_spacing` means the follower is overriding its radial route request to
track the leader and nominal gap; it is normal during spacing recovery.
Pause route progression when its motion is being restricted, and use actual
position/reached-target feedback rather than assuming elapsed time means arrival.
Existing routines that enforce progress deadlines may take their existing timeout
or landing paths if spacing corrections prevent progress; those paths are unchanged.

Distance control filters all normal XY route/manual/hold requests in a paired
flight. Z, yaw and explicit disarm remain under existing controls. Sensor gaps
do not silently replace a stored room target or substitute UWB Z for ToF.

## Verification

```sh
PYTHONPATH=src/scripts .venv/bin/python -m unittest discover -s src/scripts/localization -p 'test_*.py'
git diff --check
```

Tests use mocked ROS transport and cover nominal-gap correction direction, speed limits,
route/hold filtering, automatic identity, preflight solo/paired selection, timestamp
preservation, stale-data behavior and disarm. Dashboard tests cover connection-based
selection and retaining it across staggered arming and disconnects.
They do not establish flight stability or collision freedom. Validate each drone's
hold/braking first, then unloaded paired hover and short translations.
