# xgc2_b2_link — G3 onboard read-only data plane

This Jazzy package owns the Unitree B2/R5 status sources and the onboard half
of the frozen G3/G4 wire. The production ground consumer is
`xgc_unitree_b2_ros1_adapter`; the Python `b2_ground_peer` remains a contract
and dry-run tool only.

There is no motion path in this package. The forwarders subscribe only to
status topics, the contract reserves but does not authorize `down/cmd`, and no
executable creates a command publisher or subscription.

## Frozen wire

- Contract: `contract/zenoh_v1.yaml`
- Prefix: `xgc2/{robot_id}/up/*`
- Encoding: portable `json_utf8` for every active channel
- Domain 0 baseline: `odom`, `joint_states`, `power_summary`,
  `driver_status`, `forwarder_hb`
- Domain 17 optional arm: `arm_slave_status`, `arm_forwarder_hb`
- Transport is always explicit: `tcp` or `zenoh`; there is no `auto` fallback.

Domain 0 and Domain 17 run as separate ROS 2 processes and separate lightweight
transport client sessions. They share one ground fabric and one G4 Adapter;
the absence or failure of the arm session must not take the B2 body offline.

## Formal entrypoints for G1/G2

| Mode | Process | Formal entry | Required settings |
| --- | --- | --- | --- |
| LAB source | `b2-sim-publisher` | `ros2 run xgc2_b2_link b2_sim_publisher -- --robot-id b2-01` | `ROS_DOMAIN_ID=0`; standard ROS 2 messages only |
| LAB/FIELD body | `b2-forwarder-d0` | `ros2 run xgc2_b2_link b2_forwarder_d0 -- --config <forwarder_d0.yaml>` | `robot_id`, explicit transport endpoint; `ROS_DOMAIN_ID=0` |
| FIELD optional arm | `b2-forwarder-d17` | `ros2 run xgc2_b2_link b2_forwarder_d17 -- --config <forwarder_d17.yaml>` | `ROS_DOMAIN_ID=17`, `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`, `arx5_arm_msg/RobotStatus` |
| Contract probe only | `b2-ground-peer` | `python3 -m xgc2_b2_link.ground_peer --config config/ground_peer.yaml` | Never install as the production G4 peer |

CLI flags override values loaded through `--config`. The checked-in overlays
document all keys. LAB TCP is G4 `0.0.0.0:7448` and G3 `core:7448`; Zenoh uses
G4 `tcp/0.0.0.0:7447` and G3 `tcp/core:7447`. FIELD changes only the connect
address to the ground station LAN address.

LAB runs the simulator and d0 forwarder on `xgc2-dev-lab-agent-b2`. FIELD runs
only the forwarder(s) on `thor-b2`; the simulator is forbidden in FIELD.

## Readiness and failure semantics

Process start, target placement, and data readiness are separate evidence:

1. G1/G2 prove the Automation run and target Agent process.
2. G3 proves the d0 heartbeat contains increasing transmit counters for all
   five baseline channels.
3. G4 proves all five samples decoded within their receive-monotonic windows,
   semantic projection is live, and recovered ROS topics/TF advance.

The windows are odom/joints 1 s, power 2 s, and driver/heartbeat 3 s. Missing
configuration, ROS dependencies, Domain-17 environment, or transport startup
is a non-zero process failure. A later disconnect does not pretend the process
crashed: G4 marks the affected stream stale; any baseline stream makes B2
offline, while the optional arm becomes stale independently. Fresh samples
restore readiness without rebuilding Core state or changing run identity.

## LAB source behavior

`b2_sim_publisher` publishes typed ROS 2 sensor/status messages only:

| ROS 2 input | Wire key | Maximum output rate |
| --- | --- | ---: |
| `/b2/odom` | `up/odom` | 15 Hz |
| `/b2/joint_states` | `up/joint_states` | 15 Hz |
| `/b2/battery` (`sensor_msgs/BatteryState`) | `up/power_summary` | 2 Hz |
| `/b2/driver_status` | `up/driver_status` | 1 Hz |
| forwarder timer | `up/forwarder_hb` | 1 Hz |

Stopping the forwarder therefore stops all transport output even if the
simulator remains alive. This is required for fault injection and proves that
the simulator does not bypass G3.

## Contract loopback without ROS

Terminal A:

```bash
PYTHONPATH=. python3 -m xgc2_b2_link.ground_peer \
  --robot-id b2-01 --transport tcp --tcp-role server --tcp-port 7448
```

Terminal B (d0), with optional Terminal C using `arm_forwarder_node`:

```bash
PYTHONPATH=. python3 -m xgc2_b2_link.forwarder_node \
  --robot-id b2-01 --transport tcp --tcp-role client --tcp-port 7448 \
  --dry-run-no-ros
```

Dry-run transport fixtures test the wire only; they are not evidence that ROS
topics, an Agent placement, Core projection, or physical sources are live.

## Tests

```bash
PYTHONPATH=. python3 -m pytest test/ -q
```

## Visualization recovery

G4 publishes the frozen Run namespace, for example `/b21/odom`,
`/b21/joint_states`, `/b21/path`, and `world -> b21/b2_description` TF.
The generic Session visualization runtime owns namespaced robot descriptions
and `robot_state_publisher`; it is not a second wire input. Product acceptance
uses the Session-owned Foxglove and Lichtblick path.
