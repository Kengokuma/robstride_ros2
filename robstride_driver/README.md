# robstride_driver

The motor profile registry in `src/motor_profile.cpp` owns the RS00–RS06 and
EL05 protocol ranges. `motor_profile()` exposes these to configuration clients;
unknown names are rejected. These ranges retain the values checked against the
RobStride English manuals dated July 13, 2026, linked in the source and repository
README. They describe CAN normalization, not recommended operating limits.

`robstride_driver` is the bridge-independent core library for RobStride
actuators using the documented 29-bit private CAN protocol. It provides frame
encoding and decoding, `can_msgs/msg/Frame` topic transport, motor lifecycle
commands, feedback monitoring, and Run-mode recovery.

The package does not open a SocketCAN interface and does not provide a
standalone ROS node. Applications configure the transmit and receive topics
through `robstride_ros2_control`, then connect any compatible CAN bridge. The
example package uses `ros2_socketcan`.

Public headers are installed under `robstride_driver/` and cover:

- protocol constants and command/feedback encoding;
- per-model and per-joint limits;
- CAN topic transport;
- motor activation, stop, watchdog, and feedback handling.
- lock-free snapshots of traffic counters and per-motor feedback timing.

`RobStrideDriver::metrics()` returns the measured transmit categories,
latest-command-wins motion-command replacement count, recognized receive
counts, transport health, and per-motor mode, temperature, decoded faults,
recovery activity, feedback rate, and age. The snapshot reads a consistent
atomic status sample per motor and does not acquire the transport worker or
driver state mutex. When the normal ROS topic transport is used, the same
information is published once per second as standard diagnostics on
`/diagnostics`.

While active, the driver distinguishes temporary topic endpoint loss from a
persistent transmit failure. Persistent endpoint loss, a stalled sender, or an
unexpectedly stopped worker is propagated through the Hardware Component's
`write()` result. Motor feedback timeout remains an independent end-to-end
check because DDS publication alone cannot prove physical CAN delivery or motor
execution.

An invalid position command (including a nonfinite value or a position outside
the transformed CAN range) retains the last valid position target with holding
gains. It does not wrap to the opposite endpoint, latch a hardware error, or stop
healthy peers. The held position is refreshed on subsequent writes; valid
commands can resume without a recovery cycle. Position history is cleared when
position ownership changes, hardware starts, or a partial-operation motor loses
readiness. If no valid history exists, a representable measured position is used.
This behavior does not change communication-loss, motor-fault, or explicit-stop
handling, and does not guarantee holding torque without power or CAN delivery.

`set_zero(index, error)` sends Type 6 only while active managed partial hardware
has no claimed command interfaces and its command gate is closed and synchronized.
It rejects absent, stale, faulty, or moving target feedback. The driver blocks new
command claims during calibration, discards the old position history, and waits up
to two seconds for fresh stationary feedback near the configured position offset.
After confirming zero it sends one Type 22 save request (magic bytes 1 through 8)
to the same motor, then waits up to 500 ms for fresh stationary zero feedback.
Type 22 saves all storable parameters, not just the zero reference. Ordinary
feedback is not proof of flash persistence; verify at a fixed pose after a power
cycle. An unconfirmed zero is not saved. The gate stays closed on success or
failure; this call does not resume motion or save during ordinary activation.

Most users should integrate this library through the
[`robstride_ros2_control`](https://github.com/s2015-turtle/robstride_ros2/tree/main/robstride_ros2_control)
Hardware Component. See the
[repository README](https://github.com/s2015-turtle/robstride_ros2#readme) for
supported actuators, safety behavior, configuration parameters, and
installation instructions.

## Opt-in CSP position control

A joint with `control_mode=csp` uses `run_mode=5` and float `loc_ref` (0x7016),
not the Type 1 +/-4pi position field. Only the position command interface is
supported in this mode. `allow_partial_start=true` and
`managed_command_gate=true` are required, including at the direct driver API.
Other joints retain their existing motion-control behavior.

Required joint parameters are `csp_position_min/max` (motor-output rad),
`csp_speed_limit` (motor-output rad/s), `csp_current_limit` (A), and
`csp_torque_limit` (motor-output Nm). Bounds are explicit operational limits,
not claims about firmware travel. Match the current limit to the actual motor;
the parser accepts up to 43 A, the RS03 specification maximum. Bounds and all
limits must be finite and positive where applicable. Float endpoints are encoded
towards the interior. `command_position_min/max` remain in joint coordinates;
by default they are derived from the CSP bounds, direction, gearing and offset.
CSP does not use the motion-mode `kp`/`kd`: it retains the motor's existing
`loc_kp`, `spd_kp`, and `spd_ki`. It does not save or zero on activation.

The driver polls float `mechPos` (0x7019) at up to 50 Hz from `update_state()`.
CSP joint position comes exclusively from successful finite `mechPos` replies;
Type 2 still provides velocity, effort, mode and fault information. There is no
fallback to the wrapped Type 2 position. `csp_feedback_timeout_ms` defaults to
100 ms (40..1000 allowed). Missing, invalid or stale replies close the managed
gate and suspend new commands, even if Type 2 feedback continues. Suspension
alone does not send Stop: the motor retains its last target subject to its own
watchdog and protection. Explicit recovery uses Stop and zero current and must
be performed with the arm supported, as with the existing recovery path.

Startup/recovery confirms zero current limit, CSP mode, speed and torque limits
by parameter readback, obtains fresh `mechPos`, seeds the position target while
current is zero, and then enables. The gate remains closed with zero-current
keepalives until the controller stop/sync/start handshake completes. Active
commands send `loc_ref` followed by the configured current limit as one
coalesced/cancellable transport group. Invalid targets retain the last valid
position/current limit rather than releasing holding torque or stopping peers.
Shutdown sends zero current and Stop, never a Type 1 command for CSP joints.

An apparent `mechPos` fold larger than half the Type 2 position span while fresh
latches a CSP position fault. The old joint position is retained and recovery
cannot clear this fault; investigate the firmware/data and reactivate hardware
before proceeding. Intentional zero calibration is exempt from this check.
Power loss discards the old target through the existing gate; this does not
restore a power-off turn count or persist a new origin.
