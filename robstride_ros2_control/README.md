# robstride_ros2_control

Each configured joint exposes a `std_srvs/srv/Trigger` service at
`/robstride/set_zero/<joint_name>`. This low-level operation requires all command
controllers to be stopped and the managed gate synchronized; support the arm
externally before stopping controllers. It verifies zero feedback, requests a Type 22 save, and leaves
the command gate closed. Applications must pause command forwarding and automatic
recovery, then explicitly resynchronize controller targets before resuming.
The src2026 workspace provides `/arm_calibration/<joint_name>/set_zero` and
`/arm_calibration/resume` to coordinate these steps.

`robstride_ros2_control` exposes RobStride actuators as a ROS 2
`hardware_interface::SystemInterface`. One component can manage multiple motor
IDs on one CAN bus, and each joint can independently use position, velocity, or
effort commands.

Production robot descriptions can include
`$(find robstride_ros2_control)/description/robstride_motor_profiles.xacro`
without depending on `robstride_examples`. The helpers select a `model` whose
protocol limits are defined once in `robstride_driver`. Direct joint parameters
also accept `model` values `RS00` through `RS06`, `EL05` (alias `EduLite05`), or
`custom` for explicit numeric limits. Numeric overrides of a known model must
match its profile; use `command_*` parameters for operational limits.

Use the following plugin identifier in a `ros2_control` robot description:

```xml
<hardware>
  <plugin>robstride_ros2/RobStrideSystem</plugin>
</hardware>
```

Every joint exports position, velocity, and effort command interfaces and the
corresponding state interfaces. `online`, `ready`, and `fault` are also exported
for each joint, alongside optional temperature. Feedback timeout can be
configured with `feedback_timeout_ms`; `fail_on_feedback_timeout` publishes
offline state and gates motion writes while leaving feedback receive alive.
Recovery starts only on an explicit recovery-manager request. The driver keeps
retrying the requested motor initialization and Enable until recovery succeeds.

CAN traffic is exchanged through configurable `can_msgs/msg/Frame` topics, so
the component is not tied to a particular SocketCAN bridge. See the
[repository README](https://github.com/s2015-turtle/robstride_ros2#readme) for
the complete hardware and joint parameter reference, controller examples, and
operational limits.

## Independent motor availability

With `allow_partial_start: true`, the component stays active even when motors
are absent. CAN feedback, initialization and command gating run per motor;
a missing motor does not disable motion on healthy peers. Keep
`set_zero_on_activate: false` so reconnecting never changes the encoder origin.
The recovery manager requests startup or recovery by publishing motor indices
(URDF joint order) in a
`std_msgs/msg/UInt8MultiArray` on `/robstride/recover`. These requests do not
change the lifecycle of the whole hardware component.

Each requested motor independently confirms its watchdog and Run-mode parameters,
enables, and waits for fresh Run feedback. Its position command interface is
reset to measured position. While unavailable, its queued motion is discarded.
After recovery, the driver synchronizes the position command interface with the
latest measured position. In the robot's `managed_command_gate: true` mode, a
motor loss or a newly available motor latches the shared position-command gate
and increments its generation. The gate stays closed even if CAN feedback
returns before controller deactivation finishes. `/robstride/command_gate_state`
reports `[generation, closed, synchronized_while_unclaimed, activated_after_sync]`.
The recovery manager deactivates the standard JointTrajectoryController, waits
for position synchronization with interfaces unclaimed and fresh joint states,
then reactivates the controller. It publishes current position as the initial
command and requests gate release with the same generation on
`/robstride/release_command_gate` (`std_msgs/msg/UInt64`). Hardware rejects stale
generations, release before the stop/sync/start sequence, and initial targets
farther than 0.01 rad from current position. The first accepted target is replaced
with exact measured position.

During the gate interval only zero-gain, zero-effort keepalive frames are sent to
healthy motors so feedback and the motor watchdog remain alive; no position
control targets are sent. Missing motors receive no motion frames. Ordinary motor
power loss, feedback timeout, bridge disappearance or TX backpressure does not
change the hardware lifecycle. An explicit hardware `on_error()` is reported on
`/robstride/hardware_error`; only that event permits the manager to call the
existing controller-manager hardware lifecycle services.

`/robstride/health` publishes `[online, ready, fault]` for every motor in URDF
order. A single standard JointTrajectoryController claims all four position
joints. Once a motor has been observed ready, its loss stops the entire shared
controller until that motor returns. Initially absent motors do not prevent a
single-axis test. The other hardware joints are also paused by the shared
controller's lifecycle.

The robot uses the standard `joint_trajectory_controller/JointTrajectoryController`
with `command_interfaces: [position]` and `state_interfaces: [position]`.
`set_last_command_interface_value_as_state_on_activation: false` and
`interpolate_from_desired_state: false` keep activation and interpolation based
on measured state. Initially absent joints can have NaN feedback, but their
hardware command gate remains closed; healthy joints still operate.

The recovery manager is the sole publisher to
`/arm_position_controller/joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`).
External producers, including manual_controller, send the same standard type to
`/arm_position_controller/position_targets`, with `joint_names` in controller
order, absolute `points[].positions`, and finite optional velocities/accelerations.
The header stamp must be the current ROS time at creation/start, not zero or a
future reservation. Point times must be nonnegative and strictly increasing;
expired trajectories are dropped. This convention lets the manager reject commands
made before or during recovery even if delivered late. Manual control sends one
position point with a default 0.1 s horizon; multi-point trajectories are also
accepted. The initial recovery trajectory holds current measured position.

Producers use the latched `/arm_position_controller/recovery_ready`
(`std_msgs/msg/Bool`) to reset targets and input accumulation. The manager forwards
only after hardware acknowledges gate release, and only accepts start stamps
newer than that acknowledgement. All trajectory target positions are finite;
unavailable motor entries are gated by the driver. Do not publish directly to
the controller topic or action if recovery command filtering is required.
