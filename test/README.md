# Jazzy integration checks

Build the Driver and Description repositories together with a compatible Elite CS
SDK, then source `/opt/ros/jazzy/setup.bash` and the workspace's `install/setup.bash`.
From this repository, run:

```bash
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  python3 -m pytest -v test/test_jazzy_integration.py
```

These opt-in tests use `mock_components/GenericSystem`; they do not connect to a
robot. They exercise CS63A, CS630, and five-axis CS520H controller activation,
trajectory action execution, and MoveIt planning with time parameterization.
The dual-arm check runs the launch example from `elite_robots_driver/doc/MultiArm.md`
and verifies separate controller managers and prefixed joint interfaces.
Pytest's temporary directory retains launch logs for diagnosis.

The Description repository's `colcon test --packages-select elite_robots_description`
also checks all thirteen models with and without a TF prefix, including mesh paths.
