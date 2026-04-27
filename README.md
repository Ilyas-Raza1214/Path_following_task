# Path Follower Challenge

This repository contains a simple implementation of the `PathFollower` class for the provided  path-following application.

## Approach

The implementation uses a simple lookahead proportional controller.

At each step, the controller:

1. Finds the closest pose on the path.
2. Selects a target pose a few indices ahead.
3. Computes position and yaw error.
4. Converts those errors into translational and rotational velocity.
5. Limits the command so parcel corner speed does not exceed the maximum wheel speed.

Main equations:

```text
position_error = target_position - current_position
translation_velocity = Kp * position_error

yaw_error = wrap(target_yaw - current_yaw)
rotation_velocity = Kr * yaw_error
```

The lookahead target is selected as:

```text
target_index = min(closest_index + lookahead_steps, last_path_index)
```

## Speed Limiting

Because the parcel has a finite size, its corners can move faster than the center during rotation.

For each corner:

```text
corner_velocity = translational_velocity + rotational_velocity_at_corner
```

If any corner exceeds the configured maximum speed, the full command is scaled down.

## Build and Run



```bash
mkdir build
cd build
cmake ..
cmake --build .
```

From the build directory, run the executable:

```bash
./path_follower
```

On Windows, use:

```cmd
path_follower.exe
```


