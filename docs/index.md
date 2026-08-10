# Autumn documentation

Autumn is a VEX V5 PROS autonomous library template.

## Getting started

1. Edit `templates/config.hpp` for ports and drivetrain geometry (plain inches/degrees).
2. Implement `TemplateAuto::getRoutes()` in `templates/autonomous.hpp`.
3. Run `make libautumn` then `make`.

## Team API

- **Units**: inches, degrees, and milliseconds at the team boundary (`chassis.moveToPoint(24, 0, 4000)`).
- **Hardware**: register all devices in `templates/config.hpp`; access via `config()` in auton/driver.
- **Auton menu**: override `getRoutes()` — no manual `addRoutine` in `main.cpp`.

## Architecture

- **Core**: `Autumn`, `AutumnContext`, `AbstractConfig`, `AbstractAutonomous`, `AbstractDriver`
- **Motion**: `motion_handler`, free-function motions, `Chassis` facade
- **Localization**: `Odometry` (non-copyable — pass by reference), optional `ParticleFilter` (MCL)
- **Control**: PID, feedforward, trapezoid profiles, optional LTV unicycle + velocity PID; composable `Controller<In,Out>` + fusions wired into motions
- **Hardware HAL**: `Encoder` / `IMU` / `MotorGroup` / `AdiEncoder` facades with disconnect resilience + mutex
- **Logging**: `logger` (terminal/SD sinks) + brain `ErrorCode`s raised at HAL/motion sites
- **UI**: `AutonSelector` liblvgl menu owns screen (LLEMU fallback; do not mix)

## Motion API

```cpp
autumn::motion_handler::move([&] {
    autumn::moveToPoint(target, timeout, params, settings);
});
autumn::motion_handler::waitUntilDistance(12_in, settings.poseGetter);
```

Optional LTV path tracking:

```cpp
autumn::FollowParams params;
params.useLtvTracking = true;
chassis.follow(path, 15, 6000, false, params);
```

## Paths

Export paths from [Path.jerry.io](https://path.jerry.io) as `static/my_path.txt`:

```
x,y,speed
0,0,24
24,0,24
```

Declare with `ASSET(my_path_txt);` and pass to `chassis.follow(my_path_txt, ...)`.

## PROS Conductor

**Maintainer release:**

```bash
make libautumn
make template-release   # IS_LIBRARY=0, links prebuilt libautumn.a (team-facing zip)
pros conduct upload autumn@0.1.0
```

Use `make template` (without `-release`) when developing the library in this monorepo (`IS_LIBRARY=1`).

**Teams (remote depot):** add `depot/autumn.json` to `conductor.pros`, then:

```bash
pros conduct apply autumn
pros conduct upgrade autumn
```

User files (`templates/`, `src/main.cpp`, `static/`) are never overwritten on upgrade. System files (`include/autumn/`, `firmware/libautumn.a`) are upgraded.

The remote depot manifest lives on the `depot` branch (not yet published) rather than in this branch's `depot/` folder.
