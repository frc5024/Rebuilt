# FRC 5024 2026 Robot - Subsystems & Controls Documentation

## Overview
This document outlines all robot subsystems, their constants, and the button mapping for both driver and operator controllers.

---

## 🤖 Subsystems

### 1. **Swerve Drive Subsystem**
The primary drivetrain subsystem for holonomic movement and autonomous path following.

#### Key Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Max Linear Speed** | 4.69 m/s | Maximum driving velocity |
| **Max Linear Acceleration** | 4.0 m/s² | Maximum acceleration during teleop |
| **Max Angular Speed** | 8.0 rad/s | Maximum rotational velocity |
| **Max Angular Acceleration** | 20.0 rad/s² | Maximum rotational acceleration |
| **Drive Base Radius** | Calculated | Distance from center to wheels |
| **Odometry Frequency** | 250 Hz (FD) / 100 Hz (11) | Pose update rate |

#### Motor Configuration
- **Drive Motors**: Kraken X60 FOC (4x)
- **Steer Motors**: Falcon 500 (4x)
- **Steer Ratio**: 150:1 (MK4i L3)
- **Encoder**: CANcoder (absolute)
- **Gyro**: Pigeon 2

#### PathPlanner PID Constants
| Control | P | I | D |
|---------|---|---|---|
| **Position (X/Y)** | 10.0 | 0.1 | 0.0 |
| **Rotation (θ)** | 12.0 | 0.3 | 0.0 |

#### Motor PID Constants (SwerveModuleIOTalonFX.java)
**Drive Motor:**
- KP: 0.5, KI: 0, KD: 0
- KS: 0.14786, KV: 0.68343, KA: 0

**Steer Motor:**
- KP: 100, KI: 0, KD: 0.5
- KS: 0.1, KV: 2.66, KA: 0
- MotionMagic Cruise Velocity: 500.0 / 150 = 3.33 rot/sec
- MotionMagic Acceleration: 3.33 / 0.100 = 33.3 rot/sec²

---

### 2. **Hopper Subsystem**
Manages fuel storage and dispensing.

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Speed** | 0.85 | Motor output (0.0-1.0) |
| **Motor ID** | 8 | CAN ID |

---

### 3. **Turret Subsystem**
Rotational mechanism for aiming shooting trajectory.

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Angle Limit** | ±135° | Max rotation from center |
| **Speed** | 0.3 | Motor output (0.0-1.0) |
| **Motor Channel** | 3 | CAN/PWM ID |
| **Tolerance** | 0.5° | Angle settling tolerance |
| **Max Speed** | 70 °/sec | Max angular velocity |
| **Max Acceleration** | 30 °/sec² | Max angular acceleration |
| **Vertical Launch Angle** | 65° | Optimal shooting angle |

#### PID Constants
| Constant | Value |
|----------|-------|
| **KP** | 0.04 |
| **KI** | 0 |
| **KD** | 0.0 |
| **KS** | 0 |
| **KV** | 0.0 |
| **KA** | 0.0 |

---

### 4. **Shooter Subsystem**
Controls fuel velocity for competitive shooting.

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Set Velocity** | 3766 | Target RPM |
| **Shooter Speed** | 0.1 | Motor output |
| **Feeder Speed** | 0.85 | Feeder output |
| **Wheel Diameter** | 0.1016 m | 4 inches |

#### PID Constants
| Constant | Value |
|----------|-------|
| **KP** | 0.0006 |
| **KI** | 0 |
| **KD** | 0 |
| **KS** | 0.2 (Static friction voltage) |
| **KV** | 0.001764 (Velocity constant) |
| **KA** | 0.01 (Acceleration constant) |

#### Distance-Based Velocity Map
| Distance (m) | Target RPM |
|--------------|-----------|
| 2.4384 | 2800 |
| 3.048 | 3100 |
| 4.2672 | 3650 |
| 5.4864 | 4200 |
| 6.7056 | 4575 |
| 7.9248 | 5100 |

---

### 5. **Intake Subsystem**
Collects fuel cells from the field.

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Intake Speed** | 0.7 | Inward motor output |
| **Outtake Speed** | -0.4 | Outward motor output |
| **Extending Speed** | -0.15 | Arm extension output |
| **Retracting Speed** | 0.35 | Arm retraction output |
| **Drive Coefficient** | 0.1 | Intake-while-driving modifier |

#### Arm PID Constants
| Constant | Value |
|----------|-------|
| **KP (Arm)** | 0 |
| **KI (Arm)** | 0 |
| **KD (Arm)** | 0 |
| **KS (Arm)** | 0 |
| **KG (Arm)** | 0 |
| **KA (Arm)** | 0 |
| **KV (Arm)** | 0 |

#### Roll PID Constants
| Constant | Value |
|----------|-------|
| **KP (Roll)** | 0 |
| **KI (Roll)** | 0 |
| **KD (Roll)** | 0 |
| **KS (Roll)** | 0 |
| **KA (Roll)** | 0 |
| **KV (Roll)** | 0 |

---

### 6. **Climb Subsystem**
Mechanism for end-game positioning on climbing bars.

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Extend Speed** | 0.75 | Arm extension output |
| **Contract Speed** | -0.80 | Arm retraction output |
| **Declimb Speed** | 0.1 | Safe descent output |
| **Climb Speed** | -0.70 | Climbing output |
| **Max Position** | 780.0 | Maximum extension (radians) |
| **Min Position** | 10.0 | Minimum position (radians) |

---

### 7. **Feeder Subsystem**
Manages fuel delivery to shooter.

#### Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Motor ID** | 8 | CAN ID |

---

### 8. **Vision Subsystem**
AprilTag-based localization using Limelight cameras.

#### Cameras
| Camera | Name | Position | Orientation |
|--------|------|----------|-------------|
| **Front** | limelight-four | (-4.0", -11.75", +19.25") | 15° down |
| **Rear** | limelight-three | (-6.75", -11.75", +19.25") | 15° down, 180° rotated |

#### Vision Constants
| Constant | Value | Description |
|----------|-------|-------------|
| **Max Ambiguity** | 0.3 | Confidence threshold |
| **Max Z-Error** | 0.75 m | Height accuracy threshold |
| **Linear Std Dev Baseline** | 0.02 m | 1m/1tag position accuracy |
| **Angular Std Dev Baseline** | 0.06 rad | 1m/1tag rotation accuracy |
| **MegaTag 2 Linear Factor** | 0.5 | Multiplier (more stable) |
| **MegaTag 2 Angular Factor** | ∞ | No rotation data |

#### Camera Std Dev Factors
- Camera 0 (Front): 1.0
- Camera 1 (Rear): 1.0

---

### 9. **Robot Constants**

#### Physical Dimensions
| Constant | Value |
|----------|-------|
| **Frame Width** | 27 inches |
| **Full Width** | 34.5 inches |
| **Full Length** | 34.5 inches |
| **Mass** | 74.088 kg |
| **Moment of Inertia** | 6.883 |
| **Wheel Coefficient of Friction** | 1.2 |

---

## 🔌 Motor IDs & Addresses

### Swerve Drive Module IDs (CAN Bus)

**Front Left Module:**
| Component | CAN ID |
|-----------|--------|
| Drive Motor | 21 |
| Steer Motor | 22 |
| CANcoder | 2 |

**Front Right Module:**
| Component | CAN ID |
|-----------|--------|
| Drive Motor | 11 |
| Steer Motor | 12 |
| CANcoder | 1 |

**Back Left Module:**
| Component | CAN ID |
|-----------|--------|
| Drive Motor | 31 |
| Steer Motor | 32 |
| CANcoder | 3 |

**Back Right Module:**
| Component | CAN ID |
|-----------|--------|
| Drive Motor | 41 |
| Steer Motor | 42 |
| CANcoder | 4 |

### Other Subsystems (CAN/PWM IDs)

| Subsystem | Component | ID | Type | Controller |
|-----------|-----------|----|----|-----------|
| **Hopper** | Motor | 8 | CAN | SparkMax (REV) |
| **Turret** | Motor | 3 | CAN | SparkMax (REV) |
| **Shooter** | Lead Motor | 51 | CAN | SparkFlex (REV) |
| **Shooter** | Follower Motor | 52 | CAN | SparkFlex (REV) |
| **Intake** | Intake Motor | 60 | CAN | SparkMax (REV) |
| **Intake** | Arm Motor | 5 | CAN | SparkMax (REV) |
| **Climb** | Motor | 3 | CAN | TalonFX (CTRE) |

**Note:** Turret (ID 3, SparkMax) and Climb (ID 3, TalonFX) use different motor controller manufacturers, so they don't conflict on different networks.

---

## 🎮 Controller Mapping

### Driver Controller (Xbox Controller - Port 0)

#### Driving
| Input | Action | Effect |
|-------|--------|--------|
| **Left Stick Y** | Drive Forward/Back | Field-relative translation |
| **Left Stick X** | Strafe Left/Right | Field-relative translation |
| **Right Stick X** | Rotate | Robot-relative rotation |

#### Shooting & Intake
| Button | Action |
|--------|--------|
| **Left Trigger (LT)** | **Shoot** - Runs feeder, shooter, hopper in parallel. After 2s, retracts intake arm |
| **Right Trigger (RT)** | **Slow Mode** - Reduces drivetrain speed while held |

#### Aiming & Alignment
| Button | Action |
|--------|--------|
| **A Button** | **Snap to Hub** - Auto-aims robot toward the hub while driving normally |
| **X Button** | **Spin to Hub** - Rotates turret to aim at hub |

#### Intake & Arm
| Button | Action |
|--------|--------|
| **Y Button** | **Retract Intake Arm** - Retracts intake mechanism |
| **Right Bumper (RB)** | **Extend Intake Arm** (on press) + **Intake** (while held) - Extends arm and runs intake |

#### Climb
| Button | Action |
|--------|--------|
| **POV Up** | **Extend Climb Arm** - Extends climbing mechanism |
| **POV Down** | **Contract Climb Arm** - Retracts climbing mechanism |
| **POV Left** | **Turret Rotate Left** - Manual turret rotation (+0.1) |
| **POV Right** | **Turret Rotate Right** - Manual turret rotation (-0.1) |

---

### Operator Controller (Xbox Controller - Port 1)

#### Active Commands
| Button | Action |
|--------|--------|
| **Y Button** | **Pathfind Test** - Executes "Pathfind Test Path" command |

---

### Test Controller (Xbox Controller - Port 2)
No active bindings.

---

## 🎯 Game Piece Constants

| Constant | Value |
|----------|-------|
| **Fuel Cell Diameter** | 0.15 m |
| **Fuel Cell Mass** | 0.203 kg |

---

## 🔧 Configuration Files

- **Drive PID Values**: `src/main/java/frc/robot/subsystems/swervedrive/SwerveModuleIOTalonFX.java`
- **PathPlanner Config**: `src/main/java/frc/robot/Constants.java` (RobotConstants.PP_CONFIG)
- **Button Bindings**: `src/main/java/frc/robot/controllers/ButtonBindings.java`
- **All Subsystem Constants**: `src/main/java/frc/robot/Constants.java`

---

## 📝 Notes

### Swerve Drive Tuning
- **Motor PID Values**: Conservative gains optimized for real hardware (KP 0.5 drive, KP 100 steer)
- **MotionMagic Steering**: Configured with 500.0 cruise velocity for responsive turning
- **PathPlanner**: Uses position P: 10.0, rotation P: 12.0 for autonomous tracking

### Vision Integration
- Uses dual Limelight cameras (front & rear) for robust localization
- AprilTag-based positioning with standard deviation adjustments based on distance
- MegaTag 2 multi-tag detection for improved accuracy

### Autonomous
- Paths defined in `src/main/deploy/pathplanner/paths/`
- Auto routines available in dashboard chooser
- Supports dynamic pathfinding during match

---

## 🚀 Quick Reference

**Default Drive Mode**: Field-relative with normal speed
**Slow Mode**: Right trigger - reduces all velocities
**Auto Aim**: A button - snaps robot toward hub while driving
**Shoot**: Left trigger - full shooting sequence
**Climb**: POV up/down - extend/contract climb arm

---

*Document Last Updated: March 9, 2026*
*For questions or updates, refer to the source Constants.java file*
