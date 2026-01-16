// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double maxLinearSpeed = 4.69;
  public static final double maxLinearAcceleration = 4.0;
  public static final double maxAngularAcceleration = 20.0;
  public static final double maxAngularSpeed = 8.0; // 4.69 / driveBaseRadius;

  public static final PathConstraints CONSTRAINTS =
      new PathConstraints(4.5, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

  public static final TrapezoidProfile.Constraints X_CONSTRAINTS =
      new TrapezoidProfile.Constraints(maxLinearSpeed, maxLinearAcceleration);
  public static final TrapezoidProfile.Constraints Y_CONSTRAINTS =
      new TrapezoidProfile.Constraints(maxLinearSpeed, maxLinearAcceleration);
  public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(maxAngularSpeed, maxLinearAcceleration);

  public static final Pose2d[][] STATION_POSES =
      new Pose2d[][] {
        {
          // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
          new Pose2d(7.153, 7.272, Rotation2d.fromDegrees(180.0)),
          new Pose2d(7.153, 6.169, Rotation2d.fromDegrees(180.0)),
          new Pose2d(7.127, 1.905, Rotation2d.fromDegrees(180.0))
        },
        {
          // new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.447, 0.805, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.447, 1.991, Rotation2d.fromDegrees(0.0)),
          new Pose2d(10.447, 3.003, Rotation2d.fromDegrees(0.0))
        }
      };

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
