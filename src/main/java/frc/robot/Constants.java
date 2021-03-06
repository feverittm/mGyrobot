// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Which PID slot to pull gains from. Starting 2018, you can choose from
   * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
   * configuration.
   */
  public static final int kSlotIdx = 0;

  /**
   * Talon FX supports multiple (cascaded) PID loops. For
   * now we just want the primary one.
   */
  public static final int kPIDLoopIdx = 0;

  /**
   * Set to zero to skip waiting for confirmation, set to nonzero to wait and
   * report to DS if action fails.
   */
  public static final int kTimeoutMs = 30;

  /* Choose so that Talon does not report sensor out of phase */
  public static boolean kSensorPhase = true;

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 13;
    public static final int kLeftMotor2Port = 14;
    public static final int kRightMotor1Port = 15;
    public static final int kRightMotor2Port = 16;

    public static final double kDriveMaxSpeed = 0.25;
    public static final double kDriveTurboSpeed = 0.5;

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelGearboxRatio = 7.31;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterInches * Math.PI)
        / ((double) kEncoderCPR * kWheelGearboxRatio);

    public static final boolean kGyroReversed = false;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 0.001;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 50;
    public static final double kMaxTurnAccelerationDegPerSSquared = 90;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kDistP = 0.2;
    public static final double kDistI = 0;
    public static final double kDistD = 0;
    public static final double kMaxDistInchesPerS = 30;
    public static final double kMaxDistInchesPerSSquared = 50;
    public static final double kDistToleranceDeg = 2;
    public static final double kDistSpedToleranceDegPerS = 10; // degrees per second
  }

  public static final class ClimberConstants {
    public static final int kClimberMotorPort = 8;
    public static final int kClimberZeroPort = 0;
    public static final int kClimberMaxHeight = 120;
  }

  public static final class PIDClimberConstants {
    public static final double kClimbP = 0.1;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    public static final double kClimbTolerance = 5.0;
    public static final double kMaxClimbRateDegPerS = 20;
    public static final double kMaxClimbAccelerationDegPerSSquared = 30;

    public static final double kClimbToleranceDeg = 5;
    public static final double kClimbRateToleranceDegPerS = 10; // degrees per second
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
}
