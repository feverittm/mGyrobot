// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will turn the robot to the specified angle using a motion profile. */
public class DriveToDistance extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetDistanceInches The distance in Inches to run
   * @param drive The drive subsystem to use
   */
  public DriveToDistance(double targetDistanceInches, DriveSubsystem drive) {
    super(
        new ProfiledPIDController(
            DriveConstants.kDistP,
            DriveConstants.kDistI,
            DriveConstants.kDistD,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxDistInchesPerS,
                DriveConstants.kMaxDistInchesPerSSquared)),
        // Close loop on heading
        drive::getDistanceInches,
        // Set reference to target
        targetDistanceInches,
        // Pipe output to turn robot
        (output, setpoint) -> drive.arcadeDrive(-output, 0),
        // Require the drive
        drive);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
