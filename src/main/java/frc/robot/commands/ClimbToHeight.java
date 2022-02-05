// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.PIDClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/** A command that will move the climber to the specified height using a motion profile. */
public class ClimbToHeight extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetHeightInches The height in Inches to move
   * @param climber The climber subsystem to use
   */
  public ClimbToHeight(double targetHeightInches, ClimberSubsystem climb) {
    super(
        new ProfiledPIDController(
            PIDClimberConstants.kClimbP,
            PIDClimberConstants.kClimbI,
            PIDClimberConstants.kClimbD,
            new TrapezoidProfile.Constraints(
                PIDClimberConstants.kMaxClimbRateDegPerS,
                PIDClimberConstants.kMaxClimbAccelerationDegPerSSquared)),
        // Close loop on heading
        climb::getEncoder,
        // Set reference to target
        targetHeightInches,
        // Pipe output to turn robot
        (output, setpoint) -> climb.climb(output),
        // Require the drive
        climb);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(PIDClimberConstants.kClimbToleranceDeg, PIDClimberConstants.kClimbRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
