// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDClimb extends ProfiledPIDCommand {
  /** Creates a new PIDClimb. */
  public PIDClimb(double targetExtension, ClimberSubsystem climb) {
    super(
        new ProfiledPIDController(
            ClimberConstants.kClimbP,
            ClimberConstants.kClimbI,
            ClimberConstants.kClimbD,
            new TrapezoidProfile.Constraints(
                ClimberConstants.kMaxClimbRateDegPerS,
                ClimberConstants.kMaxClimbAccelerationDegPerSSquared)),
        // Close loop on heading
        climb::getExtension,
        // Set reference to target
        targetExtension,
        // Pipe output to turn robot
        (output, setpoint) -> climb.climb(output),
        // Require the drive
        climb);

    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(ClimberConstants.kClimbToleranceDeg, ClimberConstants.kClimbRateToleranceDegPerS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
