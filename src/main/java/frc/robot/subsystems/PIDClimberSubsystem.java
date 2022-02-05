// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class PIDClimberSubsystem extends PIDSubsystem {
  /** Creates a new PIDClimberSubsystem. */
  private final CANSparkMax m_motor = new CANSparkMax(Constants.ClimberConstants.kClimberMotorPort,
      MotorType.kBrushless);
  private final DigitalInput m_zerosw = new DigitalInput(Constants.ClimberConstants.kClimberZeroPort);
  private final RelativeEncoder m_encoder;
  private static int loop = 0;

  public PIDClimberSubsystem() {
    super(new PIDController(
        Constants.PIDClimberConstants.kClimbP,
        Constants.PIDClimberConstants.kClimbI,
        Constants.PIDClimberConstants.kClimbD));
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  public boolean getZeroSw() {
    return !m_zerosw.get();
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }

  public void climb(double speed) {
    if ((speed < 0) && (getZeroSw() || getEncoder() <= 0)) {
      disable();
      m_motor.set(0.0);
    } else if ((speed > 0) && getEncoder() >= Constants.ClimberConstants.kClimberMaxHeight) {
      disable();
      m_motor.set(0.0);
    } else {
      m_motor.set(speed);
    }
  }

  public void stop() {
    disable();
    m_motor.set(0);
    m_motor.stopMotor();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_motor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getEncoder();
  }

  public void climb2setpoint(double position) {
    getController().setSetpoint(position);
    enable();
  }

  @Override
  public void periodic() {
    loop += 1;
    if (getZeroSw()) {
      m_encoder.setPosition(0);
      getController().setSetpoint(0);
      enable();
    }
    // This method will be called once per scheduler run
    if (loop % 10 == 0) {
      SmartDashboard.putNumber("Climber Position", getEncoder());
      SmartDashboard.putBoolean("Zero Switch", getZeroSw());
      loop = 0;
    }
  }
}
