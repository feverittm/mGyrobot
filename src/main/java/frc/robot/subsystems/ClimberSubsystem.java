// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.ClimberConstants.kClimberMotorPort,
      MotorType.kBrushless);
  private final RelativeEncoder m_encoder;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }

  public void climb(double speed) {
    m_motor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
