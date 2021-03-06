// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.ClimberConstants.kClimberMotorPort,
      MotorType.kBrushless);
  private final DigitalInput m_zerosw = new DigitalInput(Constants.ClimberConstants.kClimberZeroPort);
  private final RelativeEncoder m_encoder;
  private static int loop = 0;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setInverted(true);
    m_encoder = m_motor.getEncoder();
    m_encoder.setPosition(0);
  }

  public boolean getZeroSw() {
    return !m_zerosw.get();
  }

  public void resetEncoder() {
    m_encoder.setPosition(0);
  }

  public double getExtension() {
    return m_encoder.getPosition();
  }

  public void climb(double speed) {
    if ((speed < 0) && (getZeroSw() || getExtension() <= 0)) {
      m_motor.set(0.0);
    } else if ((speed > 0) && getExtension() >= Constants.ClimberConstants.kClimberMaxHeight) {
      m_motor.set(0.0);
    } else {
      m_motor.set(speed);
    }
  }

  public void stop() {
    m_motor.set(0);
    m_motor.stopMotor();
  }

  @Override
  public void periodic() {
    loop += 1;
    if (getZeroSw()) {
      m_encoder.setPosition(0);
    }
    // This method will be called once per scheduler run
    if (loop % 10 == 0) {
      SmartDashboard.putNumber("Climber Position", getExtension());
      SmartDashboard.putNumber("Climber Velocity", m_encoder.getVelocity());
      SmartDashboard.putBoolean("Zero Switch", getZeroSw());
      loop = 0;
    }
  }
}
