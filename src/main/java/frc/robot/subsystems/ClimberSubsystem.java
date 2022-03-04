// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.ClimberConstants.kClimberMotorPort,
      MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final DigitalInput m_zero;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_encoder = m_motor.getEncoder();
    m_zero = new DigitalInput(Constants.ClimberConstants.kClimberZeroPort);
    m_motor.setIdleMode(IdleMode.kBrake);
  }

  public void move(double speed){
    if (speed >= 0.0 && getEncoder() < Constants.ClimberConstants.kTopPosition)
      m_motor.set(speed);
    else if (speed < 0 && getEncoder() > 0 && !getZeroSwitch())
      m_motor.set(speed);
    else
      m_motor.set(0);
  }

  public void stop() {
    m_motor.stopMotor();
    m_motor.setIdleMode(IdleMode.kBrake);
  }


  public boolean getZeroSwitch() {
    return(!m_zero.get());
  }

  public double getEncoder() {
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber Zero Switch", getZeroSwitch());
  }
}
