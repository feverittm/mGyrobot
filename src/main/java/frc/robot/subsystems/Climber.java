// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  private DigitalInput m_ZeroLimit;  // we need to know when we are at the climber's bottom
  private CANSparkMax m_climberMotor;
  private RelativeEncoder m_climberEncoder;

  public Climber() {
    m_climberMotor = new CANSparkMax(Constants.ClimberConstants.kClimberMotorPort, MotorType.kBrushless);
    m_climberMotor.restoreFactoryDefaults();


    m_climberEncoder = m_climberMotor.getEncoder();
    m_climberEncoder.setPosition(0);

    m_climberMotor.setSmartCurrentLimit(70);
    m_climberMotor.setIdleMode(IdleMode.kBrake);

    m_ZeroLimit = new DigitalInput(Constants.ClimberConstants.kClimberZeroPort);
  }

  public void setBrake() {
    m_climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    m_climberMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setVoltage(double voltage){
    if ((m_ZeroLimit.get() && voltage < 0.0) || voltage >= 0.0) {
      m_climberMotor.set(voltage);
    }
  }

  public double getEncoder() {
    return m_climberEncoder.getPosition();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber/Encoder Value", getEncoder());
    SmartDashboard.putNumber("Climber/Encoder Velocity", m_climberEncoder.getVelocity());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
    if (m_ZeroLimit.get() && getEncoder() != 0 ) { 
      m_climberEncoder.setPosition(0); 
    }
  }
}
