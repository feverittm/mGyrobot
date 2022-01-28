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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {
  private CANSparkMax climberMotor;
  private RelativeEncoder climberEncoder;

  public Climber() {
    climberMotor = new CANSparkMax(Constants.ClimberConstants.ClimberMotorPort, MotorType.kBrushless);

    climberEncoder = climberMotor.getEncoder();

    climberMotor.setSmartCurrentLimit(70);
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setBrake() {
    climberMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoast() {
    climberMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setVoltage(double speed){
    climberMotor.set(speed);
  }

  public double getEncoder() {
    return climberEncoder.getPosition();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Climber/Encoder value", getEncoder());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();
  }
}
