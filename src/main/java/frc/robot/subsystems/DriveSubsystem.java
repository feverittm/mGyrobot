// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final WPI_TalonFX m_leftMotor1 = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftMotor2 = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);
  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

  // The motors on the right side of the drive.
  private final WPI_TalonFX m_rightMotor1 = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightMotor2 = new WPI_TalonFX(DriveConstants.kRightMotor2Port);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The gyro sensor
  private AHRS m_ahrs;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    try {
      m_ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }

    // reset motors to factory defaults
    m_rightMotor1.configFactoryDefault();
    m_rightMotor2.configFactoryDefault();
    m_leftMotor1.configFactoryDefault();
    m_leftMotor2.configFactoryDefault();

    // Configure the integrated encoder on the TalonFX motors
    m_leftMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);
    m_rightMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.kPIDLoopIdx,
        Constants.kTimeoutMs);

    // Config the peak and nominal outputs, 12V means full
    m_leftMotor1.configNominalOutputForward(0, Constants.kTimeoutMs);
    m_leftMotor2.configNominalOutputReverse(0, Constants.kTimeoutMs);
    m_rightMotor1.configPeakOutputForward(1, Constants.kTimeoutMs);
    m_rightMotor2.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    resetEncoders();
    zeroHeading();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);
  }

  public void resetRobot() {
    resetEncoders();
    zeroHeading();
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftMotor1.setSelectedSensorPosition(0);
    m_rightMotor1.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftMotor1.getSelectedSensorPosition() - m_rightMotor1.getSelectedSensorPosition()) / 2.0;
  }

  public double getDistanceInches() {
    return (getAverageEncoderDistance() * Constants.DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Gets the average output of both sides of the drivetrain. If we are moving
   * straight then this is just
   * the speed of one side.
   *
   * @return the average voltage output
   */
  public double getAverageMotorVoltage() {
    return (m_leftMotor1.getMotorOutputVoltage() + m_rightMotor1.getMotorOutputVoltage()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_ahrs.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_ahrs.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    SmartDashboard.putNumber("Left Encoder", m_leftMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Encoder", m_rightMotor1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Average Distance", getDistanceInches());

    SmartDashboard.putNumber("Gyro Heading", m_ahrs.getAngle());
  }
}