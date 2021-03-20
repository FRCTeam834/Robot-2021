/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class GimbalLock extends SubsystemBase {
  /**
   * Creates a new ShooterPivot.
   */

  // Create new motor and limit switch objects
  WPI_TalonSRX pivot = new WPI_TalonSRX(ShooterConstants.SHOOTER_PIVOT_MOTOR_PORT);
  DigitalInput limitSwitch = new DigitalInput(ShooterConstants.HOOD_LIMIT_SWITCH_PORT);

  // The desired angle of the pivot
  double desiredAngle = 0;

  // Main constructor
  public GimbalLock() {

    // Setup the basic config of the motor
    pivot.setInverted(ShooterConstants.SHOOTER_PIVOT_INVERTED);
    pivot.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    // PID loop settings
    pivot.configClosedloopRamp(0.5);
    pivot.config_kP(0, 0.125);
    pivot.config_kI(0, 0);
    pivot.config_kD(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setDefaultCommand(new StopPivot);
  }

  // Set the motor at the desired speed
  public void setSpeed(double speed) {
    pivot.set(speed);
  }

  // Tilts the shooter up by the desired angle
  public void tiltUp(double angleInterval) {

    // Increment the desired angle, then move there
    setDesiredAngle(desiredAngle + angleInterval);
  }

  // Tilts the shooter down by the desired angle
  public void tiltDown(double angleInterval) {

    // Increment the desired angle, then move there
    setDesiredAngle(desiredAngle - angleInterval);
  }

  // Moves the pivot to the desired angle
  public void setDesiredAngle(double desiredAngle) {

    // Save the new value
    this.desiredAngle = desiredAngle;

    // Set the motor to move to the new position
    pivot.set(ControlMode.Position, ((desiredAngle / 360) * 4096) / ShooterConstants.HOOD_GEAR_RATIO);
  }

  // Halts the pivot
  public void stop() {
    pivot.set(0);
  }

  // Returns the angle of the motor
  public double getCurrentMotorAngle() {

    // Returns the rotations of the sensor (from 0 - 1) * 360 deg per rotation
    return ((pivot.getSelectedSensorPosition() / 4096) * 360);
  }

  // Returns the angle of the hood
  public double getCurrentHoodAngle() {
    return getCurrentMotorAngle() * ShooterConstants.HOOD_GEAR_RATIO;
  }

  // Returns if the limit switch is pressed
  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  // Sets the encoder's reference back to zero
  public void resetEncoder() {
    pivot.setSelectedSensorPosition(0);
  }
}
