/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class GimbalLock extends SubsystemBase {
  /**
   * Creates a new ShooterPivot.
   */

  WPI_TalonSRX pivot = new WPI_TalonSRX(Constants.SHOOTER_PIVOT_MOTOR_PORT);
  DigitalInput limitSwitch = new DigitalInput(Constants.HOOD_LIMIT_SWITCH_PORT);

  public GimbalLock() {
    pivot.setInverted(Constants.SHOOTER_PIVOT_INVERTED);
    pivot.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //setDefaultCommand(new StopPivot);
  }

  public void tiltUp(double n) {

    pivot.set(n);

  }

  public void tiltDown(double n) {

    pivot.set(-n);

  }

  public void stop() {

    pivot.set(0);

  }

  public double getEncoder() {

    return pivot.getSelectedSensorPosition() / 4096 * Math.PI * 2 * Constants.HOOD_GEAR_RATIO + 25 * 2 * Math.PI / 360;

  }

  public boolean getLimitSwitch() {
    return limitSwitch.get();
  }

  public void resetEncoder() {
    pivot.setSelectedSensorPosition(0);
  }
}
