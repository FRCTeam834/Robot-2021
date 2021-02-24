/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */
  WPI_VictorSPX climberMotor = new WPI_VictorSPX(Constants.CLIMBER_MOTOR_PORT);
  DigitalInput limitBottom = new DigitalInput(Constants.CLIMBER_LIMIT_SWITCH_PORT);

  public Climber() {
    climberMotor.setInverted(Constants.CLIMBER_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void up(double speed) {
    climberMotor.set(speed);
  }

  public void down(double speedWagon) {
    climberMotor.set(-speedWagon);
  }

  public void stop() {
    climberMotor.set(0);
  }

  //returns true if limit switch is being activated
  public boolean getLimitBottom()
  {
    return limitBottom.get();
  }

}
