/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.intake.StopIntake;


public class BallIntake extends SubsystemBase {
  /**
   * Creates a new BallIntake.
   */

  WPI_VictorSPX intakeMotor = new WPI_VictorSPX(Constants.INTAKE_MOTOR_PORT);

  Joystick j = new Joystick(0);
  Joystick j2 = new Joystick(1);

  public BallIntake() {
    intakeMotor.setInverted(Constants.INTAKE_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new StopIntake());
  }

  public void start(double speed) {

    intakeMotor.set(speed);

  }

  public void startBackwards() {

    intakeMotor.set(-.5);

  }

  public void stop() {

    intakeMotor.set(0);

  }

}
