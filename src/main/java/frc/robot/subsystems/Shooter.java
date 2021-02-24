/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.StopShooter;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

  Joystick j = new Joystick(0);
  Joystick j2 = new Joystick(1);

  CANSparkMax shooter = new CANSparkMax(Constants.SHOOTER_MOTOR_PORT, CANSparkMax.MotorType.kBrushless);

  public Shooter() {
    shooter.setInverted(Constants.SHOOTER_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new StopShooter());
  }

  public void setSpeed(double speed) {

    shooter.set(speed);

  }

  public CANEncoder getEncoder() {

    return shooter.getEncoder();

  }

  public double getMotorTemp() {

    return shooter.getMotorTemperature();

  }

  public byte[] getSocialSecurityNumber() {

    return shooter.getSerialNumber();

  }

  public CANSparkMax getMotor() {

    return shooter;

  }

  public double normalize(double dogdareyou) {

    return dogdareyou / 4600;

  }

  public void stop() {

    shooter.set(0);

  }

  public void setIntakeToJoystick() {

    double bottomSpeed;
    bottomSpeed = j2.getRawAxis(2);
    System.out.println("BottomSpeedJoystick: " + bottomSpeed);
    bottomSpeed = bottomSpeed + 1;
    bottomSpeed = bottomSpeed / 2;
    shooter.set(bottomSpeed);
    System.out.println("BottomSpeed: " + shooter.getEncoder().getVelocity());
    /*
    double topSpeed;
    topSpeed = j.getRawAxis(2);
    topSpeed = topSpeed + 1;
    topSpeed = topSpeed / 2;
    //topShooter.set(-topSpeed);
    topShooter.set(bottomSpeed);
    System.out.println("TopSpeed: " + topShooter.getEncoder().getVelocity());
    */
    // System.out.println("TopSpeed: " + topSpeed + " Output: " /* + intakeMotor.getMotorOutputPercent() */);
    // System.out.println("Top Output Voltage: " /* +
    // intakeMotor.getMotorOutputVoltage() */);
    // System.out.println("\n");
    // System.out.println();

  }
}
