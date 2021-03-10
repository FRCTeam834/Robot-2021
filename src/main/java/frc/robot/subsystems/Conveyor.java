/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ConveyorConstants;

public class Conveyor extends SubsystemBase {
  /**
   * Creates a new Conveyor.
   */
  WPI_VictorSPX conveyorMotor = new WPI_VictorSPX(ConveyorConstants.CONVEYOR_MOTOR_PORT);
  DigitalInput bottomSensor = new DigitalInput(ConveyorConstants.BALL_SENSOR_PORT);
  DigitalInput topSensor = new DigitalInput(ConveyorConstants.EMPTY_SENSOR_PORT);

  
  public Conveyor() {
    conveyorMotor.setInverted(ConveyorConstants.CONVEYOR_INVERTED);
  }

  @Override
  public void periodic() {
    // This method will be called once per schedule run
    //setDefaultCommand(new StopConveyor());
  }

  public boolean getBottomSensor() {
    return bottomSensor.get();
  }

  public boolean getTopSensor() {
    return topSensor.get();
  }

  public void start(double speed) {
    conveyorMotor.set(speed);
  }

  public void stop() {
    conveyorMotor.set(0);
  }
}
