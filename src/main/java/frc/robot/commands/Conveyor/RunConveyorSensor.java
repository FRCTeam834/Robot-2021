/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;

public class RunConveyorSensor extends CommandBase {
  /**
   * Creates a new RunConveyorSensor.
   */
  boolean sensorClear;
  int trueCounter, falseCounter;

  //ok so this variable will say if the sensor was previously covered or uncovered so we can tell when a new ball is passing in front of it.
  boolean prevBottomSensorStatus;
  boolean prevTopSensorStatus; //same deal as bottom sensor
  Timer timer = new Timer();

  public RunConveyorSensor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyor, Robot.ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensorClear = false;
    //falseCounter = 0;
    //trueCounter = 0;

    prevBottomSensorStatus = Robot.conveyor.getBottomSensor();
    prevTopSensorStatus = Robot.conveyor.getTopSensor();
    Robot.ballIntake.start(.5);
    

    Robot.conveyor.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensorClear = Robot.conveyor.getBottomSensor();

    //check if ball blocking sensor, if it's been long enough, start the motor
    if (sensorClear == false) {
      Robot.ballIntake.stop();
      timer.start();
      Robot.conveyor.start(.75);
    }

    if (timer.hasElapsed(0.35)) {
      Robot.ballIntake.start(.5);
      Robot.conveyor.stop();
      timer.stop();
      timer.reset();
    }
      //trueCounter = 0;
      //falseCounter++;
    //} else if (isBall == false) {
    //}
    //check if sensor is clear, if it's been long enough, stop the motor
/*
    if (RobotContainer.Conveyor.getBottomSensor() == true && prevBottomSensorStatus == false) {
      prevBottomSensorStatus = true;
      RobotContainer.ballCount++;
    } else if (RobotContainer.Conveyor.getTopSensor() == true && prevTopSensorStatus == false) {
      prevTopSensorStatus = true;
      RobotContainer.ballCount--;
    }

    if (RobotContainer.Conveyor.getBottomSensor() == false && prevBottomSensorStatus == true) {
      prevBottomSensorStatus = false;
    } else if (RobotContainer.Conveyor.getTopSensor() == false && prevTopSensorStatus == true) {
      prevTopSensorStatus = false;
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trueCounter = 0;
    falseCounter = 0;
    sensorClear = false;
    Robot.ballIntake.stop();
    Robot.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
