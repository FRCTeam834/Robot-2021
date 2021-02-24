/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class RunConveyorSensor extends CommandBase {
  /**
   * Creates a new RunConveyorSensor.
   */
  boolean isBall;
  int trueCounter, falseCounter;

  //ok so this variable will say if the sensor was previously covered or uncovered so we can tell when a new ball is passing in front of it.
  boolean prevBottomSensorStatus;
  boolean prevTopSensorStatus; //same deal as bottom sensor

  public RunConveyorSensor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyor, Robot.ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBall = false;
    falseCounter = 0;
    trueCounter = 0;

    prevBottomSensorStatus = Robot.conveyor.getBottomSensor();
    prevTopSensorStatus = Robot.conveyor.getTopSensor();
    Robot.ballIntake.start(.5);

    Robot.conveyor.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isBall = Robot.conveyor.getBottomSensor();

    //check if ball blocking sensor, if it's been long enough, start the motor
    // if not, add to counter
    if (isBall == false) {
      Robot.conveyor.start(.75);
      Robot.ballIntake.stop();
      falseCounter = 0;
      trueCounter = 0;
    } else if (isBall == false) {
      falseCounter++;
    }
    //check if sensor is clear, if it's been long enough, stop the motor
    // if not, add to counter
    else if (isBall == true && trueCounter == 3) {
      Robot.conveyor.stop();
      Robot.ballIntake.start(.5);
      trueCounter = 0;
      falseCounter = 0;
    } else if (isBall == true) {
      trueCounter++;
    }
/*
    if (Robot.conveyor.getBottomSensor() == true && prevBottomSensorStatus == false) {
      prevBottomSensorStatus = true;
      Robot.ballCount++;
    } else if (Robot.conveyor.getTopSensor() == true && prevTopSensorStatus == false) {
      prevTopSensorStatus = true;
      Robot.ballCount--;
    }

    if (Robot.conveyor.getBottomSensor() == false && prevBottomSensorStatus == true) {
      prevBottomSensorStatus = false;
    } else if (Robot.conveyor.getTopSensor() == false && prevTopSensorStatus == true) {
      prevTopSensorStatus = false;
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trueCounter = 0;
    falseCounter = 0;
    isBall = false;
    Robot.ballIntake.stop();
    Robot.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
