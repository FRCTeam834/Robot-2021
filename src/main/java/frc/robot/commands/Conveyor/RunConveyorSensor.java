/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

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
    addRequirements(RobotContainer.conveyor, RobotContainer.ballIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isBall = false;
    falseCounter = 0;
    trueCounter = 0;

    prevBottomSensorStatus = RobotContainer.conveyor.getBottomSensor();
    prevTopSensorStatus = RobotContainer.conveyor.getTopSensor();
    RobotContainer.ballIntake.start(.5);

    RobotContainer.conveyor.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isBall = RobotContainer.conveyor.getBottomSensor();

    //check if ball blocking sensor, if it's been long enough, start the motor
    // if not, add to counter
    if (isBall == false) {
      RobotContainer.conveyor.start(.75);
      RobotContainer.ballIntake.stop();
      falseCounter = 0;
      trueCounter = 0;
    } else if (isBall == false) {
      falseCounter++;
    }
    //check if sensor is clear, if it's been long enough, stop the motor
    // if not, add to counter
    else if (isBall == true && trueCounter == 3) {
      RobotContainer.conveyor.stop();
      RobotContainer.ballIntake.start(.5);
      trueCounter = 0;
      falseCounter = 0;
    } else if (isBall == true) {
      trueCounter++;
    }
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
    isBall = false;
    RobotContainer.ballIntake.stop();
    RobotContainer.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
