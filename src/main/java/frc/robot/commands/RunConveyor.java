/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class RunConveyor extends CommandBase {
  /**
   * Creates a new RunConveyor.
   */
  //ok so this variable will say if the sensor was previously covered or uncovered so we can tell when a new ball is passing in front of it.
  //boolean prevBottomSensorStatus;
 // boolean prevTopSensorStatus; //same deal as bottom sensor

  public RunConveyor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.conveyor.start(.75);
    //prevBottomSensorStatus = Robot.conveyor.getBottomSensor();
    //prevTopSensorStatus = Robot.conveyor.getTopSensor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //check if a ball is now entering while previous there was no ball
    //also checks if a ball is exiting the robot
    /*if (Robot.conveyor.getBottomSensor() == true && prevBottomSensorStatus == false) {
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

  // Called once the ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
