/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ConveyorConstants;

import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;


public class EmptyShooter extends CommandBase {
  /**
   * Creates a new EmptyShooter.
   */

  //ok so this variable will say if the sensor was previously covered or uncovered so we can tell when a new ball is passing in front of it.
  boolean prevBottomSensorStatus;
  boolean prevTopSensorStatus; //same deal as bottom sensor

  private boolean isFinished = false;

  public EmptyShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.shooter, RobotContainer.conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevBottomSensorStatus = RobotContainer.conveyor.getBottomSensor();
    prevTopSensorStatus = RobotContainer.conveyor.getTopSensor();

    isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.shooter.getMotor().setVoltage(ShooterConstants.S_WHEEL_VOLTAGE);
    RobotContainer.conveyor.start(ConveyorConstants.AUTON_CONVEYOR_SPEED);
    /*if (RobotContainer.ballCount == 0) {
      isFinished = true; //may need to remove exclamation point
    }

    //check if a ball is now entering while previous there was no ball
    //also checks if a ball is exiting the robot
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
