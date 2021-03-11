/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionAutonConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.createdclasses.Goal;

public class LookAtGoalX extends CommandBase {
  /**
   * Creates a new Goal.
   */
  public boolean isFinished;
  public Goal goal = new Goal();

  public LookAtGoalX() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.EVSNetworkTables);
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isFinished = false;

    try {

      if (RobotContainer.EVSNetworkTables.getGoalArray().get(0).size() != 0) {

        goal = new Goal(RobotContainer.EVSNetworkTables.getGoalArray().get(0));

      } else {

        System.out.println("No Goal Found");
        isFinished = true;

      }

    } catch (Exception e) {

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    goal.update(RobotContainer.EVSNetworkTables.getGoalArray().get(0));

    double deviation = goal.getCenterX() - 320;

    if (-1 * VisionAutonConstants.TOLERANCE < deviation && deviation < VisionAutonConstants.TOLERANCE) {

    } else if (deviation < -1 * VisionAutonConstants.TOLERANCE) {

      Robot.driveTrain.setDrive(-deviation * VisionAutonConstants.SPEED_INDEX, deviation * VisionAutonConstants.SPEED_INDEX);

    } else if (deviation > VisionAutonConstants.TOLERANCE) {

      Robot.driveTrain.setDrive(deviation * VisionAutonConstants.SPEED_INDEX, -deviation * VisionAutonConstants.SPEED_INDEX);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
