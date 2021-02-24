/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.createdclasses.PowerCell;

public class LookAtPowerCell extends CommandBase {
  /**
   * Creates a new LookAtPowerCell.
   */
  public boolean isFinished;
  public PowerCell power = new PowerCell();

  public LookAtPowerCell() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.EVSNetworkTables);
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isFinished = false;

    try {

      if (Robot.EVSNetworkTables.getPowerCellArray().get(0).size() != 0) {

        power = new PowerCell(Robot.EVSNetworkTables.getPowerCellArray().get(0));

      } else {

        System.out.println("No Power Cell Found");
        isFinished = true;

      }

    } catch (Exception e) {

    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    power.update(Robot.EVSNetworkTables.getPowerCellArray().get(0));

    double deviation = power.getCenterX() - 320;

    if (-1 * Constants.TOLERANCE < deviation && deviation < Constants.TOLERANCE) {

    } else if (deviation < -1 * Constants.TOLERANCE) {

      Robot.driveTrain.setDrive(-deviation * Constants.SPEED_INDEX, deviation * Constants.SPEED_INDEX);

    } else if (deviation > Constants.TOLERANCE) {

      Robot.driveTrain.setDrive(deviation * Constants.SPEED_INDEX, -deviation * Constants.SPEED_INDEX);

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
