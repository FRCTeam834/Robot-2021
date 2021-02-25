/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

//lol all this entire class does is just stop the conveyor belt as a defautl command
public class StopConveyor extends CommandBase {
  /**
   * Creates a new StopConveyor.
   */
  public StopConveyor() {
    // Use addRequirements() here to declare subsystem dependencies
    addRequirements(RobotContainer.conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.conveyor.stop();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
