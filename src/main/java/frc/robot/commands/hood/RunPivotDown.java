/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;

public class RunPivotDown extends CommandBase {
  /**
   * Creates a new RunClimberUp.
   */
  public RunPivotDown() {
    addRequirements(RobotContainer.gimbalLock);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.gimbalLock.tiltDown(ShooterConstants.SHOOTER_PIVOT_SPEED);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.gimbalLock.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
