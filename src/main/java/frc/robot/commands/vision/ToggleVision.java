/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ToggleVision extends CommandBase {
  /**
   * Networktables (boolean) EVS/run_vision_processing -> true / false
   */
  boolean togVisFlag;

  public ToggleVision() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.EVSNetworkTables);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.EVSNetworkTables.getVisionTable().getEntry("run_vision_processing").getBoolean(false) == true) {
      togVisFlag = false;
    }
    if (RobotContainer.EVSNetworkTables.getVisionTable().getEntry("run_vision_processing").getBoolean(false) == false) {
      togVisFlag = true;
    }

    RobotContainer.EVSNetworkTables.getVisionTable().getEntry("run_vision_processing").setBoolean(togVisFlag);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
