/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import frc.robot.subsystems.DriveTrain;

public class DriveCurvy extends CommandBase {
  /**
   * Creates a new DriveNormal.
   */

  DriveTrain d;
  Joystick l = new Joystick(0);
  Joystick r = new Joystick(1);

  public DriveCurvy() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    d = Robot.driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    d.setDrive(0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    d.setCurvyDrive(l.getX(), r.getY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    d.setDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
