/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterToSpeed extends CommandBase {
  /**
   * Creates a new ShooterToSpeed.
   */
  private boolean isFinished = false;

  public ShooterToSpeed() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isFinished = false;
    Robot.shooter.getMotor().setVoltage(Constants.S_WHEEL_VOLTAGE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Robot.shooter.getEncoder().getVelocity() >= (Constants.S_WHEEL_SPEED * 60)) {

      isFinished = true;

    }

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
