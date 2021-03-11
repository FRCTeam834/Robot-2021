/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.snapto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.lang.Math;

public class SnapTo180 extends CommandBase {
  /**
   * Creates a new Brotation.
   */
  boolean finished;
  double lMotor, rMotor;
  public SnapTo180() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain, RobotContainer.navX);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    lMotor = 1; 
    rMotor =1;

    lMotor = 1;
    rMotor = -1;
    Robot.driveTrain.setDrive(.5, -.5);
   /* //start turning robot in correct direction
    if (RobotContainer.navX.getYaw() < 0) { //if robot to right of 180, turn left
      Robot.driveTrain.setDrive(-0.5, 0.5);
      lMotor = -1;
      rMotor = 1;
    } else if (RobotContainer.navX.getYaw() >= 0) { //if robot to left of 180, turn right
      Robot.driveTrain.setDrive(0.5, -0.5);
      lMotor = 1;
      rMotor = -1;
    } 
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(RobotContainer.navX.getYaw()) >= 165) { // spin slower once close for percision 
      Robot.driveTrain.setDrive(lMotor*.25, rMotor*.25);
      if(Math.abs(RobotContainer.navX.getYaw()) >= 175) {
        //now the that we are facing 180, we can gg ez stop spinning
        finished = true;
      }
     } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
    Robot.driveTrain.setDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}