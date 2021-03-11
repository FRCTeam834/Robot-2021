/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ResetYaw extends CommandBase {
  /**
   * Creates a new ResetYaw.
   */
  boolean finished;
  double lMotor, rMotor; //multiply by speed to set direction of motor
  public ResetYaw() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.navX);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //check to see if the robot is facing closer to 0 or 180 degrees
    //if closer to 180, yaw is inverted
    finished = false;
    lMotor = 1; 
    rMotor =1;
    if(Math.abs(Math.abs(RobotContainer.navX.getYaw())-180) < Math.abs(RobotContainer.navX.getYaw())) {
      Robot.yawBackwards = true; 
    }

    //turn robot to face 0 or 180(if inverted)
    /*if(!RobotContainer.yawBackwards && RobotContainer.navX.getYaw() < 0){ //if robot to left of 0, turn right
      Robot.driveTrain.setDrive(0.5, -0.5);
      lMotor = 1;
      rMotor = -1;
    } else if (!RobotContainer.yawBackwards && RobotContainer.navX.getYaw() > 0) { //if robot to right of 0, turn left
      Robot.driveTrain.setDrive(-0.5, 0.5);
      lMotor = -1;
      rMotor = 1;
    } */
    //turn robot to face 180. doesn't need to be turned to 0 if it is not inverted because duh
    if (Robot.yawBackwards && RobotContainer.navX.getYaw() < 0) { //if robot to right of 180, turn left
      Robot.driveTrain.setDrive(0.5, -0.5);
      lMotor = -1;
      rMotor = 1;
    } else if (Robot.yawBackwards && RobotContainer.navX.getYaw() > 0) { //if robot to left of 180, turn right
      Robot.driveTrain.setDrive(-0.5, 0.5);
      lMotor = 1;
      rMotor = -1;
    } else if (!Robot.yawBackwards) {
      finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //turning to 180 (if inverted)
    if (Robot.yawBackwards) {
      if(Math.abs(RobotContainer.navX.getYaw()) >= 165) {
       Robot.driveTrain.setDrive(lMotor*.25, rMotor*.25);
       if(Math.abs(RobotContainer.navX.getYaw()) >= 175) {
         //now the that we are facing 180, we can reset yaw so it is no longer backwards!
         RobotContainer.navX.resetYaw(); 
         Robot.yawBackwards = false;
         finished = true;
       }
      } 
    } 
    /*//turning to 0 (not inverted)
    if (!RobotContainer.yawBackwards) {
      if(Math.abs(RobotContainer.navX.getYaw()) <= 10) {
       Robot.driveTrain.setDrive(lMotor*.25, rMotor*.25);
       if(Math.abs(RobotContainer.navX.getYaw()) <= 3) {
         finished = true;
       }
      } 
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
