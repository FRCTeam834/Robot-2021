/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controlpanel;
/*
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SetCPColor extends CommandBase {

  String setColor, currentColor, prevColor;
  int counter;
  boolean missionComplete;

  public SetCPColor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.controlPanelManip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setColor = "";
    currentColor = Robot.controlPanelManip.determineWheelColor();
    setColor = Robot.gameData;
    Robot.controlPanelManip.start();
    missionComplete = false;
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update current color
    currentColor = Robot.controlPanelManip.determineWheelColor();
    // System.out.println(counter);

    // check if color is valid
    if (setColor.length() == 0) {
      setColor = Robot.gameData;
    }

    // fun logic to make sure color is good
    if (setColor.equals(currentColor)) {
      counter++;
      // if it has been good color for enough time then stop spinning
      if (counter == 5) {
        missionComplete = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Robot.controlPanelManip.stop();
    missionComplete = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return missionComplete;
  }
}
*/