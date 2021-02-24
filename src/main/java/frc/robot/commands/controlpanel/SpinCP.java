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

public class SpinCP extends CommandBase {

  String currentColor, firstColor, prevColor;
  int colorChangeCounter, spinCounter;
  boolean missionComplete;

  public SpinCP() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.controlPanelManip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // find color when code first called
    firstColor = Robot.controlPanelManip.determineWheelColor();
    currentColor = Robot.controlPanelManip.determineWheelColor(); 
    prevColor = null;
    colorChangeCounter = 0;
    spinCounter = 0;
    missionComplete = false;
    Robot.controlPanelManip.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if something went wrong during initialization, try to set a first color
    if (firstColor.equals("NO MATCH")) {
      firstColor = Robot.controlPanelManip.determineWheelColor();
    }
    // check to see if the color has changed to a new acceptable color
    if (!(Robot.controlPanelManip.determineWheelColor().equals(currentColor))
        && !Robot.controlPanelManip.determineWheelColor().equals("NO MATCH")) {
      // colorChangeCounter++;
      // basically, if the detected color has been different than previous
      // for long enough, switch that to current color and set previous color to old
      // color
      if (colorChangeCounter == 0) {
        if (currentColor)
        prevColor = currentColor;
        currentColor = Robot.controlPanelManip.determineWheelColor();
        colorChangeCounter = 0;
        // if the cp has rotated back to the starting color, add 1 to counter
        if (currentColor.equals(firstColor)) {
          spinCounter++;
        }
      }
    }
    System.out.println(Robot.controlPanelManip.determineWheelColor());
    System.out.println(spinCounter);

    // if the wheel has been spun 3.5 times, stop spinning
    // each spin adds 2 to the counter
    if (spinCounter == 7) {
      missionComplete = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.controlPanelManip.stop();
    missionComplete = false;
    System.out.println("IT HAS BEEN DONE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return missionComplete;
  }*/