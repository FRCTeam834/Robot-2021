package frc.robot.commands;

//see documentation in RunConveyor.java
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Robot;

public class RunConveyorBackward extends CommandBase {
    //private boolean prevBottomSensorStatus;
    private boolean isFinished = false;

    public RunConveyorBackward() {
        addRequirements(Robot.conveyor);

    }

    @Override
    public void initialize() {
        Robot.conveyor.start(-.75);
        //prevBottomSensorStatus = Robot.conveyor.getBottomSensor();
        isFinished = false;

    }

    //called when scheduler runs while this command is scheduled
    @Override
    public void execute() {
    /*
        if (Robot.conveyor.getBottomSensor() == true && prevBottomSensorStatus == false) {
            Robot.ballCount--;
            prevBottomSensorStatus = true;
        }
        if (Robot.conveyor.getBottomSensor() == false) {
            prevBottomSensorStatus = false;
        }
        if (Robot.ballCount == 0) {
            isFinished = true;
        }
    */
    }

    @Override
    public void end(boolean interrupted) {
        Robot.conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}