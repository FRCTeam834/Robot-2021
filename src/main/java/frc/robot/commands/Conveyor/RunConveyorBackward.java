package frc.robot.commands.Conveyor;

//see documentation in RunConveyor.java
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;

public class RunConveyorBackward extends CommandBase {
    //private boolean prevBottomSensorStatus;
    private boolean isFinished = false;

    public RunConveyorBackward() {
        addRequirements(RobotContainer.conveyor);

    }

    @Override
    public void initialize() {
        RobotContainer.conveyor.start(-.75);
        //prevBottomSensorStatus = RobotContainer.Conveyor.getBottomSensor();
        isFinished = false;

    }

    //called when scheduler runs while this command is scheduled
    @Override
    public void execute() {
    /*
        if (RobotContainer.Conveyor.getBottomSensor() == true && prevBottomSensorStatus == false) {
            RobotContainer.ballCount--;
            prevBottomSensorStatus = true;
        }
        if (RobotContainer.Conveyor.getBottomSensor() == false) {
            prevBottomSensorStatus = false;
        }
        if (RobotContainer.ballCount == 0) {
            isFinished = true;
        }
    */
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.conveyor.stop();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}