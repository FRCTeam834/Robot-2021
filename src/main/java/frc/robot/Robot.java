/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.URL;
import java.net.URLConnection;

import org.opencv.core.Point;
*/
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.commands.*;
import frc.robot.commands.Conveyor.RunConveyor;
import frc.robot.commands.Conveyor.RunConveyorBackward;
import frc.robot.commands.Conveyor.RunConveyorSensor;
import frc.robot.commands.Conveyor.StopConveyor;
import frc.robot.commands.autonomous.EmptyShooterNoVision;
import frc.robot.commands.autonomous.ShooterToSpeed;
import frc.robot.commands.autonomous.autons.TestAuto;
import frc.robot.commands.autonomous.irahAutons.Beeline;
import frc.robot.commands.autonomous.irahAutons.DoABarrelRoll;
import frc.robot.commands.autonomous.irahAutons.InelasticCollision;
import frc.robot.commands.autonomous.irahAutons.PlanA;
import frc.robot.commands.autonomous.irahAutons.SlalomIBarelyKnowEm;
import frc.robot.commands.drive.DriveBackwardsDistance;
import frc.robot.commands.drive.DriveForwardDistance;
import frc.robot.commands.drive.DriveInverted;
import frc.robot.commands.drive.DriveMaxSpeed;
import frc.robot.commands.drive.DriveSlowSpeed;
import frc.robot.commands.hood.RunPivotDown;
import frc.robot.commands.hood.RunPivotUp;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeBackwards;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.commands.snapto.SnapTo0;
import frc.robot.commands.snapto.SnapTo180;
import frc.robot.commands.vision.ToggleVision;
/*
import frc.robot.commands.drive.DriveMaxSpeed;
import frc.robot.commands.drive.DriveNormal;
import frc.robot.commands.drive.DriveSlowSpeed;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.RunIntakeBackwards;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.autonomous.AimAndShoot;
import frc.robot.commands.vision.ToggleVision;
*/
import frc.robot.createdclasses.Goal;
import frc.robot.subsystems.DriveTrain;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public final static DriveSlowSpeed driveSlowSpeed = new DriveSlowSpeed();
  public  final static DriveMaxSpeed driveMaxSpeed = new DriveMaxSpeed();
  public final static DriveInverted driveInvertedd = new DriveInverted();

  public final static RunIntake runIntake = new RunIntake();
  public final static RunIntakeBackwards runIntakeBackwards = new RunIntakeBackwards();
  public final static StopIntake stopIntake = new StopIntake();

  public final static RunShooter runShooter = new RunShooter();
  public final StopShooter stopShooter = new StopShooter();

  public final RunPivotUp runPivotUp = new RunPivotUp();
  public final RunPivotDown runPivotDown = new RunPivotDown();

  public final RunConveyor runConveyor = new RunConveyor();
  public final static RunConveyorSensor runConveyorSensor = new RunConveyorSensor();
  public final RunConveyorBackward runConveyorBackward = new RunConveyorBackward();
  public final StopConveyor stopConveyor = new StopConveyor();

  public final static ToggleVision toggleVision = new ToggleVision();
  //private final AimAndShoot aimAndShoot = new AimAndShoot();
  public final ShooterToSpeed shooterToSpeed = new ShooterToSpeed();
  public final DriveBackwardsDistance driveBackwardsDistance = new DriveBackwardsDistance(12);
  public final EmptyShooterNoVision emptyShooterNoVision = new EmptyShooterNoVision();
  //private final ResetYaw resetYaw = new ResetYaw();
  public final SnapTo0 snapTo0 = new SnapTo0();
  public final SnapTo180 snapTo180 = new SnapTo180();
  public final DriveForwardDistance driveForwardDistance = new DriveForwardDistance(.25, 69);
  public final static TestAuto testAuto = new TestAuto(Robot.driveTrain);
  public final static SlalomIBarelyKnowEm slalom = new SlalomIBarelyKnowEm(Robot.driveTrain);
  public final static Beeline pathB = new Beeline();
  public final static PlanA pathA = new PlanA();
  public final static DoABarrelRoll barrelRoll = new DoABarrelRoll(Robot.driveTrain);
  public final static InelasticCollision bounce = new InelasticCollision(Robot.driveTrain);
  public static DriveTrain driveTrain;
  // public static int ballCount;
  public static boolean driveInverted;
  public static boolean yawBackwards;

  //Lights!
  public static double lights = -.45;
  Spark led;

  /*
  private int cycleCount = 0;
  private boolean recordStatus = false;
  private Object[][] commandValues = new Object[3][1500];
  private int n = 0; // index
  private double systemTimeStart = 0;
  private boolean running = false;
  private int m = 0; // index
  private int o = 0; // index
  private Object[][] commandVals = new Object[3][1500];
  private XboxController boxX = new XboxController(2);
  */
  private Goal goal;
  String gameData = "";

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {

    driveInverted = false;
    yawBackwards = false;
    m_robotContainer = new RobotContainer();
    driveTrain.resetOdometry(new Pose2d());
    RobotContainer.navX.resetYaw();
    led = new Spark(9); //Replace with real PWM channel

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removingsjhdfgas finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    gameData = DriverStation.getInstance().getGameSpecificMessage();
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    /*
     * if (SmartDashboard.getString("DB/String 1",
     * "").equalsIgnoreCase("Driver Input") && running == false) {
     * 
     * running = true;
     * 
     * try {
     * 
     * ObjectInputStream ois = new ObjectInputStream(new BufferedInputStream( new
     * FileInputStream("/home/lvuser/" + SmartDashboard.getString("DB/String 0",
     * "")))); commandVals = (Object[][]) ois.readObject(); ois.close();
     * 
     * } catch (Exception e1) {
     * 
     * e1.printStackTrace();
     * 
     * }
     * 
     * m = 0; o = 0; cycleCount = 0;
     * 
     * }
     */


    // schedule the autonomous command (example)

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    led.set(lights);
    /*
    if (running) {

      m = o;

      for (int p = m; (int) commandVals[3][p] != cycleCount; p++)
        o = p;

      for (int p = m; p < o; p++) {

        if ((int) commandVals[1][p] == 11)
          Robot.driveTrain.setDrive(.5 * ((Point) commandVals[2][p]).x, .5 * ((Point) commandVals[2][p]).y);

        if ((int) commandVals[1][p] == 12)
          Robot.driveTrain.setDrive(((Point) commandVals[2][p]).x, ((Point) commandVals[2][p]).y);

        if ((int) commandVals[1][p] == 13)
          Robot.driveTrain.setDrive(.25 * ((Point) commandVals[2][p]).x, .25 * ((Point) commandVals[2][p]).y);

        if ((int) commandVals[1][p] == 31)
          CommandScheduler.getInstance().schedule((RunIntake) m_robotContainer.getCommands().get(3));

        if ((int) commandVals[1][p] == 32)
          CommandScheduler.getInstance().schedule((RunIntakeBackwards) m_robotContainer.getCommands().get(4));

        if ((int) commandVals[1][p] == 33)
          CommandScheduler.getInstance().schedule((StopIntake) m_robotContainer.getCommands().get(5));

        if ((int) commandVals[1][p] == 41)
          CommandScheduler.getInstance().schedule((ToggleVision) m_robotContainer.getCommands().get(6));

        if ((int) commandVals[1][p] == 42)
          CommandScheduler.getInstance().schedule((AimAndShoot) m_robotContainer.getCommands().get(7));

      }

    }
    */

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    RobotContainer.navX.resetYaw();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    led.set(lights);

    SmartDashboard.putString("Rot2D", RobotContainer.navX.getRotation2d().toString());
    SmartDashboard.putNumber("Right Encoder", driveTrain.getRightEncoderValue());
    try {

      if (RobotContainer.EVSNetworkTables.getGoalArray().get(0).size() != 0) {

        goal = new Goal(RobotContainer.EVSNetworkTables.getGoalArray().get(0));

      } else {

        System.out.println("No Goal Found");

      }

    } catch (Exception e) {

    }
    try {
      SmartDashboard.putNumber("goal height", goal.getHeight());
    } catch (Exception e) {
      e.printStackTrace();
    }
    }
/*
    if (SmartDashboard.getString("DB/String 1", "").equalsIgnoreCase("Record") && !recordStatus) {
      recordStatus = true;
      systemTimeStart = System.currentTimeMillis() / 1000;
      System.out.println("Recording");
      n = 0;
      cycleCount = 0;
    }
    if (recordStatus) {
      if (!((DriveNormal) m_robotContainer.getCommands().get(0)).isFinished()) {

        commandValues[1][n] = 11;
        commandValues[2][n] = new Point(Robot.driveTrain.getJoystickL().getY(),
            Robot.driveTrain.getJoystickR().getY());
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((DriveMaxSpeed) m_robotContainer.getCommands().get(1)).isFinished()) {

        commandValues[1][n] = 12;
        commandValues[2][n] = new Point(Robot.driveTrain.getJoystickL().getY(),
            Robot.driveTrain.getJoystickR().getY());
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((DriveSlowSpeed) m_robotContainer.getCommands().get(2)).isFinished()) {

        commandValues[1][n] = 13;
        commandValues[2][n] = new Point(Robot.driveTrain.getJoystickL().getY(),
            Robot.driveTrain.getJoystickR().getY());
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((RunIntake) m_robotContainer.getCommands().get(3)).isFinished()) {

        commandValues[1][n] = 31;
        commandValues[2][n] = true;
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((RunIntakeBackwards) m_robotContainer.getCommands().get(4)).isFinished()) {

        commandValues[1][n] = 32;
        commandValues[2][n] = true;
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((StopIntake) m_robotContainer.getCommands().get(5)).isFinished()) {

        commandValues[1][n] = 33;
        commandValues[2][n] = true;
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((ToggleVision) m_robotContainer.getCommands().get(6)).isFinished()) {

        commandValues[1][n] = 41;
        commandValues[2][n] = true;
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (boxX.getAButton()) {

        commandValues[1][n] = 42;
        commandValues[2][n] = true;
        commandValues[3][n] = cycleCount;
        n++;

      }

      cycleCount++;

    }

    if (recordStatus && (System.currentTimeMillis() / 1000 > (systemTimeStart + 15))) {

      cycleCount = 0;
      n = 0;
      recordStatus = false;
      SmartDashboard.putString("DB/String 1", "");
      save(new File(SmartDashboard.getString("DB/String 0", "")));

    }

  }

  public void save(File file) {

    try {

      ObjectOutputStream oos = new ObjectOutputStream(
          new BufferedOutputStream(new FileOutputStream("/home/lvuser/" + file)));
      oos.writeObject(commandValues);
      oos.close();
      FileInputStream inputStream = new FileInputStream(file);
      byte[] buffer = new byte[(int) file.length()];
      inputStream.read(buffer);
      inputStream.close();
      URL url = new URL("ftp://anonymous@roborio-" + 834 + "-frc.local/home/lvuser/" + file);
      URLConnection conn = url.openConnection();
      conn.getOutputStream().write(buffer);
      conn.getOutputStream().close();
      file.delete();

    } catch (Exception e1) {

      e1.printStackTrace();

    }

    for (int i = 0; i < 3; i++) {

      for (int j = 0; j < 1500; j++) {

        commandValues[i][j] = null;

      }

    }

  }
*/
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
