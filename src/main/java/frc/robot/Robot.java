/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DriveMaxSpeed;
import frc.robot.commands.DriveNormal;
import frc.robot.commands.DriveSlowSpeed;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeBackwards;
import frc.robot.commands.StopIntake;
import frc.robot.commands.autonomous.AimAndShoot;
import frc.robot.commands.vision.ToggleVision;
import frc.robot.createdclasses.Goal;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.EVSNetworkTables;
import frc.robot.subsystems.GimbalLock;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
//import frc.robot.subsystems.ControlPanelManip;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Climber;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static DriveTrain driveTrain;
  public static DriveNormal driveNormal;
  public static BallIntake ballIntake;
  public static Subsystem[] runIntake;
  //public static ControlPanelManip controlPanelManip;
  public static Shooter shooter;
  public static String gameData;
  public static Conveyor conveyor;
  public static EVSNetworkTables EVSNetworkTables;
  public UsbCamera camera;
  public static GimbalLock gimbalLock;
  public static NavX navX;
  public static Climber climber;

  private RobotContainer m_robotContainer;
  
  //public static int ballCount;
  public static boolean driveInverted;
  public static boolean yawBackwards;

  private int cycleCount = 0;
  private boolean recordStatus = false;
  private Object[][] commandValues = new Object[3][1500];
  private int n = 0;
  private double systemTimeStart = 0;
  private boolean running = false;
  private int m = 0;
  private int o = 0;
  private Object[][] commandVals = new Object[3][1500];
  private XboxController boxX = new XboxController(2);
  private Goal goal;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    navX = new NavX();
    driveTrain = new DriveTrain();
    driveNormal = new DriveNormal();
    shooter = new Shooter();
    gimbalLock = new GimbalLock();
    climber = new Climber();

    ballIntake = new BallIntake();
    //controlPanelManip = new ControlPanelManip();
    conveyor = new Conveyor();
    EVSNetworkTables = new EVSNetworkTables();

    gameData = "";
    //ballCount = 0;
    driveInverted = false;
    yawBackwards = false;

    m_robotContainer = new RobotContainer();
    //camera = CameraServer.getInstance().startAutomaticCapture();
    driveTrain.resetPose();
    navX.resetYaw();

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

    if (SmartDashboard.getString("DB/String 1", "").equalsIgnoreCase("Driver Input") && running == false) {

      running = true;

      try {

        ObjectInputStream ois = new ObjectInputStream(new BufferedInputStream(
            new FileInputStream("/home/lvuser/" + SmartDashboard.getString("DB/String 0", ""))));
        commandVals = (Object[][]) ois.readObject();
        ois.close();

      } catch (Exception e1) {

        e1.printStackTrace();

      }

      m = 0;
      o = 0;
      cycleCount = 0;

    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

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

    if (running) {

      m = o;

      for (int p = m; (int) commandVals[3][p] != cycleCount; p++)
        o = p;

      for (int p = m; p < o; p++) {

        if ((int) commandVals[1][p] == 11)
          driveTrain.setDrive(.5 * ((Point) commandVals[2][p]).x, .5 * ((Point) commandVals[2][p]).y);

        if ((int) commandVals[1][p] == 12)
          driveTrain.setDrive(((Point) commandVals[2][p]).x, ((Point) commandVals[2][p]).y);

        if ((int) commandVals[1][p] == 13)
          driveTrain.setDrive(.25 * ((Point) commandVals[2][p]).x, .25 * ((Point) commandVals[2][p]).y);

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

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    navX.resetYaw();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("YAW", navX.getYaw());
    SmartDashboard.putNumber("Right Encoder", driveTrain.getRightEncoderValue());
    try {

      if (Robot.EVSNetworkTables.getGoalArray().get(0).size() != 0) {

        goal = new Goal(Robot.EVSNetworkTables.getGoalArray().get(0));

      } else {

        System.out.println("No Goal Found");

      }

    } catch (Exception e) {

    }
    try {
    SmartDashboard.putNumber("goal height", goal.getHeight());
    }
    catch (Exception e){
      e.printStackTrace();
    }
    
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
        commandValues[2][n] = new Point(driveTrain.getJoystickL().getY(), driveTrain.getJoystickR().getY());
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((DriveMaxSpeed) m_robotContainer.getCommands().get(1)).isFinished()) {

        commandValues[1][n] = 12;
        commandValues[2][n] = new Point(driveTrain.getJoystickL().getY(), driveTrain.getJoystickR().getY());
        commandValues[3][n] = cycleCount;
        n++;

      }
      if (!((DriveSlowSpeed) m_robotContainer.getCommands().get(2)).isFinished()) {

        commandValues[1][n] = 13;
        commandValues[2][n] = new Point(driveTrain.getJoystickL().getY(), driveTrain.getJoystickR().getY());
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
