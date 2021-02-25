
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import frc.robot.commands.*;
import frc.robot.commands.autonomous.autons.*;
import frc.robot.commands.autonomous.AimAndShoot;
import frc.robot.commands.autonomous.EmptyShooterNoVision;
import frc.robot.commands.autonomous.ShooterToSpeed;
import frc.robot.commands.autonomous.autons.TestAuto;
import frc.robot.commands.Conveyor.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.hood.*;
import frc.robot.commands.snapto.*;
import frc.robot.commands.vision.ToggleVision;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GimbalLock;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.EVSNetworkTables;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  //Subsystems
  public static DriveTrain driveTrain;
  public static BallIntake ballIntake;
  //public static ControlPanelManip controlPanelManip;
  public static Shooter shooter;
  public static String gameData;
  public static Conveyor conveyor;
  public static EVSNetworkTables EVSNetworkTables;
  public UsbCamera camera;
  public static GimbalLock gimbalLock;
  public static NavX navX;
  //Commands
  private final DriveNormal driveNormal = new DriveNormal();
  private final DriveSlowSpeed driveSlowSpeed = new DriveSlowSpeed();
  private final DriveMaxSpeed driveMaxSpeed = new DriveMaxSpeed();
  public final DriveInverted driveInverted = new DriveInverted();

  private final RunIntake runIntake = new RunIntake();
  private final RunIntakeBackwards runIntakeBackwards = new RunIntakeBackwards();
  private final StopIntake stopIntake = new StopIntake();

  private final RunShooter runShooter = new RunShooter();
  private final StopShooter stopShooter = new StopShooter();

  private final RunPivotUp runPivotUp = new RunPivotUp();
  private final RunPivotDown runPivotDown = new RunPivotDown();

  private final RunConveyor runConveyor = new RunConveyor();
  private final RunConveyorSensor runConveyorSensor = new RunConveyorSensor();
  private final RunConveyorBackward runConveyorBackward = new RunConveyorBackward();
  private final StopConveyor stopConveyor = new StopConveyor();

  private final ToggleVision toggleVision = new ToggleVision();
  private final AimAndShoot aimAndShoot = new AimAndShoot();
  private final ShooterToSpeed shooterToSpeed = new ShooterToSpeed();
  private final DriveBackwardsDistance driveBackwardsDistance = new DriveBackwardsDistance(12);
  private final EmptyShooterNoVision emptyShooterNoVision = new EmptyShooterNoVision();
  private final ResetYaw resetYaw = new ResetYaw();
  private final SnapTo0 snapTo0 = new SnapTo0();
  private final SnapTo180 snapTo180 = new SnapTo180();
  private final DriveForwardDistance driveForwardDistance = new DriveForwardDistance(.25, 69);
  private final TestAuto testAuto = new TestAuto(driveTrain);

  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  private final XboxController xbox = new XboxController(2);
  private final Joystick launchpad = new Joystick(3);
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  // Joystick Buttons
  private final JoystickButton
  // Left Joystick
  lJoystick1 = new JoystickButton(leftJoystick, 1), lJoystick2 = new JoystickButton(leftJoystick, 2),
      lJoystick3 = new JoystickButton(leftJoystick, 3), lJoystick4 = new JoystickButton(leftJoystick, 4),
      lJoystick5 = new JoystickButton(leftJoystick, 5), lJoystick6 = new JoystickButton(leftJoystick, 6),
      lJoystick7 = new JoystickButton(leftJoystick, 7), lJoystick8 = new JoystickButton(leftJoystick, 8),
      lJoystick9 = new JoystickButton(leftJoystick, 9), lJoystick10 = new JoystickButton(leftJoystick, 10),
      lJoystick11 = new JoystickButton(leftJoystick, 11),

      // Right Joystick
      rJoystick1 = new JoystickButton(rightJoystick, 1), rJoystick2 = new JoystickButton(rightJoystick, 2),
      rJoystick3 = new JoystickButton(rightJoystick, 3), rJoystick4 = new JoystickButton(rightJoystick, 4),
      rJoystick5 = new JoystickButton(rightJoystick, 5), rJoystick6 = new JoystickButton(rightJoystick, 6),
      rJoystick7 = new JoystickButton(rightJoystick, 7), rJoystick8 = new JoystickButton(rightJoystick, 8),
      rJoystick9 = new JoystickButton(rightJoystick, 9), rJoystick10 = new JoystickButton(rightJoystick, 10),
      rJoystick11 = new JoystickButton(rightJoystick, 11),

      // Arcade Buttons
      BGTL = new JoystickButton(launchpad, 7), BGTM = new JoystickButton(launchpad, 2),
      BGTR = new JoystickButton(launchpad, 4), BGML = new JoystickButton(launchpad, 1),
      BGMM = new JoystickButton(launchpad, 6), BGMR = new JoystickButton(launchpad, 3),
      BGBL = new JoystickButton(launchpad, 10), BGBM = new JoystickButton(launchpad, 9),
      BGBR = new JoystickButton(launchpad, 8);

  // Xbox Buttons
  private final JoystickButton xboxStart = new JoystickButton(xbox, Button.kStart.value), xboxBack = new JoystickButton(xbox, Button.kBack.value),
      xboxB = new JoystickButton(xbox, Button.kB.value), xboxA = new JoystickButton(xbox, Button.kA.value), xboxY = new JoystickButton(xbox, Button.kY.value),
      xboxX = new JoystickButton(xbox, Button.kX.value), xboxLB = new JoystickButton(xbox, Button.kBumperLeft.value), xboxRB = new JoystickButton(xbox, Button.kBumperRight.value),
      xboxLJB = new JoystickButton(xbox, 9);



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autonChooser.addOption("Test Path", testAuto);
    SmartDashboard.putData("Auto Chooser", autonChooser);
  }


  public ArrayList<Object> getCommands() {

    ArrayList<Object> t = new ArrayList<Object>();
    t.add(driveNormal);
    t.add(driveMaxSpeed);
    t.add(driveSlowSpeed);
    t.add(runIntake);
    t.add(runIntakeBackwards);
    t.add(stopIntake);
    t.add(toggleVision);
    t.add(aimAndShoot);

    return t;

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // drive speed buttons
    lJoystick1.whenPressed(driveSlowSpeed);
    rJoystick1.whenPressed(driveNormal);
    rJoystick2.whenPressed(driveMaxSpeed);
    lJoystick2.whenPressed(driveInverted);

    // shooter buttons
    xboxA.whenPressed(aimAndShoot);
    BGTL.toggleWhenPressed(runShooter);
    // BGTR.whileHeld(runPivotUp);
    // BGMR.whileHeld(runPivotDown);
    BGTR.whenPressed(snapTo0);
    BGMR.whenPressed(snapTo180);

    // conveyor/intake buttons
    xboxB.toggleWhenPressed(runConveyorSensor);
    xboxLB.whenHeld(runIntakeBackwards);
    xboxRB.whenHeld(runIntake);
    BGMM.whenHeld(runConveyor);
    // BGMR.whenHeld(runConveyorBackward);
    // add things for conveyor that I'm confused about
 

    // climber
  

    // BGML.whenPressed();
    // BGMM.whenPressed();
    // BGMR.whileHeld(runPivotUp);

    // BGBL.whenPressed();
    // BGBM.whenPressed();
    // BGBR.whileHeld(runPivotDown);

    /*
     * //xboxStart.whileHeld(); //xboxBack.whileHeld(); //xboxB.whileHeld();
     * //xboxA.whileHeld(); //xboxY.whileHeld(); //xboxX.whileHeld();
     * xboxLB.whileHeld(runIntakeBackwards); xboxRB.whileHeld(runIntake);
     * //xboxLJB.whileHeld();
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}