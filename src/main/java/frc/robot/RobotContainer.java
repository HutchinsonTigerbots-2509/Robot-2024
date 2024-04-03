// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Arm.DownArm;
import frc.robot.Commands.Arm.UpArm;
import frc.robot.Commands.Climber.ClimbExtend;
import frc.robot.Commands.Climber.ClimbRetract;
import frc.robot.Commands.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Commands.Drivetrain.ResetGyro;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeInAuto;
import frc.robot.Commands.Intake.IntakeOut;
import frc.robot.Commands.Intake.IntakeShoot;
import frc.robot.Commands.PresetPos.SafePos;
import frc.robot.Commands.PresetPos.ShootFarPos;
import frc.robot.Commands.PresetPos.WallPosition;
import frc.robot.Commands.PresetPos.FeedStationPosition;
import frc.robot.Commands.PresetPos.MainPos;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Commands.Shooter.ShootReverse;
import frc.robot.Commands.Shooter.ShootStart;
import frc.robot.Commands.Shooter.Stop;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Door;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.PathPlannerDrive;
import frc.robot.Subsystems.Shooter;

public class RobotContainer extends SubsystemBase{

  private Shooter sShooter = new Shooter();
  private Intake sIntake = new Intake();
  private Door sDoor = new Door();
  private Climber sClimb = new Climber();
  private DriveSubsystem sDrivetrain = new DriveSubsystem();
  private PathPlannerDrive sPathPlannerDrive = new PathPlannerDrive(sDrivetrain);

  /* Setting up bindings for necessary control of the swerve drive platform */
  final CommandXboxController joystick = new CommandXboxController(0); // Driver Joystick
  final Joystick ButtonBoardPrimary = new Joystick(1);
  final Joystick ButtonBoardSecondary = new Joystick(2);
  final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain
  private final Field2d field;


  // Autochooser
  SendableChooser<PathPlannerPath> AutoSelectPath = new SendableChooser<>();
  SendableChooser<String> AutoSelect = new SendableChooser<>();
  List<Pair<String,Command>> Commands;

  private void configureBindings() {


    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    final Telemetry logger = new Telemetry(sDrivetrain.MaxSpeed);



    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Drivetrain

    joystick.rightTrigger().whileTrue(sDrivetrain.cmdToggleGear());
    joystick.leftTrigger().whileTrue(sDrivetrain.cmdToggleGear());

    // Shooter

    Trigger ShootBtn;
    ShootBtn = new JoystickButton(ButtonBoardPrimary, 6);
    ShootBtn.whileTrue(new Shoot(sShooter));

    Trigger ShootReverseBtn;
    ShootReverseBtn = new JoystickButton(ButtonBoardSecondary, 10);
    ShootReverseBtn.whileTrue(new ShootReverse(sShooter));

    // Intake

    Trigger IntakeInBtn;
    IntakeInBtn = new JoystickButton(ButtonBoardPrimary, 8);
    IntakeInBtn.whileTrue(new IntakeIn(sIntake));
    Trigger IntakeOutBtn;
    IntakeOutBtn = new JoystickButton(ButtonBoardPrimary, 7);
    IntakeOutBtn.whileTrue(new IntakeOut(sIntake));

    Trigger BypassProximitySensorBtn;
    BypassProximitySensorBtn = new JoystickButton(ButtonBoardSecondary, 11);
    BypassProximitySensorBtn.onTrue(sIntake.cmdBypassProxSens());

    // Door

    Trigger DoorUpBtn;
    DoorUpBtn = new JoystickButton(ButtonBoardPrimary, 3);
    DoorUpBtn.whileTrue(new UpArm(sDoor));

    Trigger DoorDownBtn;
    DoorDownBtn = new JoystickButton(ButtonBoardPrimary, 2);
    DoorDownBtn.whileTrue(new DownArm(sDoor));
    
    // Climber

    Trigger ClimbExtendBtn;
    ClimbExtendBtn = new JoystickButton(ButtonBoardPrimary, 4);
    ClimbExtendBtn.whileTrue(new ClimbExtend(sClimb));

    Trigger ClimbRetractBtn;
    ClimbRetractBtn = new JoystickButton(ButtonBoardPrimary, 5);
    ClimbRetractBtn.whileTrue(new ClimbRetract(sClimb));

    // Preset Poses

    Trigger WallPosBtn;
    WallPosBtn = new JoystickButton(ButtonBoardPrimary, 11);
    WallPosBtn.onTrue(new WallPosition(sDoor));
    // WallPosBtn.onFalse(new SafePos(sShooter, sDoor));

    Trigger FeedStationPosBtn;
    FeedStationPosBtn = new JoystickButton(ButtonBoardPrimary, 12);
    FeedStationPosBtn.onTrue(new FeedStationPosition(sDoor));

    Trigger MainPosBtn;
    MainPosBtn = new JoystickButton(ButtonBoardPrimary, 9);
    MainPosBtn.onTrue(new MainPos(sDoor));
    //MainPosBtn.onFalse(new SafePos(sShooter, sDoor));

    Trigger ShootFarPosBtn;
    ShootFarPosBtn = new JoystickButton(ButtonBoardPrimary, 10);
    ShootFarPosBtn.onTrue(new ShootFarPos(sShooter, sDoor));
    //ShootFarPosBtn.onFalse(new SafePos(sShooter, sDoor));

    Trigger ResetGyroBtn;
  ResetGyroBtn = new JoystickButton(ButtonBoardSecondary, 12);
  ResetGyroBtn.whileTrue(new ResetGyro(sDrivetrain));

    SmartDashboard.putData(new IntakeIn(sIntake));
    
  }
  
  public RobotContainer() {

    field = new Field2d();

    // Named Commands

    NamedCommands.registerCommand("ShootStart", new ShootStart(sShooter).withTimeout(.1));
    NamedCommands.registerCommand("ShootStop", new Stop(sShooter).withTimeout(.1));

    NamedCommands.registerCommand("IntakeIn1", new IntakeInAuto(sIntake).withTimeout(1));
    NamedCommands.registerCommand("IntakeIn2", new IntakeInAuto(sIntake).withTimeout(2));
    NamedCommands.registerCommand("IntakeIn3", new IntakeInAuto(sIntake).withTimeout(3));
    NamedCommands.registerCommand("IntakeIn4", new IntakeInAuto(sIntake).withTimeout(4));

    NamedCommands.registerCommand("IntakeOut1", new IntakeOut(sIntake).withTimeout(1));
    NamedCommands.registerCommand("IntakeOut2", new IntakeOut(sIntake).withTimeout(2));
    NamedCommands.registerCommand("IntakeOut3", new IntakeOut(sIntake).withTimeout(3));
    NamedCommands.registerCommand("IntakeOut4", new IntakeOut(sIntake).withTimeout(4));
    
    

    NamedCommands.registerCommand("IntakeShoot.5", new IntakeShoot(sIntake).withTimeout(.5));
    NamedCommands.registerCommand("IntakeShoot1", new IntakeShoot(sIntake).withTimeout(1));
    NamedCommands.registerCommand("IntakeShoot2", new IntakeShoot(sIntake).withTimeout(2));
    NamedCommands.registerCommand("IntakeShoot3", new IntakeShoot(sIntake).withTimeout(3));
    NamedCommands.registerCommand("IntakeShoot4", new IntakeShoot(sIntake).withTimeout(4));


    NamedCommands.registerCommand("Shoot", new Shoot(sShooter));
    NamedCommands.registerCommand("ShootReverse", new ShootReverse(sShooter));
    NamedCommands.registerCommand("IntakeIn", new IntakeInAuto(sIntake));
    NamedCommands.registerCommand("IntakeOut", new IntakeOut(sIntake));
    NamedCommands.registerCommand("MainPos", new MainPos(sDoor).withTimeout(2));
    NamedCommands.registerCommand("SafePos", new SafePos(sShooter, sDoor).withTimeout(2));
    NamedCommands.registerCommand("ShootFarPos", new ShootFarPos(sShooter, sDoor).withTimeout(2));
    
    AutoSelect.setDefaultOption("Middle 3 Rings", "Middle 3 Ring");

    AutoSelect.addOption("Middle 2 Rings", "Middle 2 Ring");

    AutoSelect.addOption("Wall GnG", "Wall GnG");

    AutoSelect.addOption("Field Shot1 Set", "Field Long1 Set");

    AutoSelect.addOption("Potato", "Potato");

    SmartDashboard.putData(AutoSelect);
    SmartDashboard.putData(field);

    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      field.setRobotPose(pose);
    });

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      field.getObject("target pose").setPose(pose);
    });

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });


    configureBindings();
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto(AutoSelect.getSelected());
  }

  public Door getDoor()
{
  return sDoor;
}

  public Intake getIntake()
{
  return sIntake;
}

public DriveSubsystem getDrivetrain()
{
  return sDrivetrain;
}

public PathPlannerDrive getPathPlannerDrive()
{
  return sPathPlannerDrive;
}

}
