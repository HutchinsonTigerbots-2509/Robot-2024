// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Arm.DownArm;
import frc.robot.Commands.Arm.UpArm;
import frc.robot.Commands.Autos.DriveForward;
import frc.robot.Commands.Autos.Potato;
import frc.robot.Commands.Autos.PotatoShoot;
import frc.robot.Commands.Autos.TestDrive;
import frc.robot.Commands.Climber.ClimbExtend;
import frc.robot.Commands.Climber.ClimbRetract;
import frc.robot.Commands.Door.TeleDoorController;
import frc.robot.Commands.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Commands.Drivetrain.ResetGyro;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeOut;
import frc.robot.Commands.PresetPos.SafePos;
import frc.robot.Commands.PresetPos.ShootFarPos;
import frc.robot.Commands.PresetPos.ShootGoalTwoPos;
import frc.robot.Commands.PresetPos.WallPosition;
import frc.robot.Commands.PresetPos.FeedStationPosition;
import frc.robot.Commands.PresetPos.MainPos;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Commands.Shooter.ShootReverse;
import frc.robot.Commands.Shooter.Stop;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Door;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer extends SubsystemBase{

  private Shooter sShooter = new Shooter();
  private Intake sIntake = new Intake();
  private Door sDoor = new Door();
  private Climber sClimb = new Climber();
  private DriveSubsystem sDrivetrain = new DriveSubsystem();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // Driver Joystick
    private final Joystick ButtonBoardPrimary = new Joystick(1);
  private final Joystick ButtonBoardSecondary = new Joystick(2);
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain


  // Autochooser
    SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Autos

  // private TestDrive cmdTestDrive =
  // new TestDrive(sDrivetrain, sIntake, sDoor, sShooter);

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
    ShootReverseBtn = new JoystickButton(ButtonBoardSecondary, 11);
    ShootReverseBtn.whileTrue(new ShootReverse(sShooter));

    // Intake

    Trigger IntakeInBtn;
    IntakeInBtn = new JoystickButton(ButtonBoardPrimary, 8);
    IntakeInBtn.whileTrue(new IntakeIn(sIntake));
    Trigger IntakeOutBtn;
    IntakeOutBtn = new JoystickButton(ButtonBoardPrimary, 7);
    IntakeOutBtn.whileTrue(new IntakeOut(sIntake));

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
    WallPosBtn.onTrue(new WallPosition(sShooter, sDoor));
    // WallPosBtn.onFalse(new SafePos(sShooter, sDoor));

    Trigger FeedStationPosBtn;
    FeedStationPosBtn = new JoystickButton(ButtonBoardPrimary, 12);
    FeedStationPosBtn.onTrue(new FeedStationPosition(sShooter, sDoor));

     Trigger MainPosBtn;
     MainPosBtn = new JoystickButton(ButtonBoardPrimary, 9);
     MainPosBtn.onTrue(new MainPos(sShooter, sDoor));
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

      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> DriveSubsystem.drive.withVelocityX(DriveSubsystem.swerveY(joystick) * DriveSubsystem.speedValue) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(DriveSubsystem.swerveX(joystick) * DriveSubsystem.speedValue) // Drive left with negative X (left)
            .withRotationalRate(DriveSubsystem.swerveZ(joystick) * DriveSubsystem.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    AutoSelect.addOption("Test Drive", new TestDrive(sDrivetrain, sIntake, sDoor, sShooter));

    AutoSelect.addOption("Drive Forward", new DriveForward(sDrivetrain, sIntake, sDoor, sShooter));

    AutoSelect.addOption("Potato", new Potato(sDrivetrain, sIntake, sDoor, sShooter));

    AutoSelect.addOption("Potato & Shoot", new PotatoShoot(sDrivetrain, sIntake, sDoor, sShooter));

    SmartDashboard.putData(AutoSelect);

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return AutoSelect.getSelected();
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

}
