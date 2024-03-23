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
import frc.robot.Commands.Climber.ClimbExtend;
import frc.robot.Commands.Climber.ClimbRetract;
import frc.robot.Commands.Door.TeleDoorController;
import frc.robot.Commands.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeOut;
import frc.robot.Commands.PresetPos.SafePos;
import frc.robot.Commands.PresetPos.ShootFarPos;
import frc.robot.Commands.PresetPos.ShootGoalTwoPos;
import frc.robot.Commands.PresetPos.WallPosition;
import frc.robot.Commands.PresetPos.MainPos;
import frc.robot.Commands.Shooter.Shoot;
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
  private final Joystick ButtonBoardRight = new Joystick(2);
  private final Joystick ButtonBoardLeft = new Joystick(1);
  private final Joystick joystickPilot = new Joystick(3);
  // private final Joystick costick = new Joystick(1);
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain

  // Autochooser
    SendableChooser<Command> AutoSelect = new SendableChooser<>();

  // Autos


  private void configureBindings() {

    // Xbox Drive

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> DriveSubsystem.drive.withVelocityX(-joystick.getLeftY() * DriveSubsystem.speedValue) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * DriveSubsystem.speedValue) // Drive left with negative X (left)
            .withRotationalRate(joystick.getRightX() * DriveSubsystem.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));



        
      
    // Joystick Drive

    // drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(joystickPilot.getY() * speedValue) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(joystickPilot.getX() * speedValue) // Drive left with negative X (left)
    //         .withRotationalRate(joystickPilot.getZ() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //     ));

      
    //sDoor.setDefaultCommand(new TeleDoorController(sDoor.getDesiredPos(), sDoor));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    final Telemetry logger = new Telemetry(sDrivetrain.MaxSpeed);

    // reset the field-centric heading on left bumper press
    joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joystick.y().onTrue(sDrivetrain.cmdToggleGear());

    

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    // Drivetrain

    // Trigger creepBtn;
    // creepBtn = new JoystickButton(ButtonBoardLeft, 9);
    // creepBtn.onTrue(cmdToggleGear());
    // Shooter

    Trigger ShootBtn;
    Trigger ShootBtn2;
    ShootBtn = new JoystickButton(ButtonBoardRight, 11);
    ShootBtn2 = new JoystickButton(ButtonBoardLeft, 11);
    ShootBtn.whileTrue(new Shoot(sShooter));
    ShootBtn2.whileTrue(new Shoot(sShooter));

    // Intake

    Trigger IntakeInBtn;
    IntakeInBtn = new JoystickButton(ButtonBoardRight, 5);
    IntakeInBtn.whileTrue(new IntakeIn(sIntake));
    Trigger IntakeOutBtn;
    IntakeOutBtn = new JoystickButton(ButtonBoardRight, 6);
    IntakeOutBtn.whileTrue(new IntakeOut(sIntake));

    // Door

    Trigger DoorUpBtn;
    DoorUpBtn = new JoystickButton(ButtonBoardLeft, 5);
    DoorUpBtn.whileTrue(new UpArm(sDoor));

    Trigger DoorDownBtn;
    DoorDownBtn = new JoystickButton(ButtonBoardLeft, 6);
    DoorDownBtn.whileTrue(new DownArm(sDoor));
    
    // Climber

     Trigger ClimbExtendBtn;
     ClimbExtendBtn = joystick.rightBumper();
     ClimbExtendBtn.whileTrue(new ClimbExtend(sClimb));

     Trigger ClimbRetractBtn;
     ClimbRetractBtn = joystick.leftBumper();
     ClimbRetractBtn.whileTrue(new ClimbRetract(sClimb));

    // Preset Poses

    Trigger WallPosBtn;
    WallPosBtn = new JoystickButton(ButtonBoardLeft, 3);
    WallPosBtn.onTrue(new WallPosition(sShooter, sDoor));
    WallPosBtn.onFalse(new SafePos(sShooter, sDoor));

     Trigger MainPosBtn;
     MainPosBtn = new JoystickButton(ButtonBoardRight, 4);
     MainPosBtn.onTrue(new MainPos(sShooter, sDoor));
     //MainPosBtn.onFalse(new SafePos(sShooter, sDoor));

    Trigger ShootFarPosBtn;
    ShootFarPosBtn = new JoystickButton(ButtonBoardRight, 3);
    ShootFarPosBtn.onTrue(new ShootFarPos(sShooter, sDoor));
    //ShootFarPosBtn.onFalse(new SafePos(sShooter, sDoor));

    // Trigger ShootGoalTwoPosBtn;
    // ShootGoalTwoPosBtn = new JoystickButton(ButtonBoardRight, 4);
    // ShootGoalTwoPosBtn.onTrue(new ShootGoalTwoPos(sShooter, sDoor));
    // ShootGoalTwoPosBtn.onFalse(new SafePos(sShooter, sDoor));

    Trigger SafePosBtn;
    SafePosBtn = new JoystickButton(ButtonBoardRight, 2);
    SafePosBtn.onTrue(new SafePos(sShooter, sDoor));

    SmartDashboard.putData(new IntakeIn(sIntake));
  }
  

  public RobotContainer() {

    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
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
