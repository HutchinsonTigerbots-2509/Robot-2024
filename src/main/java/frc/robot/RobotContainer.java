// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Arm.DownArm;
import frc.robot.Commands.Arm.UpArm;
import frc.robot.Commands.Intake.IntakeIn;
import frc.robot.Commands.Intake.IntakeOut;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Commands.Shooter.Stop;
import frc.robot.Constants.Constants;
import frc.robot.Subsystems.Door;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private Shooter sShooter = new Shooter();
  private Intake sIntake = new Intake();
  private Door sDoor = new Door();

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  // private final Joystick costick = new Joystick(1);
  private final CommandSwerveDrivetrain drivetrain = Constants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick.a().whileTrue(new Shoot(sShooter));
    joystick.b().whileTrue(new IntakeIn(sIntake));
    joystick.x().whileTrue(new IntakeOut(sIntake));
    joystick.leftBumper().whileTrue(new UpArm(sDoor));
    joystick.rightBumper().whileTrue(new DownArm(sDoor));

    // ShootBtn = new JoystickButton(joystick.getHID(), 0);
    // ShootBtn.whileTrue(new Shoot(sShooter));

    // IntakeBtn = new JoystickButton(joystick.getHID(), 1);
    // IntakeBtn.whileTrue(new IntakeIn(sIntake));

    // IntakeOutBtn = new JoystickButton(joystick.getHID(), 2);
    // IntakeOutBtn.whileTrue(new IntakeOut(sIntake));

    // IntakeBtn = new JoystickButton(costick, XboxController.Button.kRightBumper.value);
    // IntakeBtn.whileTrue(new IntakeIn(sIntake));

    SmartDashboard.putData(new IntakeIn(sIntake));

  }

  private Trigger ShootBtn;
  private Trigger IntakeBtn;
  private Trigger IntakeOutBtn;

  public RobotContainer() {
    configureBindings(
      
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Door getDoor()
{
  return sDoor;
}
}
