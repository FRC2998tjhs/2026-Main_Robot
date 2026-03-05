// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;

import swervelib.SwerveInputStream;

public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final Vision vision = new Vision();

  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"),
      vision);
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(swerve);

  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled
   * by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
      () -> driverXbox.getLeftY() * -1,
      () -> driverXbox.getLeftX() * -1)
      .withControllerRotationAxis(() -> -driverXbox.getRightX())
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private SendableChooser<Command> buildAutoChooser() {
    var autoChooser = AutoBuilder.buildAutoChooser();

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward", swerve.driveForward().withTimeout(1));

    return autoChooser;
  }

  private void configureBindings() {
    Command driveFieldOrientedDirectAngle = swerve.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = swerve.driveFieldOriented(driveAngularVelocity);

    swerve.setDefaultCommand(driveFieldOrientedAngularVelocity);
    // shooter.setDefaultCommand(shooter.powerFromSupplier(() -> driverXbox.getRightTriggerAxis()));
    // shooter.setDefaultCommand(shooter.powerFromSupplier(() -> 1.0));

    driverXbox.start().onTrue((Commands.runOnce(swerve::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(swerve::addFakeVisionReading));

    driverXbox.leftBumper().onTrue(intake.up());
    driverXbox.rightBumper().onTrue(intake.down());
    driverXbox.y().onTrue(intake.pickup());
    driverXbox.b().onTrue(intake.stop());

    driverXbox.x().whileTrue(shooter.testShoot(() -> 10.0));
    driverXbox.a().whileTrue(shooter.testShoot(() -> 20.0));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void telopPeriodic() {
    if (driverXbox.leftBumper().getAsBoolean()) {
      intake.setLiftSpeed(1);
    } else if (driverXbox.rightBumper().getAsBoolean()) {
      intake.setLiftSpeed(-1);
    } else {
      intake.setLiftSpeed(0);
    }
    // mainShooter.set(-driverXbox.getRightTriggerAxis());
    // secondaryShooter.set(driverXbox.getRightTriggerAxis());
    // if (driverXbox.leftBumper().getAsBoolean()) {
    // left_pickup.set(-0.084);
    // right_pickup.set(0.05);
    // } else if (driverXbox.rightBumper().getAsBoolean()) {
    // left_pickup.set(0.084);
    // right_pickup.set(-0.05);
    // } else {
    // left_pickup.set(0);
    // right_pickup.set(0);
    // }
  }
}
