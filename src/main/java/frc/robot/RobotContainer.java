// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTarget;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.dyn4j.geometry.Vector3;

import swervelib.SwerveInputStream;

public class RobotContainer {
  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final Field2d field2d = new Field2d();
  private final Vision vision = new Vision(field2d);

  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"),
      vision);
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(swerve, field2d);

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

  SwerveInputStream rightStickAngle = driveAngularVelocity.copy()
      .withControllerHeadingAxis(allianceRelative(driverXbox::getRightX),
          allianceRelative(driverXbox::getRightY))
      .headingWhile(true);

  SwerveInputStream pointAndDrive = driveAngularVelocity.copy().withControllerHeadingAxis(() -> -driverXbox.getRightX(),
      () -> -driverXbox.getRightY())
      .headingWhile(true);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData("Field", vision.field2d);
  }

  private DoubleSupplier allianceRelative(DoubleSupplier v) {
    return () -> {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        return v.getAsDouble();
      } else {
        return -v.getAsDouble();
      }
    };
  }

  private SendableChooser<Command> buildAutoChooser() {
    var autoChooser = AutoBuilder.buildAutoChooser();

    // autoChooser.addOption("Drive Forward", swerve.driveForward().withTimeout(1));
    autoChooser.setDefaultOption("Home Deflector", shooter.homeDeflector());
    autoChooser.addOption("Do Nothing", Commands.none());

    return autoChooser;
  }

  private void configureBindings() {
    driverXbox.povUp().onTrue(shooter.homeDeflector());

    if (DriverStation.isTest()) {
      shooter.setDefaultCommand(shooter.deflectorTo(() -> new Rotation2d(driverXbox.getRightX(), driverXbox.getRightY())));
      return;
    }

    Command driveCommand = swerve.driveFieldOriented(rightStickAngle);
    swerve.setDefaultCommand(driveCommand);

    // var redGoal = new Vector3(Units.inchesToMeters(469.11),
    // Units.inchesToMeters(158.84), Units.inchesToMeters(72));
    // shooter.setDefaultCommand(shooter.shootAbsolute(() -> redGoal));

    // var targetPrimary = Shuffleboard.getTab("Shooter").add("Primary RPM",
    // 1500).getEntry();
    // var targetSecondary = Shuffleboard.getTab("Shooter").add("Secondary RPM",
    // 1500).getEntry();
    // shooter.setDefaultCommand(shooter.run(() -> {
      // shooter.targetPrimaryRpm = targetPrimary.getDouble(1500);
      // shooter.targetSecondaryRpm = targetSecondary.getDouble(1500);
    // }));

    driverXbox.start().onTrue(Commands.runOnce(swerve::zeroGyro));

    driverXbox.rightTrigger().onTrue(shooter.setKicker(() -> 1)).onFalse(shooter.setKicker(() -> 0));
    driverXbox.leftStick().onTrue(shooter.toggle());
    
    driverXbox.leftBumper().onTrue(intake.up());
    driverXbox.rightBumper().onTrue(intake.down());
    driverXbox.y().onTrue(intake.pickup());
    driverXbox.b().onTrue(intake.stop());

    driverXbox.a().whileTrue(shooter.shootAt(() -> ShooterTarget.GOAL));
    driverXbox.x().whileTrue(shooter.shootAt(() -> ShooterTarget.ALLIANCE));

    driverXbox.povLeft().whileTrue(shooter.unstuck());
    driverXbox.povRight().whileTrue(intake.eject());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  public void robotInit() {
  }

  public void teleopInit() {
  }

  public void telopPeriodic() {
  }

  public void disabledInit() {
  }

  public void testInit() {
    shooter.reset();
  }

  public void testPeriodic() {
  }
}
