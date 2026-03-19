package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTarget;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class XboxControllerConfig implements ControllerConfig {
  private static final CommandXboxController driverXbox = new CommandXboxController(0);

  public void configureBindings(ShooterSubsystem shooter, IntakeSubsystem intake, SwerveSubsystem swerve) {
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1)
        .withControllerRotationAxis(() -> -driverXbox.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true);

    SwerveInputStream rightStickAngle = driveAngularVelocity.copy()
        .withControllerHeadingAxis(ControllerConfig.allianceRelative(driverXbox::getRightX),
            ControllerConfig.allianceRelative(driverXbox::getRightY))
        .headingWhile(true);

    SwerveInputStream stickDriveAngularVelocity = SwerveInputStream.of(swerve.getSwerveDrive(),
        () -> driverXbox.getLeftY() * -1,
        () -> driverXbox.getLeftX() * -1)
        .withControllerRotationAxis(() -> -driverXbox.getRightX())
        .deadband(OperatorConstants.DEADBAND)
        .scaleTranslation(0.8)
        .allianceRelativeControl(true)
        .withControllerHeadingAxis(ControllerConfig.allianceRelative(driverXbox::getRightX),
            ControllerConfig.allianceRelative(driverXbox::getRightY))
        .headingWhile(true);

    SwerveInputStream pointAndDrive = driveAngularVelocity.copy()
        .withControllerHeadingAxis(() -> -driverXbox.getRightX(),
            () -> -driverXbox.getRightY())
        .headingWhile(true);

    driverXbox.leftStick().onTrue(shooter.toggle());
    driverXbox.rightTrigger().onTrue(shooter.setKicker(() -> 1)).onFalse(shooter.setKicker(() -> 0));

    driverXbox.povUp().onTrue(shooter.homeDeflector());

    if (DriverStation.isTest()) {
      shooter.setDefaultCommand(
          shooter.deflectorTo(() -> new Rotation2d(-driverXbox.getRightY(), -driverXbox.getRightX())));
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

    driverXbox.leftBumper().onTrue(intake.up());
    driverXbox.rightBumper().onTrue(intake.down());
    driverXbox.y().onTrue(intake.pickup());
    driverXbox.b().onTrue(intake.stop());

    driverXbox.a().whileTrue(shooter.shootAt(() -> ShooterTarget.GOAL));
    driverXbox.x().whileTrue(shooter.shootAt(() -> ShooterTarget.ALLIANCE));

    driverXbox.povLeft().whileTrue(shooter.unstuck());
    driverXbox.povRight().whileTrue(intake.eject());
  }
}