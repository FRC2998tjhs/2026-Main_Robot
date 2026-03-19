// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Vision;

import java.io.File;

public class RobotContainer {
  private final Field2d field2d = new Field2d();
  private final Vision vision = new Vision(field2d);

  private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"),
      vision);
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem(swerve, field2d);

  private final SendableChooser<Command> autoChooser;

  // private final ControllerConfig config = new XboxControllerConfig();
  private final ControllerConfig config = new StickControllerConfig();

  public RobotContainer() {
    config.configureBindings(shooter, intake, swerve);

    DriverStation.silenceJoystickConnectionWarning(true);

    autoChooser = buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    SmartDashboard.putData("Field", vision.field2d);
  }

  private SendableChooser<Command> buildAutoChooser() {
    var autoChooser = AutoBuilder.buildAutoChooser();

    // autoChooser.addOption("Drive Forward", swerve.driveForward().withTimeout(1));
    autoChooser.setDefaultOption("Home Deflector", shooter.homeDeflector());
    autoChooser.addOption("Do Nothing", Commands.none());

    return autoChooser;
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
