package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterTarget;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class StickControllerConfig implements ControllerConfig {
    public static final CommandJoystick stickLeft = new CommandJoystick(2);
    public static final CommandJoystick stickRight = new CommandJoystick(3);

    @Override
    public void configureBindings(ShooterSubsystem shooter, IntakeSubsystem intake, SwerveSubsystem swerve) {
        SwerveInputStream stickDrive = SwerveInputStream.of(swerve.getSwerveDrive(),
                () -> getXTranslation(),
                () -> getYTranslation())
                .withControllerRotationAxis(() -> getRotation())
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        Command driveCommand = swerve.driveFieldOriented(stickDrive);
        swerve.setDefaultCommand(driveCommand);

        stickLeft.button(9).onTrue(Commands.runOnce(swerve::zeroGyro));

        stickLeft.button(3).onTrue(intake.up());
        stickLeft.button(2).onTrue(intake.down());
        stickRight.button(3).onTrue(intake.pickup());
        stickRight.button(2).onTrue(intake.stop());

        // stickLeft.button(5).onTrue(shooter.setDeflectorSpeed(() -> -stickLeft.getRawAxis(2))).onFalse(shooter.setDeflectorSpeed(() -> 0));
        // stickLeft.button(4).onTrue(shooter.setDeflectorSpeed(() -> stickLeft.getRawAxis(2))).onFalse(shooter.setDeflectorSpeed(() -> 0));
        stickLeft.button(11).onTrue(shooter.logEncoder());
        // stickLeft.button(10).onTrue(shooter.zeroEncoder());

        stickLeft.button(1).whileTrue(shooter.shootAt(() -> ShooterTarget.GOAL));
        stickRight.button(1).whileTrue(shooter.shootAt(() -> ShooterTarget.ALLIANCE));

        stickLeft.button(8).onTrue(shooter.homeDeflector());
        stickRight.button(4).onTrue(shooter.setKicker(() -> 1)).onFalse(shooter.setKicker(() -> 0));

        stickRight.button(10).whileTrue(shooter.unstuck());
        stickRight.button(11).whileTrue(intake.eject());

    }

    private double getRotation() {
        return (stickLeft.getY() - stickRight.getY()) / 2;
    }

    // forward and backward movement
    private double getXTranslation() {
        return (stickLeft.getY() + stickRight.getY()) / 2 * -1;
    }

    // leftward and rightward movement
    private double getYTranslation() {
        return (stickLeft.getX() + stickRight.getX()) / 2 * -1;
    }
}
