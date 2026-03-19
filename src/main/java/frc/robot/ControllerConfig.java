package frc.robot;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public interface ControllerConfig {
    void configureBindings(ShooterSubsystem shooter, IntakeSubsystem intake, SwerveSubsystem swerve);

    public static DoubleSupplier allianceRelative(DoubleSupplier v) {
        return () -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                return v.getAsDouble();
            } else {
                return -v.getAsDouble();
            }
        };
    }
}
