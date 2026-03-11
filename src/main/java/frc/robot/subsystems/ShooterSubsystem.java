package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UiTunablePIDController;

public class ShooterSubsystem extends SubsystemBase {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2);
    private static final double EXIT_ANGLE = 45.0;
    private static final double GRAVITY = 9.81;

    private final SwerveSubsystem swerve;

    private final SparkMax primary = new SparkMax(19, MotorType.kBrushless);
    private final PIDController primaryPid = new UiTunablePIDController(0.0, 0.0, 0);

    private final SparkMax secondary = new SparkMax(15, MotorType.kBrushless);
    private final PIDController secondaryPid = new PIDController(0.0, 0.0, 0.0);

    private final SparkMax deflector = new SparkMax(11, MotorType.kBrushless);
    private final PIDController deflectorPid = new PIDController(0.1, 0.0, 0.01);
    private static final double ENCODER_STEPS_PER_ROTATION = 3.0905;

    private final PWMVictorSPX kicker = new PWMVictorSPX(1);

    private double targetRpm = 0.0;
    private final double maxRpm = 2500;

    // Rotation relative to robot. 0 is shooting forward.
    private Rotation2d targetDeflectorRotation = Rotation2d.kZero;

    public ShooterSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;
    }

    public Command dontShoot() {
        return this.run(() -> {
            targetRpm = 0.0;
        });
    }

    public Command powerFromSupplier(DoubleSupplier power) {
        return this.run(() -> {
            double now = power.getAsDouble();
            targetRpm = maxRpm * now;
        });
    }

    public Command shootAbsolute(Supplier<Vector3> globalTarget) {
        return this.run(() -> {
            Vector3 target = globalTarget.get();
            var robotPose = this.swerve.getPose();

            targetDeflectorRotation = deflectorRotation(robotPose, target);

            // Vector2 distanceAlongShootingPlane = shootingPlane(robotPose, target);
            // setSpeedForShootingPlane(distanceAlongShootingPlane);
        });
    }

    private Rotation2d deflectorRotation(Pose2d robotPose, Vector3 target) {
        var offset = new Translation2d(-1, 1);
        var rotatedOffset = offset.rotateBy(robotPose.getRotation());
        var shooterPosition = rotatedOffset.plus(robotPose.getTranslation());

        var shooterToTarget = new Translation2d(target.x, target.y).minus(shooterPosition);
        var globalShooterAngle = shooterToTarget.getAngle();
        return globalShooterAngle.minus(robotPose.getRotation());
    }

    public Command testShoot(DoubleSupplier distanceMeters) {
        return this.run(() -> {
            double dx = distanceMeters.getAsDouble();
            double deflectorExitPointHeight = .508;

            setSpeedForShootingPlane(new Vector2(dx, deflectorExitPointHeight));
        });
    }

    private void setSpeedForShootingPlane(Vector2 delta) {
        double sin = Math.sin(Math.toRadians(EXIT_ANGLE));
        double cos = Math.cos(Math.toRadians(EXIT_ANGLE));

        double numerator = GRAVITY * delta.x;
        double denominator = 2 * cos * (((delta.y * cos) / delta.x) - sin);

        double exitSpeed = Math.sqrt(Math.abs(numerator / denominator));
        targetRpm = exitSpeed / (2 * Math.PI * WHEEL_RADIUS) * 60.;
    }

    @Override
    public void periodic() {
        shooterPeriodic();
        deflectorPeriodic();
    }

    private void shooterPeriodic() {
        double primaryBase = 0.00025 * targetRpm;
        double primaryPower = primaryPid.calculate(-primary.getEncoder().getVelocity(), targetRpm);
        double maxPrimaryPower = 1;
        primaryPower = MathUtil.clamp(primaryBase + primaryPower, -maxPrimaryPower, maxPrimaryPower);

        double secondaryBase = 0.00025 * targetRpm;
        double secondaryPower = secondaryPid.calculate(-secondary.getEncoder().getVelocity(), targetRpm);
        secondaryPower = MathUtil.clamp(secondaryBase + secondaryPower, -1, 1);

        primary.set(-primaryPower);
        secondary.set(-secondaryPower);
    }

    private void deflectorPeriodic() {
        var encoder = deflector.getEncoder().getPosition();
        var encoderRotation = Rotation2d.fromRadians(encoder / ENCODER_STEPS_PER_ROTATION * 2 * Math.PI);
        var error = targetDeflectorRotation.minus(encoderRotation);

        double power = deflectorPid.calculate(-error.getRadians());
        double maxPower = 0.1;
        deflector.set(MathUtil.clamp(power, -maxPower, maxPower));
    }

    public void setKicker(double speed) {
        kicker.set(speed);
    }

    public Command deflectorTo(Supplier<Rotation2d> rotation) {
        return this.run(() -> {
            targetDeflectorRotation = rotation.get();
        });
    }

    public void zeroDeflector() {
        deflector.getEncoder().setPosition(0);
    }
}
