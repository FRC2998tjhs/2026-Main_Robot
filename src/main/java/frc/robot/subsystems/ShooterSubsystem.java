package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.Supplier;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    // private static final double DEFLECTOR_POSITION_WHEN_SET = -0.0714285671710968;
    private static final double DEFLECTOR_POSITION_WHEN_SET = 0;

    private final SwerveSubsystem swerve;

    private final SparkMax primary = new SparkMax(19, MotorType.kBrushless);
    private final PIDController primaryPid = new PIDController(0.001, 0.0, 0);

    private final SparkMax secondary = new SparkMax(15, MotorType.kBrushless);
    private final PIDController secondaryPid = new PIDController(0.001, 0.0, 0.0);

    private final SparkMax deflector = new SparkMax(11, MotorType.kBrushless);
    private final PIDController deflectorPid = new PIDController(0.1, 0.0, 0.01);
    private static final double ENCODER_STEPS_PER_ROTATION = 3.0905;

    private final PWMVictorSPX kicker = new PWMVictorSPX(1);

    public double targetPrimaryRpm = 0.0;
    public double targetSecondaryRpm = 0.0;
    private final double maxRpm = 3500;

    // Rotation relative to robot. 0 is shooting forward.
    private Rotation2d targetDeflectorRotation = Rotation2d.kZero;

    private final Field2d field2d;

    public ShooterSubsystem(SwerveSubsystem swerve, Field2d field2d) {
        this.swerve = swerve;
        this.field2d = field2d;
    }

    public Command shootAbsolute(Supplier<Vector3> globalTarget) {
        return this.run(() -> {
            Vector3 target = globalTarget.get();
            field2d.getObject("Target").setPose(target.x, target.y, new Rotation2d());

            var robotPose = this.swerve.getPose();
            targetDeflectorRotation = deflectorRotation(robotPose, target);

            Vector2 distanceAlongShootingPlane = shootingPlane(robotPose, target);
            setSpeedForShootingPlane(distanceAlongShootingPlane);
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

    private Vector2 shootingPlane(Pose2d robotPose, Vector3 target) {
        var result = new Vector2();

        var dx = target.x - robotPose.getX();
        var dy = target.y - robotPose.getY();
        result.x = Math.sqrt(dx * dx + dy * dy);

        return result;
    }

    private void setSpeedForShootingPlane(Vector2 delta) {
        // double sin = Math.sin(Math.toRadians(EXIT_ANGLE));
        // double cos = Math.cos(Math.toRadians(EXIT_ANGLE));

        // double numerator = GRAVITY * delta.x;
        // double denominator = 2 * cos * (((delta.y * cos) / delta.x) - sin);

        // double exitSpeed = Math.sqrt(Math.abs(numerator / denominator));
        // targetRpm = exitSpeed / (2 * Math.PI * WHEEL_RADIUS) * 60.;
        targetPrimaryRpm = 1800;
        targetSecondaryRpm = 1800;
    }

    @Override
    public void periodic() {
        shooterPeriodic();
        deflectorPeriodic();
    }

    private void shooterPeriodic() {
        double primaryBase = 0.00025 * targetPrimaryRpm;
        double primaryPower = primaryPid.calculate(-primary.getEncoder().getVelocity(), targetPrimaryRpm);
        primaryPower = MathUtil.clamp(primaryBase + primaryPower, -1, 1);
        primary.set(-primaryPower);

        double secondaryBase = 0.00022 * targetSecondaryRpm;
        double secondaryPower = secondaryPid.calculate(-secondary.getEncoder().getVelocity(), targetSecondaryRpm);
        secondaryPower = MathUtil.clamp(secondaryBase + secondaryPower, -1, 1);
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
        kicker.set(-speed);
    }

    public Command deflectorTo(Supplier<Rotation2d> rotation) {
        return this.run(() -> {
            targetDeflectorRotation = rotation.get();
        });
    }

    public void zeroDeflector() {
        System.out.println(deflector.getEncoder().getPosition());
        deflector.getEncoder().setPosition(DEFLECTOR_POSITION_WHEN_SET);
    }

    public Command unstuck() {
        return this.run(() -> {
            targetPrimaryRpm = -maxRpm;
            setKicker(-1);
        });
    }
}
