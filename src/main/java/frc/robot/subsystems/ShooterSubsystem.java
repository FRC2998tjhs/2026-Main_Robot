package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Vector2;
import org.dyn4j.geometry.Vector3;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SafeSparkMax;
import frc.robot.UiTunablePIDController;

public class ShooterSubsystem extends SubsystemBase {

    private static final int DEFAULT_MIN_RPM = 1350;
    // private static final int MINIMUM_RPM_TO_NOT_JAM = 1200;
    // private static final double GOAL_LEFT_SIDE = 352.893;
    // private static final double GOAL_RIGHT_SIDE = 282.483;
    // // private static final double DEFLECTOR_POSITION_WHEN_SET =
    // // -0.0714285671710968;
    private static final double DEFLECTOR_POSITION_WHEN_SET = -8.4;
    private static final int deflectorHeight = 17;

    private final SwerveSubsystem swerve;

    private final SparkMax primary = new SafeSparkMax(19, MotorType.kBrushless);
    private final PIDController primaryPid = new PIDController(0.001, 0.0, 0);

    private final SparkMax secondary = new SafeSparkMax(15, MotorType.kBrushless);
    private final PIDController secondaryPid = new PIDController(0.001, 0.0, 0.0);

    private final SparkMax deflector = new SparkMax(11, MotorType.kBrushless);
    private final PIDController deflectorPid = new UiTunablePIDController(1.2, 0.0, 0.05);
    private static final double ENCODER_STEPS_PER_ROTATION = 49.468939208984375;
    private final DigitalInput homingLimit = new DigitalInput(0);
    private boolean isHomed = false;

    private final SparkMax kicker = new SparkMax(17, MotorType.kBrushless);

    public double targetRpm = 0.0;
    private final double maxRpm = 3500;

    // Rotation relative to robot. 0 is shooting forward.
    private Rotation2d targetDeflectorRotation = Rotation2d.kZero;

    private final Field2d field2d;
    private GenericEntry shooterStatus;
    private GenericEntry minRPM;
    private GenericEntry primaryTableRpm;
    private double kickerSpeed = 0;
    private GenericEntry secondaryTableRpm;
    private GenericEntry dist;

    public void reset() {
        // isHomed = false;
        // targetRpm = 0.0;
        // targetDeflectorRotation = Rotation2d.kZero;
    }

    public Command toggle() {
        return Commands.runOnce(() -> {
            shooterStatus.setBoolean(!shooterStatus.getBoolean(true));
        });
    }

    public Command setKicker(DoubleSupplier speed) {
        return Commands.runOnce(() -> {
            if (shooterStatus.getBoolean(true)) {
                kickerSpeed = speed.getAsDouble();
            }
        });
    }

    public ShooterSubsystem(SwerveSubsystem swerve, Field2d field2d) {
        this.swerve = swerve;
        this.field2d = field2d;

        this.shooterStatus = Shuffleboard.getTab("Shooter").add("Shooter Status", true).getEntry();
        this.minRPM = Shuffleboard.getTab("Shooter").add("Min RPM", DEFAULT_MIN_RPM).getEntry();
        this.primaryTableRpm = Shuffleboard.getTab("Shooter").add("Primary RPM", primary.getEncoder().getVelocity())
                .getEntry();
        this.secondaryTableRpm = Shuffleboard.getTab("Shooter").add("Secondary RPM", primary.getEncoder().getVelocity())
                .getEntry();
        this.dist = Shuffleboard.getTab("Shooter").add("Distance", 10).getEntry();

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

    public Command shootAt(Supplier<ShooterTarget> globalTarget) {
        Vector3 redGoal = new Vector3(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72));
        Vector3 blueGoal = new Vector3(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84),
                Units.inchesToMeters(72));

        return this.shootAbsolute(() -> {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (!alliance.isPresent()) {
                return new Vector3(-1, -1, -1);
            }

            boolean isBlue = alliance.get() == Alliance.Blue;

            if (globalTarget.get() == ShooterTarget.GOAL) {
                if (isBlue) {
                    return blueGoal;
                } else {
                    return redGoal;
                }
            }
            return targetAlliance(isBlue);
        });
    }

    private Vector3 targetAlliance(boolean isBlue) {
        Pose2d robotPose = this.swerve.getPose();
        return new Vector3(robotPose.getX(), Units.inchesToMeters(91.055 + (isBlue ? 469.11 : 0)), 0);
    }

    private Rotation2d deflectorRotation(Pose2d robotPose, Vector3 target) {
        var offset = new Translation2d(-0.1375, -0.0755);
        var rotatedOffset = offset.rotateBy(robotPose.getRotation());
        var shooterPosition = rotatedOffset.plus(robotPose.getTranslation());
        
        field2d.getObject("Shooter Pose").setPose(shooterPosition.getX(), shooterPosition.getY(), new Rotation2d());

        var shooterToTarget = new Translation2d(target.x, target.y).minus(shooterPosition);
        var globalShooterAngle = shooterToTarget.getAngle();
        return globalShooterAngle.minus(robotPose.getRotation());
    }

    private Vector2 shootingPlane(Pose2d robotPose, Vector3 target) {
        var result = new Vector2();

        var dx = target.x - robotPose.getX();
        var dy = target.y - robotPose.getY();
        result.x = Math.sqrt(dx * dx + dy * dy);
        // result.x = testDist.getDouble(35);

        result.y = target.z - Units.inchesToMeters(deflectorHeight);

        return result;
    }

    private void setSpeedForShootingPlane(Vector2 delta) {
        // double sin = Math.sin(Math.toRadians(EXIT_ANGLE));
        // double cos = Math.cos(Math.toRadians(EXIT_ANGLE));

        // double numerator = GRAVITY * delta.x;
        // double denominator = 2 * cos * (((delta.y * cos) / delta.x) - sin);

        // double exitSpeed = Math.sqrt(Math.abs(numerator / denominator));
        // targetRpm = exitSpeed / (2 * Math.PI * WHEEL_RADIUS) * 60.;

        targetRpm = 1300;

        // targetRpm = 841 * Math.exp(0.0025 * dist.getDouble(10));
    }

    @Override
    public void periodic() {
        if (!isHomed) {
            return;
        }
        shooterPeriodic();
        deflectorPeriodic();

        primaryTableRpm.setDouble(primary.getEncoder().getVelocity());
        secondaryTableRpm.setDouble(secondary.getEncoder().getVelocity());
    }

    private void shooterPeriodic() {
        if (!shooterStatus.getBoolean(true)) {
            primary.set(0);
            secondary.set(0);
            return;
        }

        double primaryMeasuredRpm = -primary.getEncoder().getVelocity();

        targetRpm = 841 * Math.exp(0.0025 * dist.getDouble(10));

        targetRpm = targetRpm < minRPM.getDouble(DEFAULT_MIN_RPM) ? minRPM.getDouble(DEFAULT_MIN_RPM) : targetRpm;
        double primaryBase = 0.00025 * targetRpm;
        double primaryPower = primaryPid.calculate(primaryMeasuredRpm, targetRpm);
        primaryPower = MathUtil.clamp(primaryBase + primaryPower, -1, 1);
        primary.set(-primaryPower);

        double secondaryBase = 0.00022 * targetRpm;
        double secondaryPower = secondaryPid.calculate(-secondary.getEncoder().getVelocity(), targetRpm);
        secondaryPower = MathUtil.clamp(secondaryBase + secondaryPower, -1, 1);
        secondary.set(-secondaryPower);

        if (primaryMeasuredRpm < minRPM.getDouble(DEFAULT_MIN_RPM) || primaryMeasuredRpm < targetRpm * 0.95) {
            kicker.set(0);
            return;
        }
        kicker.set(kickerSpeed);
    }

    private void deflectorPeriodic() {
        // if (true)
        //     return;
        var encoder = deflector.getEncoder().getPosition();
        var encoderRotation = Rotation2d.fromRadians((encoder) / ENCODER_STEPS_PER_ROTATION * 2 * Math.PI);
        var error = targetDeflectorRotation.minus(encoderRotation);

        double power = deflectorPid.calculate(-error.getRadians());
        deflector.set(power);
    }

    public Command deflectorTo(Supplier<Rotation2d> rotation) {
        return this.run(() -> {
            targetDeflectorRotation = rotation.get();
        });
    }

    public Command setDeflectorSpeed(DoubleSupplier speed) {
        return this.run(() -> {
            deflector.set(speed.getAsDouble());
        });
    }

    public Command homeDeflector() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    isHomed = false;
                    deflector.set(-0.8);
                    System.out.println("starting homing");
                }),
                Commands.waitUntil(() -> !homingLimit.get()),
                Commands.runOnce(() -> {
                    deflector.set(0.10);
                }),
                Commands.waitUntil(() -> !homingLimit.get()),
                Commands.runOnce(() -> {
                    deflector.set(0.02);
                }),
                Commands.waitUntil(() -> homingLimit.get()),
                Commands.runOnce(() -> {
                    deflector.set(0.0);
                    isHomed = true;
                    deflector.getEncoder().setPosition(DEFLECTOR_POSITION_WHEN_SET);
                    System.out.println("stopping homing");
                }));
    }

    public Command unstuck() {
        return this.run(() -> {
            targetRpm = -maxRpm;
            kickerSpeed = 1.;
        });
    }

    public Command logEncoder() {
        return Commands.runOnce(() -> {
            System.out.println(deflector.getEncoder().getPosition());
        });
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> {
            deflector.getEncoder().setPosition(0);
            System.out.println("zeroed encoder");
        });
    }
}
