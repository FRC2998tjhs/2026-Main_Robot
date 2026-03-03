package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final PWMVictorSPX rightLiftMotor = new PWMVictorSPX(0);
    private final SparkMax leftLiftMotor = new SparkMax(3, MotorType.kBrushless);
    private final SparkMax pickup = new SparkMax(8, MotorType.kBrushless);

    private final double maxPickupRpm = 2000;
    private double targetPickupRpm = 0;
    private final PIDController pickupPid = new PIDController(0.0001, 0.0002, 0);

    private final double liftSpeedMax = 0.5;
    private final double leftSpeedMultiple = 1.68;
    private final double liftTime = 1.5;

    // Speed from -1 to 1. 1 is up, -1 is down.
    public void setLiftSpeed(double speed) {
        rightLiftMotor.set(-liftSpeedMax * speed);
        leftLiftMotor.set(liftSpeedMax * leftSpeedMultiple * speed);
    }

    public Command up() {
        return Commands.sequence(
                this.runOnce(() -> {
                    this.setLiftSpeed(1);
                }),
                Commands.waitSeconds(liftTime),
                this.runOnce(() -> {
                    this.setLiftSpeed(0);
                }));
    }

    public Command down() {
        return Commands.sequence(
                this.runOnce(() -> {
                    this.setLiftSpeed(-1);
                }),
                Commands.waitSeconds(liftTime),
                this.runOnce(() -> {
                    this.setLiftSpeed(0);
                }));
    }

    public Command pickup() {
        return Commands.runOnce(() -> {
            targetPickupRpm = maxPickupRpm;
        });
    }
    
    public Command eject() {
        return Commands.runOnce(() -> {
            targetPickupRpm = -maxPickupRpm;
        });
    }
    
    public Command stop() {
        return Commands.runOnce(() -> {
            targetPickupRpm = 0;
        });
    }

    @Override
    public void periodic() {
        double velocity = pickup.getEncoder().getVelocity();
        if (Math.abs(velocity - targetPickupRpm) < 1) {
            return;
        }

        double speed = pickupPid.calculate(velocity, targetPickupRpm);
        var clamped = MathUtil.clamp(speed, -1, 1);

        System.out.println("Velocity: " + velocity + ", target: " + targetPickupRpm + ", speed: " + clamped);

        pickup.set(clamped);
    }
}
