package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax pickup = new SparkMax(8, MotorType.kBrushless);

    private final SparkMax leftLiftMotor = new SparkMax(3, MotorType.kBrushless);
    private final double leftSpeedMax = 0.3;

    private final PWMVictorSPX rightLiftMotor = new PWMVictorSPX(0);
    private final double rightSpeedMax = 0.24;

    private final double maxPickupRpm = 4000;
    private double targetPickupRpm = 0;
    private final PIDController pickupPid = new PIDController(0.0001, 0.0, 0);

    // Speed from -1 to 1. 1 is up, -1 is down.
    public void setLiftSpeed(double speed) {
        rightLiftMotor.set(-rightSpeedMax * speed);
        leftLiftMotor.set(-leftSpeedMax * speed);
    }

    public Command up() {
        return Commands.sequence(
                this.runOnce(() -> {
                    this.setLiftSpeed(1);
                }),
                Commands.waitSeconds(0.5),
                this.runOnce(() -> {
                    this.setLiftSpeed(0);
                }));
    }

    public Command down() {
        return Commands.sequence(
                this.runOnce(() -> {
                    this.setLiftSpeed(-0.5);
                }),
                Commands.waitSeconds(0.6),
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

        double kF = 0.00026;
        double feedForward = kF * targetPickupRpm;

        double pid = pickupPid.calculate(velocity, targetPickupRpm);

        pickup.set(feedForward + pid);
    }
}
