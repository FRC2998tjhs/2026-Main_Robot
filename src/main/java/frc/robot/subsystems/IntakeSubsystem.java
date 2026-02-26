package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    /*
     * TODO
     * functions:
     * -> start pickup
     * -> stop pickup
     */

    private final PWMVictorSPX rightLiftMotor = new PWMVictorSPX(0);
    private final PWMVictorSPX leftLiftMotor = new PWMVictorSPX(1);

    private final SparkMax pickup = new SparkMax(8, MotorType.kBrushless);

    private final double pickupSpeed = 1;
    private final double liftSpeedMax = 1;
    private final double leftSpeedMultiple = 1.68;
    private final double liftTime = 1;

    // Speed from -1 to 1. 1 is up, -1 is down.
    private void setLiftSpeed(double speed) {
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
            pickup.set(pickupSpeed);
        });
    }

    public Command eject() {
        return Commands.runOnce(() -> {
            pickup.set(-pickupSpeed);
        });
    }

    public Command stop() {
        return Commands.runOnce(() -> {
            pickup.set(0);
        });
    }
}
