package frc.robot;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

public class SafeSparkMax extends SparkMax {
    private Double dangerStart = null;
    private boolean enabled = true;

    public SafeSparkMax(int canid, MotorType motorType) {
        super(canid, motorType);
        CommandScheduler.getInstance().schedule(Commands.run(() -> this.periodic()));
    }

    @Override
    public void set(double speed) {
        if (!enabled) {
            super.set(0);
            return;
        }
        if (Math.abs(speed) >= .01 && dangerStart == null) {
            dangerStart = Timer.getFPGATimestamp();
        }
        super.set(speed);
    }

    private void periodic() {
        if (dangerStart != null && Timer.getFPGATimestamp() - dangerStart >= 2) {
            super.set(0);
            this.enabled = false;
            System.out.println("disabled motor with can " + super.getDeviceId());
        }
        if (Math.abs(super.getEncoder().getVelocity()) >= 20) {
            dangerStart = null;
        }
    }
}
