package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax primary = new SparkMax(19, MotorType.kBrushless);
    private final SparkMax secondary = new SparkMax(15, MotorType.kBrushless);

    private final SparkMax deflector = new SparkMax(16, MotorType.kBrushless);

    public Command powerFromSupplier(DoubleSupplier power) {
        return this.run(() -> {
            double now = power.getAsDouble();
            // System.out.println(primary.getAbsoluteEncoder().getVelocity());
            primary.set(-now);
            secondary.set(-now);
        });
    }
}
