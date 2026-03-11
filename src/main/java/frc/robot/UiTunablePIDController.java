package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class UiTunablePIDController extends PIDController {
    private final ShuffleboardTab shuffleboardTab;

    private final GenericEntry pEntry;
    private final GenericEntry iEntry;
    private final GenericEntry dEntry;

    private final GenericEntry measurement;
    private final GenericEntry setpoint;
    private final GenericEntry output;

    public UiTunablePIDController(double p, double i, double d) {
        super(p, i, d);
        this.shuffleboardTab = Shuffleboard.getTab("PID Tuning");
        this.pEntry = shuffleboardTab.add("p", p).getEntry();
        this.iEntry = shuffleboardTab.add("i", i).getEntry();
        this.dEntry = shuffleboardTab.add("d", d).getEntry();

        this.measurement = shuffleboardTab.add("measurement", 0).getEntry();
        this.setpoint = shuffleboardTab.add("setpoint", 0).getEntry();
        this.output = shuffleboardTab.add("output", 0).getEntry();
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        super.setP(this.pEntry.getDouble(super.getP()));
        super.setI(this.iEntry.getDouble(super.getI()));
        super.setD(this.dEntry.getDouble(super.getD()));

        this.setpoint.setDouble(setpoint);
        this.measurement.setDouble(measurement);

        double output = super.calculate(measurement, setpoint);
        this.output.setDouble(output);
        return output;
    }

    @Override
    public double calculate(double measurement) {
        super.setP(this.pEntry.getDouble(super.getP()));
        super.setI(this.iEntry.getDouble(super.getI()));
        super.setD(this.dEntry.getDouble(super.getD()));

        this.measurement.setDouble(measurement);

        double output = super.calculate(measurement);
        this.output.setDouble(output);
        return output;
    }
}
