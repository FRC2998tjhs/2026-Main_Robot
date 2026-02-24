package frc.robot.subsystems.swervedrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Transform2d;

public class ShooterSubsystem {
    private Vision vision;
    private SparkMax turnMotor;

    public ShooterSubsystem(Vision vision) {
        System.out.println("greg");
        this.vision = vision;
        // this.turnMotor = new SparkMax(19, MotorType.kBrushless);
    }

    public void turnThingy() {
        System.out.println(vision.getEstimatedGlobalPose(Vision.Cameras.LEFT_CAM));
        // vision.getAprilTagPose(4, new Transform2d(0,0));
    }
}
