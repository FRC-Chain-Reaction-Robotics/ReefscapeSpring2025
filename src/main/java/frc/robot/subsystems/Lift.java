package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    private SparkFlex lifter;
    
    public Lift() {
        lifter = new SparkFlex(Constants.Lift.LIFTER_MOTOR_ID, MotorType.kBrushless);
    }

    public void pull() {

    }

    public void release() {
        getCurrentCommand();
    }
}
