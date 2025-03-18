package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    private SparkFlex lifter;
    private RelativeEncoder encoder;
    
    public Lift() {
        lifter = new SparkFlex(Constants.Lift.LIFTER_MOTOR_ID, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(2*Math.PI/Constants.Lift.GEAR_RATIO);
        lifter.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        encoder = lifter.getEncoder();
    }

    public void pull() {
        lifter.set(0.5);
    }

    public void release() {
        lifter.set(-0.5);
    }
}
