package frc.robot.subsystems;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Elevator extends SubsystemBase {
    private SparkMax sparkMax1, sparkMax2;
    private SparkLimitSwitch forwardLimitSwitch, reverseLimitSwitch;

    public Elevator()
    {
        sparkMax1 = new SparkMax(15, MotorType.kBrushless);
        sparkMax2 = new SparkMax(16, MotorType.kBrushless);
        forwardLimitSwitch = sparkMax1.getForwardLimitSwitch();
        reverseLimitSwitch = sparkMax1.getReverseLimitSwitch();

    }

    public void forward() {
        sparkMax1.set(1.0);
        sparkMax2.set(1.0);

        new RunCommand(() -> {
            if(forwardLimitSwitch.isPressed())
            {
                sparkMax1.set(0);
                sparkMax2.set(0);
            }
        }, this).until(() -> {
            return forwardLimitSwitch.isPressed();
        });

    }
    
}
