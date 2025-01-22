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
        // Orignal IDs:
        // SparkMax1: 15
        // SparkMax2: 16
        sparkMax1 = new SparkMax(15, MotorType.kBrushless);
        sparkMax2 = new SparkMax(16, MotorType.kBrushless);
        forwardLimitSwitch = sparkMax1.getForwardLimitSwitch();
        reverseLimitSwitch = sparkMax1.getReverseLimitSwitch();
    }
    
    public void reverse() {
        sparkMax1.set(-1.0);
        sparkMax2.set(-1.0);

        new RunCommand(() -> {
            if(reverseLimitSwitch.isPressed())
            {
                sparkMax1.set(0);
                sparkMax2.set(0);
            }
        }, this).until(() -> {
            return reverseLimitSwitch.isPressed();
        });

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
