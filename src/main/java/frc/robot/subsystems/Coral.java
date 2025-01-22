package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase {
    private SparkMax wheelMotor;
    private SparkMax turnMotor;
    private SparkAbsoluteEncoder angleReader;
    private double[] positions = {-60, 30, 0, 30, 60};
    
    public Coral() {
        wheelMotor = new SparkMax(17,  MotorType.kBrushless);
        turnMotor = new SparkMax(18,  MotorType.kBrushless);
        angleReader = turnMotor.getAbsoluteEncoder();
    }

    public void turn(int notch) {
        final double angleEpsilon = 0.5;
        final double angleRelativeTurnRate = 0.5;

        new RunCommand(() -> {
            double angleDiff = angleReader.getPosition() - positions[notch];
            if(Math.abs(angleDiff) >= angleEpsilon)
            {
                double speed = 0;
                if(angleDiff < 0)
                    speed = angleRelativeTurnRate;
                else if(angleDiff > 0)
                    speed = -angleRelativeTurnRate;

                turnMotor.set(speed);
            }
        }, this).until(() -> {
            return Math.abs(angleReader.getPosition() - positions[notch]) < angleEpsilon;
        });
    }
}
