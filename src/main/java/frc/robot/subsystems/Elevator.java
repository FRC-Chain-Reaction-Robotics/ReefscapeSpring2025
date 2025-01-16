package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Elevator {
    private SparkMax sparkMax1, sparkMax2;
    public Elevator()
    {
        sparkMax1 = new SparkMax(15, MotorType.kBrushless);
        sparkMax2 = new SparkMax(16, MotorType.kBrushless);
        
    }


    
}
