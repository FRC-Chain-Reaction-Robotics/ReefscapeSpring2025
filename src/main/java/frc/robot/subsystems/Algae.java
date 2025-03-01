package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class Algae {
     private SparkMax intakeMotor;
     private SparkMax angleMotor;

    public Algae() {
        intakeMotor = new SparkMax(x, MotorType.kBrushless);
        angleMotor = new SparkMax(x, MotorType.kBrushless);

    }

    public void off() {
        intakeMotor.set(0.0);
        angleMotor.set(0.0);
    }

    public void intake(double speed) {
        intakeMotor.set(-speed);

    }

    public void outtake(double speed) {
        intakeMotor.set(1);
    }

    public void rotateup(double speed) {
        angleMotor.set(1);
    }

    public void rotatedown(double speed) {
        angleMotor.set(-speed);
    }




}
