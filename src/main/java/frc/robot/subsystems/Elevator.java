package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private SparkMax sparkMax1, sparkMax2;
    private DigitalInput forwardLimitSwitch, reverseLimitSwitch;

    public Elevator() {
        // Orignal IDs:
        // SparkMax1: 15
        // SparkMax2: 16
        sparkMax1 = new SparkMax(18, MotorType.kBrushless);
        sparkMax2 = new SparkMax(16, MotorType.kBrushless);
        forwardLimitSwitch = new DigitalInput(9);
        reverseLimitSwitch = new DigitalInput(8);
    }

    public void up(double speed) {
        if (forwardLimitSwitch.get()) {
            sparkMax1.set(speed);
            sparkMax2.set(speed);
        } else {
            sparkMax1.set(0.0);
            sparkMax2.set(0.0);
        }
    }

    public void down(double speed) {
        if (reverseLimitSwitch.get()) {
            sparkMax1.set(-speed);
            sparkMax2.set(-speed);
        } else {
            sparkMax1.set(0.0);
            sparkMax2.set(0.0);
        }
    }

    public void off() {
        sparkMax1.set(0.0);
        sparkMax2.set(0.0);
    }

    public Command elevatorCommand(BooleanSupplier up, BooleanSupplier down) {
        return run(() -> {
            if (!(down.getAsBoolean() || up.getAsBoolean())) {
                off();
            } else if (down.getAsBoolean()) {
                down(1.0);
            } else {
                up(1.0);
            }
        });
    }
}
