package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ImproperInitException;

public class Coral extends SubsystemBase {
    private SparkMax wheelMotor;
    private SparkMax turnMotor;
    private double[] positions = {}; // Count forward in "rotations" from back stop
    private final ArmFeedforward m_shooterFeedforward = new ArmFeedforward(Constants.Coral.kS, Constants.Coral.kG, Constants.Coral.kV, Constants.Coral.kA);
    private final PIDController m_shooterFeedback = new PIDController(0.1, 0.0, 0.0);
    private RelativeEncoder pivotEncoder;
    private DigitalInput backLimitSwitch, forwardLimitSwitch;

    public Coral() {
        wheelMotor = new SparkMax(17, MotorType.kBrushless);
        turnMotor = new SparkMax(18, MotorType.kBrushless);
        pivotEncoder = turnMotor.getAlternateEncoder();
        backLimitSwitch = new DigitalInput(7);
        forwardLimitSwitch = new DigitalInput(6);
        if (backLimitSwitch.get()) {
            pivotEncoder.setPosition(0);
        } else {
            throw new ImproperInitException();
        }
    }

    public void coralOut() {
        wheelMotor.set(1);
    }

    public void coralIn() {
        wheelMotor.set(-1);
    }

    public void coralOff() {
        wheelMotor.set(0);
    }

    public void pivotOff() {
        turnMotor.set(0.0);
    }

    public Command coralPivotCommand(DoubleSupplier angle) {
        return run(() -> {
            double targetAngle = (angle.getAsDouble() * 360)%360;
            // Keeps input to PID controller within bounds
            if (targetAngle < 0) {
                targetAngle = 0;
            } else if (targetAngle > Constants.Coral.MAX_ANGLE) {
                targetAngle = Constants.Coral.MAX_ANGLE;
            }

            turnMotor.setVoltage(m_shooterFeedback.calculate(pivotEncoder.getPosition(), targetAngle) +);
        });
    }

}
