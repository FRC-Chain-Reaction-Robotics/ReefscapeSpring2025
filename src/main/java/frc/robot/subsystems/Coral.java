package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ImproperInitException;

public class Coral extends SubsystemBase {
    private SparkMax wheelMotor;
    private SparkMax turnMotor;
    private double[] positions = {}; // Count forward in "rotations" from back stop
    private final ArmFeedforward m_shooterFeedforward = new ArmFeedforward(Constants.Coral.kS, Constants.Coral.kG,
            Constants.Coral.kV);
    private final ProfiledPIDController m_shooterFeedback = new ProfiledPIDController(0.1, 0.0, 0.1, 
    new TrapezoidProfile.Constraints(Constants.Coral.MAX_V, Constants.Coral.MAX_A));
    private DutyCycleEncoder pivotEncoder;
    private DigitalInput backLimitSwitch, forwardLimitSwitch;
    private DoublePublisher pub;

    public Coral() {
        wheelMotor = new SparkMax(18, MotorType.kBrushless);
        
        turnMotor = new SparkMax(17, MotorType.kBrushless);
        pivotEncoder = new DutyCycleEncoder(5);
        m_shooterFeedback.enableContinuousInput(0.0, 1.0);

        backLimitSwitch = new DigitalInput(7);
        forwardLimitSwitch = new DigitalInput(6);
        
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
            double targetAngle = 0.5;
            // Keeps input to PID controller within bounds
            // if (targetAngle < 0) {
            //     targetAngle = 0;
            // } else if (targetAngle > Constants.Coral.MAX_ANGLE) {
            //     targetAngle = Constants.Coral.MAX_ANGLE;
            // }
            // System.out.println("AAAAAAAAAAAAA: " + pivotEncoder.get());
            // System.out.println("ABBBBBBBBA: " + targetAngle);
            // System.out.println("ABCCCCCBA: " + (m_shooterFeedback.calculate(pivotEncoder.get(), targetAngle)        
            // + m_shooterFeedforward.calculate(m_shooterFeedback.getSetpoint().position + Constants.Coral.ffAngleOffset, 
            // m_shooterFeedback.getSetpoint().velocity)));

            turnMotor.setVoltage(m_shooterFeedback.calculate(pivotEncoder.get(), targetAngle)        
            + m_shooterFeedforward.calculate(m_shooterFeedback.getSetpoint().position + Constants.Coral.ffAngleOffset, 
             m_shooterFeedback.getSetpoint().velocity));
        });
    }

}
