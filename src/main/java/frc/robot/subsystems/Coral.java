package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ImproperInitException;

public class Coral extends SubsystemBase {
    private SparkMax wheelMotor;
    private SparkMax turnMotor;

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(45),
                    Units.degreesToRadians(90))); // Max arm speed and acceleration.
    private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();

    private final LinearSystem<N2, N1, N2> m_armPlant = LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeoVortex(1),
            Constants.Coral.kArmMOI, Constants.Coral.kArmGearing);

    
    @SuppressWarnings("unchecked")
    private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            (LinearSystem<N2, N1, N1>) m_armPlant.slice(0),
            VecBuilder.fill(0.015, 0.17), // How accurate we
            // think our model is, in radians and radians/sec
            VecBuilder.fill(0.01), // How accurate we think our encoder position
            // data is. In this case we very highly trust our encoder position reading.
            0.020);

    // A LQR uses feedback to create voltage commands.
    @SuppressWarnings("unchecked")
    private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
            (LinearSystem<N2, N1, N1>) m_armPlant.slice(0),
            VecBuilder.fill(Units.degreesToRadians(1.0), Units.degreesToRadians(10.0)), // qelms.
            // Position and velocity error tolerances, in radians and radians per second.
            // Decrease
            // this
            // to more heavily penalize state excursion, or make the controller behave more
            // aggressively. In this example we weight position much more highly than
            // velocity, but
            // this
            // can be tuned to balance the two.
            VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12
            // is a good
            // starting point because that is the (approximate) maximum voltage of a
            // battery
            0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be

    // The state-space loop combines a controller, observer, feedforward and plant
    // for easy control.
    @SuppressWarnings("unchecked")
    private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
            (LinearSystem<N2, N1, N1>) m_armPlant.slice(0), m_controller, m_observer, 12.0, 0.020);

    private DutyCycleEncoder pivotEncoder;
    private Encoder m_encoder;
    private DigitalInput backLimitSwitch, forwardLimitSwitch;

    public Coral() {
        wheelMotor = new SparkMax(18, MotorType.kBrushless);

        turnMotor = new SparkMax(17, MotorType.kBrushless);
        m_encoder = new Encoder(1, 2, false);

        backLimitSwitch = new DigitalInput(7);
        forwardLimitSwitch = new DigitalInput(6);

        // Reset our loop to make sure it's in a known state.
        m_loop.reset(VecBuilder.fill(m_encoder.getDistance(), m_encoder.getRate()));

        // Reset our last reference to the current state.
        m_lastProfiledReference = new TrapezoidProfile.State(m_encoder.getDistance(), m_encoder.getRate());
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
            TrapezoidProfile.State goal = new TrapezoidProfile.State(Constants.Coral.kRaisedPosition, 0.0);

            // Step our TrapezoidalProfile forward 20ms and set it as our next reference
            m_lastProfiledReference = m_profile.calculate(0.020, m_lastProfiledReference, goal);
            m_loop.setNextR(m_lastProfiledReference.position, m_lastProfiledReference.velocity);
            // Correct our Kalman filter's state vector estimate with encoder data.
            m_loop.correct(VecBuilder.fill(m_encoder.getDistance()));

            // Update our LQR to generate new voltage commands and use the voltages to
            // predict the next
            // state with out Kalman filter.
            m_loop.predict(0.020);

            // Send the new calculated voltage to the motors.
            // voltage = duty cycle * battery voltage, so
            // duty cycle = voltage / battery voltage
            double nextVoltage = m_loop.getU(0);
            turnMotor.setVoltage(nextVoltage);
        });
    }

}
