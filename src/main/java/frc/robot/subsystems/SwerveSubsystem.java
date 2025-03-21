package frc.robot.subsystems;

import java.io.File;
import java.io.FileReader;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    File swerveJsonDirectory;
    SwerveDrive swerveDrive;
    PPHolonomicDriveController pathFollowerConfig;
    RobotConfig robotConfig;

    public SwerveSubsystem() {
        swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        JSONObject pidfDriveConfig, pidfAngleConfig;

        try {
            // Create Swerve Drive
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(Constants.Swerve.MAX_SPEED);

            // Get nessecary configs for AutoBuilder configuration
        
            JSONParser parser = new JSONParser();
            JSONObject pidfConfigFile = (JSONObject) parser
                    .parse(new FileReader(new File(swerveJsonDirectory, "modules/pidfproperties.json")));
            pidfDriveConfig = (JSONObject) pidfConfigFile.get("drive");
            pidfAngleConfig = (JSONObject) pidfConfigFile.get("angle");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        pathFollowerConfig = new PPHolonomicDriveController(
            new PIDConstants( // translation constants
                (Double) pidfDriveConfig.get("p"),
                (Double) pidfDriveConfig.get("i"),
                (Double) pidfDriveConfig.get("d"),
                (Double) pidfDriveConfig.get("iz")
            ),
            new PIDConstants( // rotation constants
                (Double) pidfAngleConfig.get("p"),
                (Double) pidfAngleConfig.get("i"),
                (Double) pidfAngleConfig.get("d"),
                (Double) pidfAngleConfig.get("iz")
            )
        );

        Translation2d[] moduleOffsets = {
            new Translation2d(-Constants.Swerve.HORIZONTAL_MODULE_DISTANCE, Constants.Swerve.VERTICAL_MODULE_DISTANCE), // FL
            new Translation2d(Constants.Swerve.HORIZONTAL_MODULE_DISTANCE, Constants.Swerve.VERTICAL_MODULE_DISTANCE), // FR
            new Translation2d(-Constants.Swerve.HORIZONTAL_MODULE_DISTANCE, -Constants.Swerve.VERTICAL_MODULE_DISTANCE), // BL
            new Translation2d(Constants.Swerve.HORIZONTAL_MODULE_DISTANCE, -Constants.Swerve.VERTICAL_MODULE_DISTANCE), // BR
        };

        ModuleConfig moduleConfig = new ModuleConfig(
            Constants.Swerve.WHEEL_RADIUS, 
            Constants.Swerve.REAL_MAX_SPEED, 
            Constants.Swerve.WHEEL_COF, 
            new DCMotor(12, 3.6, 211, 3.6, 6784*2*Math.PI, 1).withReduction(Constants.Swerve.DRIVE_RATIO), 
            Constants.Swerve.DRIVE_CURRENT_LIM, 
            1
        );

        robotConfig = new RobotConfig(Constants.Robot.MASS, Constants.Robot.MOMENT_OF_INERTIA, moduleConfig, moduleOffsets);
        // These two final lines are only needed for simulation purposes, they are
        // configured based on control method of a real bot
        if (!Robot.isReal()) {
            swerveDrive.setHeadingCorrection(false);
            swerveDrive.setCosineCompensator(false);
        }

        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds,
                (speeds) -> driveRobotRelative(speeds),
                pathFollowerConfig,
                robotConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
        );
    }

    /**
     * Command to drive the robot using translative values and heading as a
     * setpoint.
     *
     * @param translationX Translation in the X direction.
     * @param translationY Translation in the Y direction.
     * @param headingX     Heading X to calculate angle of the joystick.
     * @param headingY     Heading Y to calculate angle of the joystick.
     * @return Drive command.
     */
    @Deprecated
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 1);

            // Make the robot move
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumModuleDriveVelocity()));
        });
    }

    @Deprecated
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
                    translationX.getAsDouble(),
                    translationY.getAsDouble(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumModuleDriveVelocity()));
        });
    }

    /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction. Cubed for smoother controls.
   * @param translationY     Translation in the Y direction. Cubed for smoother controls.
   * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      provideVisionPos();
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                            translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                            translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                        true,
                        false);
    });
  }

  public Command simpleAutoCommand() {
    return new InstantCommand(() -> {
        swerveDrive.drive(new ChassisSpeeds(-3, 0, 0));
    },
    this);
  }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public void driveRobotRelative(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    public Pose2d getPose() {
        provideVisionPos();
        return swerveDrive.getPose();
    }

    public void resetPose(Pose2d pose) {
        if (Robot.isReal()) {
            swerveDrive.resetOdometry(pose);
        }
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public void provideVisionPos() {
        if (LimelightHelpers.getTargetCount(Constants.Limelight.NAME) >= 2) {
            swerveDrive.addVisionMeasurement(LimelightHelpers.getBotPose2d(Constants.Limelight.NAME),
                    Timer.getFPGATimestamp());
        }
    }
}
