package frc.robot;

import java.util.Objects;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Publishes swerve drivetrain state to SmartDashboard/NetworkTables and the on-disk signal log. */
public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     *
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("Robot").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    // Below these speeds, we call it "stopped" for logging - ignores small control-loop noise.
    private static final double kTranslationLogThresholdMps = 0.1;
    // Raised from 0.1 - was logging "Rotating..." on tiny noise, not real movement.
    private static final double kRotationLogThresholdRadPerSec = 0.3;

    // null means "stopped". Tracked so logMotionChanges() only prints on a change, not every loop.
    private String lastTranslationLabel = null;
    private String lastRotationLabel = null;

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        logMotionChanges(state.Speeds);

        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
        SignalLogger.writeInteger("DriveState/FailedDaqs", state.FailedDaqs);

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }

    /**
     * Logs "Going &lt;direction&gt; at velocity &lt;x&gt;" / "Rotating clockwise" /
     * "counterclockwise" / "Stopped" to the console, but only when it changes.
     */
    private void logMotionChanges(ChassisSpeeds speeds) {
        String translationLabel = translationLabel(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        String rotationLabel = rotationLabel(speeds.omegaRadiansPerSecond);
        boolean wasMoving = lastTranslationLabel != null || lastRotationLabel != null;
        boolean isMoving = translationLabel != null || rotationLabel != null;

        if (!Objects.equals(translationLabel, lastTranslationLabel) && translationLabel != null) {
            double speed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
            RobotLog.log(String.format("Going %s at velocity %.2f", translationLabel, speed));
        }
        if (!Objects.equals(rotationLabel, lastRotationLabel) && rotationLabel != null) {
            RobotLog.log("Rotating " + rotationLabel);
        }
        if (wasMoving && !isMoving) {
            RobotLog.log("Stopped");
        }

        lastTranslationLabel = translationLabel;
        lastRotationLabel = rotationLabel;
    }

    /** Null (below threshold) means "stopped"; otherwise the dominant axis's direction. */
    private static String translationLabel(double vxMetersPerSecond, double vyMetersPerSecond) {
        if (Math.hypot(vxMetersPerSecond, vyMetersPerSecond) < kTranslationLogThresholdMps) {
            return null;
        }
        // Dominant axis determines the label - a perfectly diagonal command is rare in practice
        // and this is a human-readable log line, not a precise measurement.
        // Confirmed backwards on the robot vs. WPILib's nominal +X-is-forward convention - flipped.
        return Math.abs(vxMetersPerSecond) >= Math.abs(vyMetersPerSecond)
            ? (vxMetersPerSecond > 0 ? "backward" : "forward")
            : (vyMetersPerSecond > 0 ? "left" : "right");
    }

    /** Null (below threshold) means "stopped"; otherwise "clockwise"/"counterclockwise". */
    private static String rotationLabel(double omegaRadiansPerSecond) {
        if (Math.abs(omegaRadiansPerSecond) < kRotationLogThresholdRadPerSec) {
            return null;
        }
        // WPILib convention: positive omega is counter-clockwise.
        return omegaRadiansPerSecond > 0 ? "counterclockwise" : "clockwise";
    }
}
