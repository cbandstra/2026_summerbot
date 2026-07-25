package frc.robot.generated;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Swerve constants for 4x SDS Mk5n modules (Kraken X60 drive, Kraken X44 steer, CTRE CANcoder),
 * on the roboRIO's onboard CAN bus, with a CTRE Pigeon 2 for heading.
 *
 * <p>CAN IDs, gear ratios, coupling ratio, inversions, speed-at-12V, and CANcoder magnet offsets
 * below are copied from team 4296's tridentrobotics/2026_official repo - per the team, this is
 * the same physical chassis/modules (SDS Mk5n / Kraken X60 / Kraken X44 / CANcoder), just under a
 * new project, and the CAN ID convention (steer 0-3, drive 4-7, encoders 11-14, Pigeon 20 — see
 * that repo's {@code Constants.CanIDs}) carries over unchanged. Physical module positions here
 * use this project's own 19.75in x 19.75in wheelbase/trackwidth - the X/Y offsets in that other
 * repo implying a 23in x 23.5in frame were a measurement error there, not a real difference.
 *
 * <p><b>Before this will drive correctly on the real robot, the team should still:</b>
 *
 * <ol>
 *   <li>Confirm the CAN IDs below actually match how this chassis's devices are currently
 *       assigned in Phoenix Tuner X, in case wiring changed since 2026_official was written.
 *   <li>Spot-check each CANcoder's {@code kXxxEncoderOffset} still holds: with each module
 *       manually pointed straight forward (wheel bevel gear facing the robot's front), read the
 *       CANcoder's absolute position in Tuner X and confirm it reads ~0. If a module was removed/
 *       reseated since 2026_official was tuned, its magnet offset may have shifted and need
 *       re-zeroing. See https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
 *   <li>Verify {@code kXxxSteerMotorInverted} / {@code kInvertLeftSide}/{@code kInvertRightSide}:
 *       drive the robot forward slowly and confirm all four wheels drive forward (not spinning in
 *       place), then rotate each module by hand and confirm the CANcoder position increases when
 *       turning counterclockwise (viewed from above).
 *   <li>Tune {@code steerGains}/{@code driveGains} and {@code kSlipCurrent} for this specific
 *       robot using the SysId routines wired into {@code CommandSwerveDrivetrain} (see
 *       RobotContainer bindings) — the gains below are still generic starting points, not
 *       measured results, even though the other constants now come from this same robot.
 * </ol>
 */
public class TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    //
    // kP/kD below are weakened from the Tuner X template defaults (kP=100, kD=0.5). Those
    // defaults are tuned for FusedCANcoder running its closed loop at 1 kHz (Phoenix Pro only);
    // this project uses RemoteCANcoder (no Pro license - see kSteerFeedbackType below), whose
    // position feedback only updates over CAN at ~100 Hz, so the same kP is much more aggressive
    // relative to the loop's actual rate and can cause the steer motors to oscillate/chatter
    // ("gears sound loud" when it's actually the steer motor hunting, not mechanical noise). See
    // https://www.chiefdelphi.com/t/phoenix-6-generated-swerve-default-steer-gains-oscillating/483291
    // These values are a conservative starting point, not a final tuned result - if modules still
    // chatter, lower kP further; if steering feels sluggish/spongy, raise kP back up in small
    // steps once kD is damping well. A proper SysId steer characterization (already wired into
    // CommandSwerveDrivetrain) will get more precise numbers than manual tuning.
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(8).withKI(0).withKD(0.01)
        .withKS(0.1).withKV(1.91).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // Kraken X60 (drive) and Kraken X44 (steer) both use the integrated TalonFX controller
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // RemoteCANcoder does not require a Phoenix Pro license, unlike Fused*/Sync* CANcoder.
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Default supply current limit is 70 A, but it can be lowered to avoid brownouts.
                // Supply current limits can be larger than the breaker current rating.
                .withSupplyCurrentLimit(Amps.of(70))
                .withSupplyCurrentLimitEnable(true)
        );
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                // Swerve azimuth does not require much torque output, so we can set a relatively low
                // stator current limit to help avoid brownouts without impacting performance.
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

    // CAN bus that the devices are located on; roboRIO onboard bus (no CANivore)
    public static final CANBus kCANBus = CANBus.roboRIO();

    // Measured robot speed (m/s) at 12 V applied output;
    // This is NOT the desired max robot speed - see MaxSpeed in RobotContainer instead;
    // Value taken from team 4296's tridentrobotics/2026_official repo, which drives the same
    // SDS Mk5n + Kraken X60/X44 module hardware - re-verify empirically on this chassis.
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.85);

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // Value taken from team 4296's tridentrobotics/2026_official repo (same Mk5n module hardware).
    private static final double kCoupleRatio = 3.375;

    // Mk5n drive reduction. Value taken from team 4296's tridentrobotics/2026_official repo
    // (same Mk5n module hardware) - confirm it matches this chassis's pinion/gear selection.
    private static final double kDriveGearRatio = 5.2734375;
    // Mk5n steering reduction (287:11), confirmed both by SDS's published spec and by team
    // 4296's tridentrobotics/2026_official repo.
    private static final double kSteerGearRatio = 287.0 / 11.0;
    private static final Distance kWheelRadius = Inches.of(2.0); // 4 in wheel diameter

    // Inversions taken from team 4296's tridentrobotics/2026_official repo (same module hardware).
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    // Matches team 4296's tridentrobotics/2026_official CAN ID convention (Constants.CanIDs.Pigeon).
    private static final int kPigeonId = 20;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.035);
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // CAN IDs and CANcoder magnet offsets match team 4296's tridentrobotics/2026_official repo
    // (see Constants.CanIDs there: steer motors 0-3, drive motors 4-7, CANcoders 11-14, Pigeon
    // 20) - per the team this is the same physical chassis/modules, so the offsets carry over.

    // Front Left
    private static final int kFrontLeftDriveMotorId = 4;
    private static final int kFrontLeftSteerMotorId = 0;
    private static final int kFrontLeftEncoderId = 14;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.3642578125);
    private static final boolean kFrontLeftSteerMotorInverted = false;
    private static final boolean kFrontLeftEncoderInverted = false;

    private static final Distance kFrontLeftXPos = Inches.of(9.875);
    private static final Distance kFrontLeftYPos = Inches.of(9.875);

    // Front Right
    private static final int kFrontRightDriveMotorId = 6;
    private static final int kFrontRightSteerMotorId = 3;
    private static final int kFrontRightEncoderId = 13;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.018310546875);
    private static final boolean kFrontRightSteerMotorInverted = false;
    private static final boolean kFrontRightEncoderInverted = false;

    private static final Distance kFrontRightXPos = Inches.of(9.875);
    private static final Distance kFrontRightYPos = Inches.of(-9.875);

    // Back Left
    private static final int kBackLeftDriveMotorId = 5;
    private static final int kBackLeftSteerMotorId = 1;
    private static final int kBackLeftEncoderId = 12;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.300537109375);
    private static final boolean kBackLeftSteerMotorInverted = false;
    private static final boolean kBackLeftEncoderInverted = false;

    private static final Distance kBackLeftXPos = Inches.of(-9.875);
    private static final Distance kBackLeftYPos = Inches.of(9.875);

    // Back Right
    private static final int kBackRightDriveMotorId = 7;
    private static final int kBackRightSteerMotorId = 2;
    private static final int kBackRightEncoderId = 11;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.035888671875);
    private static final boolean kBackRightSteerMotorInverted = false;
    private static final boolean kBackRightEncoderInverted = false;

    private static final Distance kBackRightXPos = Inches.of(-9.875);
    private static final Distance kBackRightYPos = Inches.of(-9.875);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    /**
     * Creates a CommandSwerveDrivetrain instance.
     * This should only be called once in your robot program.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight
        );
    }

    /**
     * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types.
     */
    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
         * @param modules               Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency The frequency to run the odometry loop. If
         *                                unspecified or set to 0 Hz, this is 250 Hz on
         *                                CAN FD, and 100 Hz on CAN 2.0.
         * @param modules                 Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        /**
         * Constructs a CTRE SwerveDrivetrain using the specified constants.
         * <p>
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them through
         * getters in the classes.
         *
         * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
         * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
         *                                  unspecified or set to 0 Hz, this is 250 Hz on
         *                                  CAN FD, and 100 Hz on CAN 2.0.
         * @param odometryStandardDeviation The standard deviation for odometry calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param visionStandardDeviation   The standard deviation for vision calculation
         *                                  in the form [x, y, theta]ᵀ, with units in meters
         *                                  and radians
         * @param modules                   Constants for each specific module
         */
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}
