package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    // --- HARDWARE ---
    private final TalonFX turretMotor;
    private final String subsystemName; // e.g., "LeftTurret" or "RightTurret" for debugging

    // --- CONTROL REQUEST ---
    // Motion Magic Voltage is the Gold Standard for Krakens (compensates for battery sag)
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

    // --- CONSTANTS ---
    // 15:1 Gear Ratio (Example) -> 15 motor rotations = 1 turret rotation
    private static final double GEAR_RATIO = 15.0; 
    
    // Soft Limits: +/- 180 degrees = +/- 0.5 rotations.
    // We use +/- 0.48 (approx 173 degrees) to leave a safety buffer so we don't hit hard stops.
    private static final double SOFT_LIMIT_FWD_ROT = 0.48;
    private static final double SOFT_LIMIT_REV_ROT = -0.48;

    // Tolerance to consider the turret "at target" (in degrees)
    private static final double AIM_TOLERANCE_DEG = 2.0;

    /**
     * Constructor
     * @param canId The CAN ID of the Kraken X60
     * @param name Name for SmartDashboard/Logging (e.g., "Left", "Right")
     */
    public TurretSubsystem(int canId, String name) {
        this.subsystemName = name;
        
        // "rio" is the CAN bus name. Change to "canivore" if you use one.
        this.turretMotor = new TalonFX(canId, "rio"); 

        configureTurret();
    }

    private void configureTurret() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // 1. FEEDBACK & GEARING
        // Setting this allows us to work in "Mechanism Rotations" (1.0 = 1 full turret spin)
        cfg.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        // 2. SOFT LIMITS (Critical for +/- 180 degree cables)
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = SOFT_LIMIT_FWD_ROT;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = SOFT_LIMIT_REV_ROT;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // 3. MOTION MAGIC SETTINGS (The "Snappy" factor)
        // Tune these! Start slow, then increase.
        cfg.MotionMagic.MotionMagicCruiseVelocity = 2.5; // Rotations per second (Very fast)
        cfg.MotionMagic.MotionMagicAcceleration = 5.0;   // Rotations per sec^2 (Snap!)
        cfg.MotionMagic.MotionMagicJerk = 0.0;           // 0 = Infinite jerk (Fastest response)

        // 4. PID GAINS (Must be tuned via SysId!)
        // Placeholders for a Kraken X60
        cfg.Slot0.kP = 40.0; // Stiff P term
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.5;
        cfg.Slot0.kV = 0.12; // Velocity Feedforward
        cfg.Slot0.kS = 0.25; // Static Friction Feedforward

        // 5. CURRENT LIMITS (Safety)
        cfg.CurrentLimits.StatorCurrentLimit = 60.0; // Amps
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply configuration
        turretMotor.getConfigurator().apply(cfg);
        
        // Ensure motor brakes when idle so the turret doesn't flop around
        turretMotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Reset position on boot (Assumption: Turret starts facing FORWARD/CENTERED)
        // Ideally, use an absolute encoder or limit switch to seed this properly.
        turretMotor.setPosition(0);
    }

    /**
     * Main Control Method
     * @param targetAngle Robot-Relative angle (0 is front, +/- 180 is back)
     */
    public void setTargetAngle(Rotation2d targetAngle) {
        // 1. Get Rotations (-0.5 to 0.5)
        double targetRotations = targetAngle.getRotations();

        // 2. Normalize to Range (-0.5 to 0.5)
        // This ensures that if we ask for 181 degrees, it wraps to -179.
        targetRotations = MathUtil.inputModulus(targetRotations, -0.5, 0.5);

        // 3. Clamp to Soft Limits
        // If the target is in the "dead zone" (back of robot where cables wrap),
        // we clamp to the nearest safe limit.
        targetRotations = MathUtil.clamp(targetRotations, SOFT_LIMIT_REV_ROT, SOFT_LIMIT_FWD_ROT);

        // 4. Command the Motor
        turretMotor.setControl(mmRequest.withPosition(targetRotations));
    }

    /**
     * @return The current actual angle of the turret (Robot Relative)
     */
    public Rotation2d getCurrentAngle() {
        double rotations = turretMotor.getPosition().getValueAsDouble();
        return Rotation2d.fromRotations(rotations);
    }

    /**
     * @return Error in degrees (Difference between Target and Actual)
     * Used by Superstructure to check if we are "Locked On"
     */
    public double getErrorDegrees() {
        // Get closed loop error from the motor controller (in Rotations)
        double errorRotations = turretMotor.getClosedLoopError().getValueAsDouble();
        return Rotation2d.fromRotations(errorRotations).getDegrees();
    }
    
    /**
     * Helper to zero the turret manually or via a limit switch
     */
    public void resetPosition() {
        turretMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // Logging for Driver Station
        SmartDashboard.putNumber(subsystemName + "/AngleDeg", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber(subsystemName + "/ErrorDeg", getErrorDegrees());
    }
}