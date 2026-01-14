package frc.robot.subsystems;

// --- REV IMPORTS (New 2026 API) ---
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

// --- CTRE IMPORTS (Phoenix 6) ---
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.ShootingTables;

public class ShooterSubsystem extends SubsystemBase {

    // --- HARDWARE ---
    private final TalonFX topMotor;           // Kraken X60 (Phoenix 6)
    private final SparkFlex bottomMotor;      // Neo Vortex (REVLib 2026)

    // --- CONTROLLERS ---
    // New REV class for PID control
    private final SparkClosedLoopController bottomController;
    private final RelativeEncoder bottomEncoder;
    
    // Phoenix 6 Request Object
    private final VelocityVoltage topRequest = new VelocityVoltage(0);

    private final String name;

    // --- STATE ---
    private double targetTopRPM = 0;
    private double targetBottomRPM = 0;

    private static final double RPM_TOLERANCE_PERCENT = 0.03;

    public ShooterSubsystem(int topCanId, int bottomCanId, String name) {
        this.name = name;

        // 1. Initialize Kraken (Top)
        topMotor = new TalonFX(topCanId, "rio");
        configureKraken();

        // 2. Initialize Vortex (Bottom) - NEW SYNTAX
        // Note: Use SparkFlex class for Vortex
        bottomMotor = new SparkFlex(bottomCanId, MotorType.kBrushless);
        bottomController = bottomMotor.getClosedLoopController();
        bottomEncoder = bottomMotor.getEncoder();
        
        configureVortex();
    }

    // --- CONFIGURATION: KRAKEN (Phoenix 6) ---
    private void configureKraken() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.StatorCurrentLimit = 60.0;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        
        // PID (Tune kV via SysId!)
        cfg.Slot0.kP = 0.1; 
        cfg.Slot0.kV = 0.12; 

        topMotor.getConfigurator().apply(cfg);
        topMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    // --- CONFIGURATION: VORTEX (REVLib 2026) ---
    private void configureVortex() {
        // Create the config object
        SparkFlexConfig config = new SparkFlexConfig();

        // 1. Idle Mode & Current Limits
        config.idleMode(IdleMode.kCoast)
              .smartCurrentLimit(60);

        // 2. PIDF Configuration
        // Note: "closedLoop" property handles PID now
        config.closedLoop
              .p(0.0001)
              .i(0.0)
              .d(0.0)
              .velocityFF(0.00016) // New name for kFF in some versions, check docs!
              .outputRange(-1, 1);

        // 3. Encoder Config (if needed)
        // config.encoder.velocityConversionFactor(1.0);

        // 4. Apply Configuration
        // ResetSafeParameters ensures we start clean.
        // PersistParameters saves it to flash so it remembers after power cycle.
        bottomMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // --- CONTROL LOGIC ---

    public void setTargetDistance(double distanceMeters) {
        targetTopRPM = ShootingTables.topFlywheelMap.get(distanceMeters);
        targetBottomRPM = ShootingTables.bottomFlywheelMap.get(distanceMeters);

        if (targetBottomRPM > 100) {
            // --- BOTTOM (VORTEX) ---
            // New Method: setSetpoint() instead of setReference()
            bottomController.setSetpoint(targetBottomRPM, ControlType.kVelocity);

            // --- TOP (KRAKEN) ---
            // Convert RPM to RPS
            double targetRPS = targetTopRPM / 60.0;
            topMotor.setControl(topRequest.withVelocity(targetRPS));
            
        } else {
            stop();
        }
    }

    public void stop() {
        // REV: Stop by setting voltage to 0 is safest
        bottomMotor.stopMotor();
        // CTRE: Stop
        topMotor.stopMotor();
        
        targetTopRPM = 0;
        targetBottomRPM = 0;
    }

    public boolean isReadyToFire() {
        // REV: getVelocity() is still standard on the encoder object
        double botVelRPM = bottomEncoder.getVelocity();
        
        // CTRE: getVelocity() returns a StatusSignal, getValue() gets RPS
        double topVelRPM = topMotor.getVelocity().getValueAsDouble() * 60.0;

        double topError = Math.abs(targetTopRPM - topVelRPM);
        double botError = Math.abs(targetBottomRPM - botVelRPM);

        boolean topReady = topError < (targetTopRPM * RPM_TOLERANCE_PERCENT);
        boolean botReady = botError < (targetBottomRPM * RPM_TOLERANCE_PERCENT);
        
        return topReady && botReady && (targetBottomRPM > 100);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(name + "/TopRPM", topMotor.getVelocity().getValueAsDouble() * 60.0);
        SmartDashboard.putNumber(name + "/BotRPM", bottomEncoder.getVelocity());
        SmartDashboard.putBoolean(name + "/Ready", isReadyToFire());
    }
}