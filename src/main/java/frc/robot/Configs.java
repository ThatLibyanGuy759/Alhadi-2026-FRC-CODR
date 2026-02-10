package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;
import frc.robot.wrist.WristConstants;

public final class Configs {


    private Configs() {}

  public static final class ShooterConfigs {


        public static TalonFXConfiguration shooterMotorConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();
       
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;

        
        config.Slot0.kS = 0.25;
        config.Slot0.kV = 0.12; 
        config.Slot0.kA = 0.01;

        config.Feedback.SensorToMechanismRatio = 6.0;

            config.Feedback.SensorToMechanismRatio = 6.0;
           
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;

            config.CurrentLimits.StatorCurrentLimit = 100;
            config.CurrentLimits.StatorCurrentLimitEnable = true;

            config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -100;


            return config;
        }
    }

    public static final class WristConfigs {
        public static final TalonFXConfiguration wristConfig = new TalonFXConfiguration();
        
        // Gear ratio: motor rotations to mechanism rotations
        // TODO: Set actual gear ratio for wrist 
        public static final double WRIST_GEAR_RATIO = 1.0;
        
        static {
            // Motor direction - original REV config had inverted(true)
            wristConfig. MotorOutput.Inverted = InvertedValue. Clockwise_Positive;

            // Neutral/Idle mode - original had IdleMode.kBrake
            wristConfig. MotorOutput.NeutralMode = NeutralModeValue.Brake;

            // Current limits - original had smartCurrentLimit(20)
            wristConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            wristConfig. CurrentLimits. SupplyCurrentLimit = WristConstants.supplyCurrentLimit;

            // Stator current limit for additional protection
            wristConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            wristConfig.CurrentLimits.StatorCurrentLimit = WristConstants.statorCurrentLimit;

            // Feedback configuration
            // Phoenix 6 uses rotations as the native unit
            // We apply gear ratio so 1 motor rotation = (1/GEAR_RATIO) mechanism rotations
            wristConfig.Feedback. SensorToMechanismRatio = WRIST_GEAR_RATIO;

            // Slot 0 PID gains for position control
            wristConfig.Slot0.kP = WristConstants.kP;
            wristConfig.Slot0.kI = WristConstants.kI;
            wristConfig.Slot0.kD = WristConstants.kD;
            wristConfig.Slot0.kV = WristConstants. kV;
            wristConfig. Slot0.kS = WristConstants. kS;

            // Soft limits in mechanism rotations (degrees converted to rotations)
            wristConfig. SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            wristConfig.SoftwareLimitSwitch. ForwardSoftLimitThreshold = WristConstants.maxPosition / 360.0;
            wristConfig. SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            wristConfig. SoftwareLimitSwitch.ReverseSoftLimitThreshold = WristConstants.minPositon / 360.0;

            // Closed-loop ramp rate (optional, smooth motion)
            wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;
        }
    }
}
