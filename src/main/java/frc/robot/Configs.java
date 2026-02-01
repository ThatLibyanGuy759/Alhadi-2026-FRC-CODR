package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.Constants.ShooterConstants;

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
}
