package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Configs;


public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotorOne;
  private final TalonFX shooterMotorTwo;
  private final TalonFX kickerMotorOne;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public ShooterSubsystem() {
    shooterMotorOne = new TalonFX(ShooterConstants.shooterMotorOneID);
    shooterMotorTwo = new TalonFX(ShooterConstants.shooterMotorTwoID);
    kickerMotorOne = new TalonFX(ShooterConstants.kickerMotorID);

    configureMotors();
    }

  private void configureMotors() {
    shooterMotorOne.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());
    shooterMotorTwo.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());

    kickerMotorOne.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());


  }


    public void shootFuel(double speed) {
      shooterMotorOne.setControl(dutyCycle.withOutput(speed));
      shooterMotorTwo.setControl(dutyCycle.withOutput(speed));
      kickerMotorOne.setControl(dutyCycle.withOutput(speed));
    }


    public void runVelocity(double rps) {
      velocityRequest.Velocity = rps;
      shooterMotorOne.setControl(velocityRequest);
      shooterMotorTwo.setControl(velocityRequest);
      kickerMotorOne.setControl(velocityRequest);
    }

  
}
