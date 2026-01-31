// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;

import javax.xml.validation.SchemaFactoryConfigurationError;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import frc.robot.Configs;
import frc.robot.Configs.ShooterConfigs;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import com.ctre.phoenix6.controls.VoltageOut;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorOne;
    private final TalonFX shooterMotorTwo;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0); 
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;

    /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    shooterMotorOne = new TalonFX(ShooterConstants.shooterMotorOneID);
    shooterMotorTwo = new TalonFX(ShooterConstants.shooterMotorTwoID);
      m_SysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(null,
      Volts.of(4),
      null, 
      (state) -> SignalLogger.writeString("state", state.toString())),
      new SysIdRoutine.Mechanism(
        (volts) -> shooterMotorOne.setControl(sysIdControl.withOutput(volts.in(Volts))),
        log -> {
          log.motor("Shooter Motor")
          .voltage(shooterMotorOne.getMotorVoltage().getValue())
          .angularPosition(shooterMotorOne.getPosition().getValue())
          .angularVelocity(shooterMotorOne.getVelocity().getValue());
        },
      this));
    configureMotors();
    SignalLogger.setPath("/home/vuser/logs/");
    
  }

  public void configureMotors() {
    shooterMotorOne.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());
    shooterMotorTwo.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());

    shooterMotorOne.getVelocity().setUpdateFrequency(100);
    shooterMotorOne.getPosition().setUpdateFrequency(100);

  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.dynamic(direction);
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_SysIdRoutine.quasistatic(direction);
  }

  public void shootFuel(double speed) {
    shooterMotorOne.setControl(dutyCycle.withOutput(speed));
  }

  public void runVelocity(double rps) {
    voltageRequest.Velocity = rps;
    shooterMotorOne.setControl(voltageRequest); 
  }
  
public void runVelocityTorqueFOC(double rps) { 
  shooterMotorOne.setControl(velocityTorqueRequest.withVelocity(rps));
  shooterMotorTwo.setControl(velocityTorqueRequest.withVelocity(rps));

  // Logging for verification
  SmartDashboard.putNumber("Motor Target RPS", rps);
  SmartDashboard.putNumber("Motor1 Actual RPS", shooterMotorOne.getVelocity().getValueAsDouble());
  SmartDashboard.putNumber("Motor2 Actual RPS", shooterMotorTwo.getVelocity().getValueAsDouble());
}



  

  public void printVoltageOutput() {
    double motorVoltage = shooterMotorOne.getMotorVoltage().getValueAsDouble();
    SmartDashboard.putNumber("Motor Voltage", motorVoltage);
  }

  public void resetVoltageOutput() {
    SmartDashboard.putNumber("Motor Voltage", 0);
  }

  public void printCurrentLimits() {
    SmartDashboard.putNumber("Shooter Stator Current", shooterMotorOne.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Supply Current", shooterMotorOne.getSupplyCurrent().getValueAsDouble());
  }

  public void printRPM() {
    double motorRPS = shooterMotorOne.getVelocity().getValueAsDouble();
    double shooterRPM = motorRPS * 60.0;
    SmartDashboard.putNumber("Shooter Motor RPM", shooterRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}