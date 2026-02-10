// Copyright (c) FIRST and other WPILib contributors. 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls. PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com. ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot. Configs;
import frc.robot.wrist.WristConstants;

public class WristSubsystem extends SubsystemBase {
  private final TalonFX wristMotor;

  // Control requests (reusable objects for performance)
  private final PositionVoltage positionRequest = new PositionVoltage(0). withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  public WristSubsystem() {
    wristMotor = new TalonFX(WristConstants.wristMotorCANId);

    // Apply configuration from Configs.WristConfigs
    StatusCode status = wristMotor.getConfigurator().apply(Configs.WristConfigs.wristConfig);
    if (! status.isOK()) {
      System.err.println("Failed to apply wrist motor config: " + status. toString());
    }

    // Set the motor position to 0 at startup (or seed from an absolute encoder if available)
    wristMotor.setPosition(0);
  }

  /**
   * Commands the wrist to rotate to a specified position.
   * 
   * @param positionDegrees The target position in degrees
   */
  public void wristRotateToPosition(double positionDegrees) {
    double targetRotations = degreesToRotations(positionDegrees);
    wristMotor.setControl(positionRequest.withPosition(targetRotations));
  }

  /**
   * Gets the current wrist angle in degrees. 
   * 
   * @return The wrist angle in degrees
   */
  public double getWristAngle() {
    double rotations = wristMotor.getPosition(). getValueAsDouble();
    return rotationsToDegrees(rotations);
  }

  /**
   * Prints the wrist position to SmartDashboard for debugging.
   */
  public void printWristPosition() {
    SmartDashboard. putNumber("Wrist Position", getWristAngle());
    SmartDashboard.putNumber("Wrist Velocity (deg/s)", getWristVelocity());
    SmartDashboard.putNumber("Wrist Current (A)", wristMotor.getSupplyCurrent(). getValueAsDouble());
  }

  /**
   * Gets the current wrist velocity in degrees per second. 
   * 
   * @return The wrist velocity in degrees/second
   */
  public double getWristVelocity() {
    double rotationsPerSecond = wristMotor.getVelocity().getValueAsDouble();
    return rotationsPerSecond * 360.0;
  }

  /**
   * Sets the wrist motor speed directly (open-loop control). 
   * 
   * @param speed The speed from -1. 0 to 1.0
   */
  public void setWristSpeed(double speed) {
    wristMotor.setControl(voltageRequest. withOutput(speed * 12.0));
  }

  /**
   * Stops the wrist motor. 
   */
  public void stop() {
    wristMotor.setControl(voltageRequest.withOutput(0));
  }

  /**
   * Converts degrees to motor rotations.
   * Phoenix 6 native unit is rotations. 
   */
  private static double degreesToRotations(double degrees) {
    return degrees / 360.0;
  }

  /**
   * Converts motor rotations to degrees. 
   */
  private static double rotationsToDegrees(double rotations) {
    return rotations * 360.0;
  }

  @Override
  public void periodic() {
    // Uncomment for debugging:
    // printWristPosition();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}