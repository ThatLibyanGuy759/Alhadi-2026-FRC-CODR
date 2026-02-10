// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.shooter.ShooterSubsystem;
import frc.robot.wrist.WristSubsystem;

/** An example command that uses an example subsystem. */
public class WristExtendCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
  private final WristSubsystem m_wristSubsystem;
  private final double wristSpeed;
  /**
   * Creates a new ExampleCommand.
   * 
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristExtendCommand(WristSubsystem m_wristSubsystem, double wristSpeed) {
    this.m_wristSubsystem = m_wristSubsystem;
    this.wristSpeed = wristSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wristSubsystem.setWristSpeed(wristSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.setWristSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}