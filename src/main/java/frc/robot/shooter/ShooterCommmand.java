// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommmand extends Command {
  private final ShooterSubsystem m_shooterSubsystem;
  private final double m_targetRPS;

  public ShooterCommmand(ShooterSubsystem shooterSubsystem, double targetRPS) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_targetRPS = targetRPS;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.shootFuel(m_targetRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.shootFuel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}