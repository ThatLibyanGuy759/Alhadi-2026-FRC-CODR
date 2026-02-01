// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooter;


import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShooterCommmand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  // private final ExampleSubsystem m_subsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final double targetRPS;
  private final double kickerSpeed;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommmand(ShooterSubsystem m_shooterSubsystem, double targetRPS, double kickerSpeed) {
    // m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooterSubsystem = m_shooterSubsystem;
    this.targetRPS = targetRPS;
    this.kickerSpeed = kickerSpeed;
    addRequirements(m_shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.runVelocityTorqueFOC(targetRPS, kickerSpeed);
    m_shooterSubsystem.printCurrentLimits();
    m_shooterSubsystem.printRPM();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.runVelocityTorqueFOC(0, 0);
    m_shooterSubsystem.printCurrentLimits();
    m_shooterSubsystem.printRPM();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
