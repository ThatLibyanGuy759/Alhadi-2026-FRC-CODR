package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class AutoTestCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Pose2d m_targetPose;
    

    private final PIDController xController = new PIDController(2.0, 0, 0);
    private final PIDController yController = new PIDController(2.0, 0, 0);
    private final PIDController thetaController = new PIDController(1.5, 0, 0);

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric();

    public AutoTestCommand(CommandSwerveDrivetrain drivetrain, Pose2d targetPose) {
        m_drivetrain = drivetrain;
        m_targetPose = targetPose;
        addRequirements(m_drivetrain);
        

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        

        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        thetaController.setTolerance(Math.toRadians(2));
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_drivetrain.getState().Pose;

   
        double xSpeed = xController.calculate(currentPose.getX(), m_targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), m_targetPose.getY());
        double thetaSpeed = thetaController.calculate(currentPose.getRotation().getRadians(), 
                                                       m_targetPose.getRotation().getRadians());

        m_drivetrain.setControl(m_driveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(thetaSpeed));
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
    }
}