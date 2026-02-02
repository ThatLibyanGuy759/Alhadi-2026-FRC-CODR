package frc.robot.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterMotorOne;
    private final TalonFX shooterMotorTwo;
    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage voltageRequest = new VelocityVoltage(0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC velocityTorqueRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut sysIdControl = new VoltageOut(0);
    private final SysIdRoutine m_SysIdRoutine;

 
    private FlywheelSim m_flywheelSim;
    private Mechanism2d m_mech;
    private MechanismLigament2d m_fuelVisual;
    

    private double m_shotTime = 0;
    private final double kLaunchVelocity = 15.0; 
    private final double kLaunchAngleRad = Math.toRadians(35); 

    public ShooterSubsystem() {
        shooterMotorOne = new TalonFX(ShooterConstants.shooterMotorOneID);
        shooterMotorTwo = new TalonFX(ShooterConstants.shooterMotorTwoID);

        m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(null, Volts.of(4), null, (state) -> SignalLogger.writeString("state", state.toString())),
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

        if (RobotBase.isSimulation()) {
            var flywheelPlant = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(2), 0.0008, 1.0);
            m_flywheelSim = new FlywheelSim(flywheelPlant, DCMotor.getFalcon500(2));

            m_mech = new Mechanism2d(50, 50);
            MechanismRoot2d root = m_mech.getRoot("ShooterExit", 10, 25);
            m_fuelVisual = root.append(new MechanismLigament2d("Fuel", 0, 0, 6, new Color8Bit(Color.kOrange)));
            
            SmartDashboard.putData("Shooter/MechVisual", m_mech);
        }
    }

    @Override
    public void simulationPeriodic() {
        m_flywheelSim.setInput(shooterMotorOne.getSimState().getMotorVoltage());
        m_flywheelSim.update(0.020);

        double velocityRPS = m_flywheelSim.getAngularVelocityRPM() / 60.0;
        shooterMotorOne.getSimState().setRotorVelocity(velocityRPS);
        shooterMotorTwo.getSimState().setRotorVelocity(velocityRPS);
        
        shooterMotorOne.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
        shooterMotorTwo.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());

       
        if (velocityRPS > 20.0) { 
            m_shotTime += 0.020;
            
     
            double x = kLaunchVelocity * Math.cos(kLaunchAngleRad) * m_shotTime;
            double z = (kLaunchVelocity * Math.sin(kLaunchAngleRad) * m_shotTime) - (0.5 * 9.81 * m_shotTime * m_shotTime);
            z += 0.5; // Start height of shooter (0.5 meters)

            if (z < 0) { 
                m_shotTime = 0;
            }

            Pose3d fuelPose = new Pose3d(new Translation3d(x, 0, z), new Rotation3d());
            SmartDashboard.putNumberArray("Shooter/FuelPose3d", new double[] {
                fuelPose.getX(), fuelPose.getY(), fuelPose.getZ(),
                fuelPose.getRotation().getQuaternion().getW(),
                fuelPose.getRotation().getQuaternion().getX(),
                fuelPose.getRotation().getQuaternion().getY(),
                fuelPose.getRotation().getQuaternion().getZ()
            });

   
            m_fuelVisual.setLength(x * 10); 
        } else {
            m_shotTime = 0;
            m_fuelVisual.setLength(0);
            SmartDashboard.putNumberArray("Shooter/FuelPose3d", new double[] {0,0,-1,1,0,0,0});
        }
    }

    public void configureMotors() {
        shooterMotorOne.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());
        shooterMotorTwo.getConfigurator().apply(Configs.ShooterConfigs.shooterMotorConfig());
    }

    public void shootFuel(double speed) {
        shooterMotorOne.setControl(dutyCycle.withOutput(speed));
        shooterMotorTwo.setControl(dutyCycle.withOutput(speed));
    }

    public void runVelocity(double rps) {
        shooterMotorOne.setControl(voltageRequest.withVelocity(rps));
        shooterMotorTwo.setControl(voltageRequest.withVelocity(rps));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/ActualRPS", shooterMotorOne.getVelocity().getValueAsDouble());
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) { return m_SysIdRoutine.dynamic(direction); }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) { return m_SysIdRoutine.quasistatic(direction); }
}