package frc.robot.wrist;

public final class WristConstants{
    public static final int wristMotorCANId = 12;//Temp
    
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double minOutput = -1;
    public static final double maxOutput = 1;
    public static final double velocityFF = 0.5;

    public static final double maxPosition = 225;
    public static final double minPositon = 5;

    // Added feedforward and limits for configs
    public static final double supplyCurrentLimit = 30.0; // TODO: set actual wrist current limit
    public static final double statorCurrentLimit = 30.0; // TODO: set actual wrist stator limit
    public static final double kV = 0.0; // TODO: tune feedforward V
    public static final double kS = 0.0; // TODO: tune feedforward S
  }
