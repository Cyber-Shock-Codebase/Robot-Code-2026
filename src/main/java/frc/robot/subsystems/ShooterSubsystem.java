package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase
{
  // TODO: Add detailed comments explaining the example, similar to the ExponentiallyProfiledArmSubsystem

  private final SparkMax                   ShooterMastermotor   = new SparkMax(ShooterConstants.SHOOTER_MOTOR0_ID, MotorType.kBrushless);
  // private final SparkMax                   ShooterFollower1     = new SparkMax(ShooterConstants.SHOOTER_MOTOR1_ID, MotorType.kBrushless);
  private final SparkMax                   ShooterFollower2     = new SparkMax(ShooterConstants.SHOOTER_MOTOR2_ID, MotorType.kBrushless);
  private final SparkMax                   ShooterFollower3     = new SparkMax(ShooterConstants.SHOOTER_MOTOR3_ID, MotorType.kBrushless);
  private final SparkMax                   ShooterFollower4     = new SparkMax(ShooterConstants.SHOOTER_MOTOR4_ID, MotorType.kBrushless); 
  // private final SparkMax                   ShooterFollower5     = new SparkMax(ShooterConstants.SHOOTER_MOTOR5_ID, MotorType.kBrushless);
  private final SparkMax                   Feedermotor          = new SparkMax(ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
  
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0.1, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(  0.64308681672)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Set up followers, each pair has a boolean for whether that motor should be inverted relative to the master
      .withFollowers(
                    //  Pair.of(ShooterFollower1, false),
                     Pair.of(ShooterFollower2, false),
                     Pair.of(ShooterFollower3, true),
                     Pair.of(ShooterFollower4, true)
                    //  Pair.of(ShooterFollower5, false)
                     );
  private final SmartMotorController       motor       = new SparkWrapper(ShooterMastermotor, DCMotor.getNEO(4), motorConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig(motor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(7))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(ShooterConstants.minSpeed, ShooterConstants.maxSpeed)
      .withSpeedometerSimulation(ShooterConstants.maxSpeed);
  private final FlyWheel       shooter       = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {}

  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

  public Command setDutyCycle(double dutyCycle) {return shooter.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.setSpeed(speed);}

  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

  public Command standby() {return shooter.setSpeed(ShooterConstants.minSpeed);}

  // public void feedWhen(double feederSpeed, Supplier<AngularVelocity> targetShootSpeed, Supplier<AngularVelocity> currentShootSpeed, double tolerance) {
  //   if (currentShootSpeed.get().isNear(targetShootSpeed.get(), tolerance)) 
  //    {
  //      Feedermotor.set(feederSpeed);
  //    } else {
  //      Feedermotor.set(0);
  //    }
  // }

  public void feed() {
    Feedermotor.set(ShooterConstants.feederSpeedNorm);
  }

  public void feed(double feederSpeed) {
    Feedermotor.set(feederSpeed);
  }

  public void stopFeeding() {
    Feedermotor.set(0);
  }

  // public void armshooter() {
  //   if !shooter.getSpeed().equals(shooter.setSpeed()) feedermotor.setVoltage(Volts.of(ShooterConstants.feederSpeedNorm));
  // }

  public Command sysId() {return shooter.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}

  @Override
  public void periodic() {
      shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
      shooter.simIterate();
  }
}