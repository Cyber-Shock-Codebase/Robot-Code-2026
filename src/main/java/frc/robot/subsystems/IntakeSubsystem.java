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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase
{
  // TODO: Add detailed comments explaining the example, similar to the ExponentiallyProfiledArmSubsystem

  private final SparkMax IntakeMotor    = new SparkMax(Constants.intakeConstants.MOTOR_ID, MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor       = new SparkWrapper(IntakeMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig IntakeConfig = new FlyWheelConfig(motor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("IntakeMech", TelemetryVerbosity.HIGH)
      .withSoftLimit(RPM.of(-5000), RPM.of(5000))
      .withSpeedometerSimulation(RPM.of(5000));
  private final FlyWheel       Intake       = new FlyWheel(IntakeConfig);

  public IntakeSubsystem() {}

  public AngularVelocity getVelocity() {return Intake.getSpeed();}
  public Command setVelocity(AngularVelocity speed) {return Intake.setSpeed(speed);}

  public Command setDutyCycle(double dutyCycle) {return Intake.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return Intake.setSpeed(speed);}

  public Command setDutyCycle(Supplier<Double> dutyCycle) {return Intake.set(dutyCycle);}

  public Command sysId() {return Intake.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));}
  @Override
  public void periodic() {
      Intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
      Intake.simIterate();
  }
}