package frc.robot.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import java.io.InputStream;
import java.util.List;
import java.util.Map;
import java.util.concurrent.PriorityBlockingQueue;
import java.util.function.Supplier;

import com.fasterxml.jackson.databind.util.RootNameLookup;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;




public class ShootOnTheMoveCommand extends Command {
  private final SwerveSubsystem drivetrain;
  final ShooterSubsystem shooter;
  private final SwerveInputStream inputStream;
  private Translation3d aimPoint; // The point to aim at
  private AngularVelocity latestShootSpeed;
  private Angle HoodAngle;
  private Angle latestWindage;
  private final Angle                      setpointTolerance   = Degrees.of(0);
  private final AngularVelocity            maxProfiledVelocity = RotationsPerSecond.of(60);
  private final AngularAcceleration    maxProfiledAcceleration = RotationsPerSecondPerSecond.of(60);
  private static AngularVelocity targetShootSpeed = RPM.of(0);
  private static boolean withinTolerance = false;

  private final ProfiledPIDController  pidController           = new ProfiledPIDController(1,
                                                                                          0,
                                                                                          0.01,
                                                                                           new Constraints(
                                                                                           maxProfiledVelocity.in(RadiansPerSecond),
                                                                                           maxProfiledAcceleration.in(RadiansPerSecondPerSecond)));
  private final SimpleMotorFeedforward feedforward             = new SimpleMotorFeedforward(0, 0, 0);

  public ShootOnTheMoveCommand(
      SwerveSubsystem drivetrain,
      SwerveInputStream inputStream,
      ShooterSubsystem shooter,
      Translation3d aimPoint) {
    this.drivetrain = drivetrain;
    this.inputStream = inputStream;
    this.shooter = shooter;
    this.aimPoint = aimPoint;
    pidController.setTolerance(setpointTolerance.in(Radian));
    pidController.enableContinuousInput(0, 2*Math.PI);

    

    // We use the drivetrain to determine linear velocity, but don't require it for
    // control. We
    // will be using the superstructure to control the shooting mechanism so it's a
    // requirement.
    // addRequirements(superstructure);

    // TODO: figure out if the above is actually required. Right now, when you start
    // some other command, the auto aim can't start back up again
  }

  @Override
  public void initialize() {
    super.initialize();

    HoodAngle = Degrees.of(28);
    latestWindage = Degrees.of(drivetrain.getHeading().getDegrees());
    latestShootSpeed = shooter.getVelocity();

    // TODO: when this current command ends, we should probably cancel the dynamic
    // aim command
    // superstructure
    //     .aimDynamicCommand(
    //         () -> {
    //           return this.latestShootSpeed;
    //         },
    //         () -> {
    //           return this.latestTurretAngle;
    //         },
    //         () -> {
    //           return this.latestHoodAngle;
    //         })
    //     .schedule();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void execute() {
    // Calculate trajectory to aimPoint
    var target = AllianceFlipUtil.apply(aimPoint);

    var shooterLocation =
        new Translation3d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), 0)
            .plus( new Translation3d(-5.72,0,16.4));

    // Ignore this parameter for now, the range tables will account for it :/
    // var deltaH = target.getMeasureZ().minus(shooterLocation.getMeasureZ());
    var shooterOnGround = new Translation2d(shooterLocation.getX(), shooterLocation.getY());
    var targetOnGround = new Translation2d(target.getX(), target.getY());

    var distanceToTarget = Meters.of(shooterOnGround.getDistance(targetOnGround));

    // Get time of flight. We could try to do this analytically but for now it's
    // easier and more realistic
    // to use a simple linear approximation based on empirical data.
    double timeOfFlight = getFlightTime(distanceToTarget);

    // Calculate corrective vector based on our current velocity multiplied by time
    // of flight.
    // If we're stationary, this should be zero. If we're backing up, this will be
    // "ahead" of the target, etc.
    var updatedPosition = drivetrain.getFieldVelocity().times(timeOfFlight);
    var correctiveVector =
        new Translation2d(updatedPosition.vxMetersPerSecond, updatedPosition.vyMetersPerSecond)
            .unaryMinus();
    var correctiveVector3d = new Translation3d(correctiveVector.getX(), correctiveVector.getY(), 0);

    // Logger.recordOutput("FieldSimulation/AimTargetCorrected",
    //     new Pose3d(target.plus(correctiveVector3d), Rotation3d.kZero));

    var correctedTarget = targetOnGround.plus(correctiveVector);

    var vectorToTarget = drivetrain.getPose().getTranslation().minus(correctedTarget);

    var correctedDistance = Meters.of(vectorToTarget.getNorm());
    var calculatedHeading = vectorToTarget.getAngle().getMeasure();
        // .rotateBy(drivetrain.getHeading()).getMeasure();
    // .minus(Degrees.of(180)); // .unaryminus

    // Logger.recordOutput("ShootOnTheMove/RobotHeading", drivetrain.getHeading());
    // Logger.recordOutput("ShootOnTheMove/CalculatedHeading", calculatedHeading);
    // Logger.recordOutput("ShootOnTheMove/distanceToTarget", distanceToTarget);
    
    // drivetrain.getSwerveDrive().field.getObject("targetPose").setTrajectory(
    //         TrajectoryGenerator.generateTrajectory(
    //         new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), Rotation2d.fromRadians(calculatedHeading.in(Radian))),
    //         List.of(),
    //         new Pose2d(correctedTarget.getX(), correctedTarget.getY(), drivetrain.getHeading()),
    //         new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0))));

    latestWindage = calculatedHeading;
    latestShootSpeed = calculateRequiredShooterSpeed(correctedDistance);
    targetShootSpeed = calculateRequiredShooterSpeed(correctedDistance);

    // TODO: add this back if/when we have a real hood, for now, just set it to the
    // current angle
    // latestHoodAngle = calculateRequiredHoodAngle(correctedDistance);
    HoodAngle = Degrees.of(28);

    // Set the shooter speed to the calculated values
    shooter.setVelocity(targetShootSpeed);

    // Set the robot's heading to the turret angle 
    var output = pidController.calculate(drivetrain.getHeading().getRadians(),
                                         new State(calculatedHeading.magnitude(), 0));
    var feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    var originalSpeed     = this.inputStream.get();
    originalSpeed.omegaRadiansPerSecond =  (output + feedforwardOutput);
    originalSpeed.vxMetersPerSecond = (originalSpeed.vxMetersPerSecond/3);
    originalSpeed.vyMetersPerSecond = (originalSpeed.vyMetersPerSecond/3);
    drivetrain.driveFieldOriented(originalSpeed);
    double rawAngleDiff = drivetrain.getHeading().getRadians() - calculatedHeading.in(Radian);
    double angleDiff = Math.atan2(Math.sin(rawAngleDiff), Math.cos(rawAngleDiff)); // normalize to [-pi, pi]
    withinTolerance = shooter.getVelocity().isNear(targetShootSpeed, ShooterConstants.flywheelTolerance)
        && Math.abs(angleDiff) <= ShooterConstants.headingTolerance.in(Radian);
    // drivetrain.driveFieldOriented(
    //   SwerveInputStream.of(drivetrain.getSwerveDrive(),
    //                                                             () -> driverXbox.getLeftY() * -1,
    //                                                             () -> driverXbox.getLeftX() * -1)
    //                                                         .withControllerRotationAxis(driverXbox::getRightX)
    //                                                         .deadband(OperatorConstants.DEADBAND)
    //                                                         .scaleTranslation(0.8)
    //                                                         .allianceRelativeControl(true)
    //                                                         .withControllerHeadingAxis(() -> Math.cos(latestWindage.magnitude()),
    //                                                                                    () -> Math.sin(latestWindage.magnitude()))
    //                                                         .headingWhile(true)
    //   );

    SmartDashboard.putNumber("shoot speed", latestShootSpeed.magnitude());
    SmartDashboard.putNumber("target angle", latestWindage.magnitude());
    SmartDashboard.putBoolean(getName(), withinTolerance);
    SmartDashboard.putNumber(getName(), angleDiff);
    SmartDashboard.putNumber("VelocityDiff", shooter.getVelocity().minus(targetShootSpeed).magnitude());
    // SmartDashboard.putBoolean("Encoder A Raw", rotorSeededFromAbs);



    System.out.println("Shooting at distance: " + correctedDistance
     + " requires speed: " + latestShootSpeed
     + ", is speed: " + shooter.getVelocity()
     + ", turret angle: " + drivetrain.getHeading()
     + ", target angle:" + calculatedHeading
     + ", within tolerance: " + withinTolerance
     + " Target X: " + correctedTarget.getX()
     + " Target Y: " + correctedTarget.getY()
    //  + Math.cos(latestWindage.magnitude())
    //  + Math.sin(latestWindage.magnitude())
     );
  }

  public static AngularVelocity getTargetShootSpeed() {
    // This is a placeholder implementation. In a real implementation, you would
    // want to store the latest calculated shooter speed in a way that this method
    // can access it.
    return targetShootSpeed;
  }

  public static boolean isWithinTolerance() {
    return withinTolerance;
  }

  private double getFlightTime(Distance distanceToTarget) {
    // Simple linear approximation based on empirical data.
    return TIME_OF_FLIGHT_BY_DISTANCE.get(distanceToTarget.in(Meters));
  }

  private AngularVelocity calculateRequiredShooterSpeed(Distance distanceToTarget) {
    return RPM.of(SHOOTING_SPEED_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  private Angle calculateRequiredHoodAngle(Distance distanceToTarget) {
    return Degrees.of(HOOD_ANGLE_BY_DISTANCE.get(distanceToTarget.in(Meters)));
  }

  // meters, seconds
  private static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(1.0, 1.0), Map.entry(4.86, 1.5));
  // Distance in meters, time in seconds
  // TODO: add more data points here.
  // CLOSE: NEED
  // MID: maybe good enough
  // FAR: NEED

  // meters, RPS
  private static final InterpolatingDoubleTreeMap SHOOTING_SPEED_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(Inches.of(88).in(Meters), 5800.0),
          Map.entry(Inches.of(101).in(Meters), 6300.0),
          Map.entry(Inches.of(133).in(Meters), 7050.0),
          Map.entry(Inches.of(166).in(Meters), 7900.0),
          Map.entry(Inches.of(209).in(Meters), 8600.0), 
          Map.entry(Inches.of(118.51).in(Meters), 7500.0),
          Map.entry(Inches.of(111.51).in(Meters), 7000.0), 
          Map.entry(Inches.of(97.51).in(Meters), 6800.0), 
          Map.entry(Inches.of(87.51).in(Meters), 6600.0), 
          Map.entry(Inches.of(77.51).in(Meters), 6100.0));

  // meters, degrees
  private static final InterpolatingDoubleTreeMap HOOD_ANGLE_BY_DISTANCE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(1.0, 15.0), Map.entry(2.0, 30.0), Map.entry(3.0, 45.0));
}
