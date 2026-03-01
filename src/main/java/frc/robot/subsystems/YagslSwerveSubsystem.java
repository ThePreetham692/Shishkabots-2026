package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.io.File;
import java.util.Locale;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;

public class YagslSwerveSubsystem extends SubsystemBase {
  private static final String[] MODULE_LABELS = {"FL", "FR", "BL", "BR"};

  private final SlewRateLimiter xSpeedLimiter =
      new SlewRateLimiter(Constants.DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
  private final SlewRateLimiter ySpeedLimiter =
      new SlewRateLimiter(Constants.DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
  private final SlewRateLimiter rotLimiter =
      new SlewRateLimiter(Constants.DriveConstants.MAX_ROTATIONAL_SLEW_RATE_RPS);
  private final Field2d field = new Field2d();
  private final GenericEntry configLoadedEntry;
  private final GenericEntry headingDegreesEntry;
  private final GenericEntry maxSpeedEntry;
  private final GenericEntry maxAngularSpeedEntry;
  private final GenericEntry[] rawAbsoluteEntries = new GenericEntry[MODULE_LABELS.length];
  private final GenericEntry[] rawAngleEntries = new GenericEntry[MODULE_LABELS.length];
  private final GenericEntry[] rawDriveEntries = new GenericEntry[MODULE_LABELS.length];
  private final GenericEntry[] measuredSpeedEntries = new GenericEntry[MODULE_LABELS.length];
  private final GenericEntry[] measuredAngleEntries = new GenericEntry[MODULE_LABELS.length];
  private final GenericEntry[] calibrationAbsEntries = new GenericEntry[MODULE_LABELS.length];

  private SwerveDrive swerveDrive;
  private boolean configLoaded;

  public YagslSwerveSubsystem() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drivebase");
    configLoadedEntry =
        driveTab
            .add("YAGSL Config Loaded", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(2, 1)
            .getEntry();
    headingDegreesEntry =
        driveTab.add("Heading (deg)", 0.0).withPosition(0, 1).withSize(2, 1).getEntry();
    maxSpeedEntry =
        driveTab.add("Max Speed (mps)", 0.0).withPosition(0, 2).withSize(2, 1).getEntry();
    maxAngularSpeedEntry =
        driveTab.add("Max Angular (rps)", 0.0).withPosition(0, 3).withSize(2, 1).getEntry();

    for (int i = 0; i < MODULE_LABELS.length; i++) {
      ShuffleboardLayout moduleLayout =
          driveTab
              .getLayout(MODULE_LABELS[i], BuiltInLayouts.kList)
              .withPosition(2 + i, 0)
              .withSize(1, 5);
      rawAbsoluteEntries[i] = moduleLayout.add("Abs", Double.NaN).getEntry();
      rawAngleEntries[i] = moduleLayout.add("Angle Enc", Double.NaN).getEntry();
      rawDriveEntries[i] = moduleLayout.add("Drive Enc", Double.NaN).getEntry();
      measuredSpeedEntries[i] = moduleLayout.add("Speed", Double.NaN).getEntry();
      measuredAngleEntries[i] = moduleLayout.add("Angle Deg", Double.NaN).getEntry();
    }

    calibrationAbsEntries[0] =
        driveTab
            .add("Cal FL Abs", Double.NaN)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 4)
            .withSize(1, 1)
            .getEntry();
    calibrationAbsEntries[1] =
        driveTab
            .add("Cal FR Abs", Double.NaN)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 4)
            .withSize(1, 1)
            .getEntry();
    calibrationAbsEntries[2] =
        driveTab
            .add("Cal BL Abs", Double.NaN)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 5)
            .withSize(1, 1)
            .getEntry();
    calibrationAbsEntries[3] =
        driveTab
            .add("Cal BR Abs", Double.NaN)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(1, 5)
            .withSize(1, 1)
            .getEntry();

    File swerveConfigDir = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive =
          new SwerveParser(swerveConfigDir)
              .createSwerveDrive(Constants.DriveConstants.MAX_SPEED_IN_MPS);
      configLoaded = true;
      configureSwerveDrive();
      DriverStation.reportWarning(
          "YAGSL loaded config from " + swerveConfigDir.getAbsolutePath(), false);
    } catch (Exception e) {
      configLoaded = false;
      swerveDrive = null;
      DriverStation.reportError(
          "YAGSL failed to load config from "
              + swerveConfigDir.getAbsolutePath()
              + ": "
              + e.getMessage(),
          e.getStackTrace());
    }

    SmartDashboard.putData("Field", field);
    SmartDashboard.putBoolean("YAGSL/ConfigLoaded", configLoaded);
    configureAutoBuilder();
  }

  public boolean isConfigLoaded() {
    return configLoaded;
  }

  public void drive(double xInput, double yInput, double omegaInput) {
    drive(xInput, yInput, omegaInput, true);
  }

  public void drive(double xInput, double yInput, double omegaInput, boolean fieldRelative) {
    if (!configLoaded || swerveDrive == null) {
      return;
    }

    if (Math.abs(xInput) < 1E-6 && Math.abs(yInput) < 1E-6 && Math.abs(omegaInput) < 1E-6) {
      stop();
      return;
    }

    double vx = xSpeedLimiter.calculate(xInput) * swerveDrive.getMaximumChassisVelocity();
    double vy = ySpeedLimiter.calculate(yInput) * swerveDrive.getMaximumChassisVelocity();
    double omega =
        rotLimiter.calculate(omegaInput) * swerveDrive.getMaximumChassisAngularVelocity();

    driveMetersPerSecond(vx, vy, omega, fieldRelative);
  }

  public void driveMetersPerSecond(
      double xMetersPerSecond,
      double yMetersPerSecond,
      double omegaRadiansPerSecond,
      boolean fieldRelative) {
    if (swerveDrive == null) {
      return;
    }

    ChassisSpeeds speeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond, getHeading())
            : new ChassisSpeeds(xMetersPerSecond, yMetersPerSecond, omegaRadiansPerSecond);
    driveRobotRelative(speeds);
  }

  public void drive(ChassisSpeeds speeds) {
    driveRobotRelative(speeds);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    if (swerveDrive == null) {
      return;
    }
    swerveDrive.drive(speeds);
  }

  public void stop() {
    if (swerveDrive == null) {
      return;
    }
    xSpeedLimiter.reset(0.0);
    ySpeedLimiter.reset(0.0);
    rotLimiter.reset(0.0);
    swerveDrive.drive(new ChassisSpeeds());
  }

  public void lockPose() {
    if (swerveDrive != null) {
      swerveDrive.lockPose();
    }
  }

  public void zeroGyro() {
    if (swerveDrive != null) {
      swerveDrive.zeroGyro();
    }
  }

  public void zeroHeading() {
    zeroGyro();
  }

  public Rotation2d getGyroRotation() {
    return getHeading();
  }

  public Rotation2d getHeading() {
    return swerveDrive == null ? new Rotation2d() : swerveDrive.getYaw();
  }

  public Pose2d getPose() {
    return swerveDrive == null ? new Pose2d() : swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d pose) {
    if (swerveDrive != null) {
      swerveDrive.resetOdometry(pose);
    }
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return swerveDrive == null ? new ChassisSpeeds() : swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldSpeeds() {
    return swerveDrive == null ? new ChassisSpeeds() : swerveDrive.getFieldVelocity();
  }

  public SwerveModuleState[] getModuleStates() {
    return swerveDrive == null ? new SwerveModuleState[0] : swerveDrive.getStates();
  }

  public SwerveModulePosition[] getModulePositions() {
    return swerveDrive == null ? new SwerveModulePosition[0] : swerveDrive.getModulePositions();
  }

  public Command driveToEndPose(Pose2d endPose) {
    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    return AutoBuilder.pathfindToPose(endPose, constraints, 0.0);
  }

  public void printCalibrationAbsoluteEncoders() {
    SwerveModule[] modules = getModules();
    if (modules.length < 4) {
      DriverStation.reportWarning("YAGSL calibration print: modules unavailable.", false);
      return;
    }
    String message =
        String.format(
            Locale.US,
            "YAGSL ABS OFFSETS -> FL=%.6f FR=%.6f BL=%.6f BR=%.6f",
            modules[0].getRawAbsolutePosition(),
            modules[1].getRawAbsolutePosition(),
            modules[2].getRawAbsolutePosition(),
            modules[3].getRawAbsolutePosition());
    DriverStation.reportWarning(message, false);
  }

  private void configureSwerveDrive() {
    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setModuleStateOptimization(true);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setChassisDiscretization(true, true, 0.02);
    swerveDrive.setModuleEncoderAutoSynchronize(true, 1.0);
    swerveDrive.setMaximumAllowableSpeeds(
        Constants.DriveConstants.MAX_SPEED_IN_MPS,
        Constants.DriveConstants.MAX_ANGULAR_SPEED_IN_RPS);
  }

  private void configureAutoBuilder() {
    try {
      Constants.DriveConstants.pathPlannerConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      try {
        Constants.DriveConstants.pathPlannerConfig =
            new RobotConfig(
                50.0,
                6.0,
                new ModuleConfig(
                    0.0508,
                    Constants.DriveConstants.MAX_SPEED_IN_MPS,
                    1.19,
                    DCMotor.getNEO(1).withReduction(6.75),
                    40.0,
                    1),
                Constants.DriveConstants.FRONT_LEFT_LOCATION,
                Constants.DriveConstants.FRONT_RIGHT_LOCATION,
                Constants.DriveConstants.BACK_LEFT_LOCATION,
                Constants.DriveConstants.BACK_RIGHT_LOCATION);
      } catch (Exception ignored) {
        Constants.DriveConstants.pathPlannerConfig = null;
      }
    }

    if (Constants.DriveConstants.pathPlannerConfig == null || swerveDrive == null) {
      DriverStation.reportWarning(
          "YAGSL PathPlanner AutoBuilder not configured: missing robot config or drive",
          false);
      return;
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getCurrentSpeeds,
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(1.0, 0.0, 0.0)),
        Constants.DriveConstants.pathPlannerConfig,
        () ->
            DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false),
        this);
  }

  private SwerveModule[] getModules() {
    return swerveDrive == null ? new SwerveModule[0] : swerveDrive.getModules();
  }

  private void publishModuleTelemetry() {
    SwerveModule[] modules = getModules();
    if (modules.length == 0) {
      for (String label : MODULE_LABELS) {
        SmartDashboard.putNumber("YAGSL/" + label + "/RawAbsoluteEncoder", Double.NaN);
        SmartDashboard.putNumber("YAGSL/" + label + "/RawAngleEncoder", Double.NaN);
        SmartDashboard.putNumber("YAGSL/" + label + "/RawDriveEncoder", Double.NaN);
        SmartDashboard.putNumber("YAGSL/" + label + "/MeasuredSpeedMps", Double.NaN);
        SmartDashboard.putNumber("YAGSL/" + label + "/MeasuredAngleDeg", Double.NaN);
      }
      for (int i = 0; i < MODULE_LABELS.length; i++) {
        rawAbsoluteEntries[i].setDouble(Double.NaN);
        rawAngleEntries[i].setDouble(Double.NaN);
        rawDriveEntries[i].setDouble(Double.NaN);
        measuredSpeedEntries[i].setDouble(Double.NaN);
        measuredAngleEntries[i].setDouble(Double.NaN);
        calibrationAbsEntries[i].setDouble(Double.NaN);
      }
      return;
    }

    int count = Math.min(modules.length, MODULE_LABELS.length);
    for (int i = 0; i < count; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = module.getState();
      String label = MODULE_LABELS[i];

      SmartDashboard.putNumber("YAGSL/" + label + "/RawAbsoluteEncoder", module.getRawAbsolutePosition());
      SmartDashboard.putNumber("YAGSL/" + label + "/RawAngleEncoder", module.getRelativePosition());
      SmartDashboard.putNumber("YAGSL/" + label + "/RawDriveEncoder", module.getPosition().distanceMeters);
      SmartDashboard.putNumber("YAGSL/" + label + "/MeasuredSpeedMps", state.speedMetersPerSecond);
      SmartDashboard.putNumber("YAGSL/" + label + "/MeasuredAngleDeg", state.angle.getDegrees());

      rawAbsoluteEntries[i].setDouble(module.getRawAbsolutePosition());
      rawAngleEntries[i].setDouble(module.getRelativePosition());
      rawDriveEntries[i].setDouble(module.getPosition().distanceMeters);
      measuredSpeedEntries[i].setDouble(state.speedMetersPerSecond);
      measuredAngleEntries[i].setDouble(state.angle.getDegrees());
      calibrationAbsEntries[i].setDouble(module.getRawAbsolutePosition());
    }
  }

  @Override
  public void periodic() {
    double headingDegrees = getHeading().getDegrees();
    double maxSpeed = swerveDrive == null ? 0.0 : swerveDrive.getMaximumChassisVelocity();
    double maxAngularSpeed = swerveDrive == null ? 0.0 : swerveDrive.getMaximumChassisAngularVelocity();

    SmartDashboard.putBoolean("YAGSL/ConfigLoaded", configLoaded);
    SmartDashboard.putNumber("YAGSL/HeadingDegrees", headingDegrees);
    SmartDashboard.putNumber("YAGSL/MaxSpeedMps", maxSpeed);
    SmartDashboard.putNumber("YAGSL/MaxAngularSpeedRps", maxAngularSpeed);

    configLoadedEntry.setBoolean(configLoaded);
    headingDegreesEntry.setDouble(headingDegrees);
    maxSpeedEntry.setDouble(maxSpeed);
    maxAngularSpeedEntry.setDouble(maxAngularSpeed);

    field.setRobotPose(getPose());
    publishModuleTelemetry();
  }
}
