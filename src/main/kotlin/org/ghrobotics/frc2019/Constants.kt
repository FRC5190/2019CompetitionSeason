/*
 * FRC Team 5190
 * Green Hope Falcons
 */

package org.ghrobotics.frc2019

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive
import org.ghrobotics.frc2019.subsystems.arm.ArmNativeUnitModel
import org.ghrobotics.frc2019.subsystems.elevator.SpringCascadeNativeUnitModel
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.derivedunits.acceleration
import org.ghrobotics.lib.mathematics.units.derivedunits.velocity
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel
import org.ghrobotics.lib.mathematics.units.nativeunits.STU
import kotlin.math.pow

@Suppress("MemberVisibilityCanBePrivate")
object Constants {

    // PHSYICS
    const val kAccelerationDueToGravity = 9.80665

    // GLOBAL CTRE TIMEOUT
    const val kCTRETimeout = 10

    // MOTOR IDS
    const val kDriveLeftMasterId        = 1
    const val kDriveLeftSlaveId         = 2
    const val kDriveRightMasterId       = 3
    const val kDriveRightSlaveId        = 4
    const val kElevatorMasterId         = 5
    const val kElevatorSlave1Id         = 6
    const val kElevatorSlave2Id         = 7
    const val kElevatorSlave3Id         = 8
    const val kArmId                    = 9
    const val kIntakeLeftId             = 10
    const val kIntakeRightId            = 11
    const val kClimbFrontWinchMasterId  = 12
    const val kClimbFrontWinchSlaveId   = 13
    const val kClimbBackWinchMasterId   = 14
    const val kClimbBackWinchSlaveId    = 15
    const val kClimbWheelId             = 16


    // ANALOG INPUT
    const val kLeftBallSensorId     = 2
    const val kRightBallSensorId    = 3


    // GYROS
    const val kPigeonIMUId = 17


    // PNEUMATICS
    const val kPCMId                        = 41
    const val kIntakeExtensionSolenoidId    = 2
    const val kIntakeLauncherSolenoidId     = 0
    const val kDriveSolenoidId              = 3
    const val kRampsSolenoidId              = 1
    const val kClimberWheelSolenoidId       = 5


    // FIELD
    val kLevel2HabitatRightX            = 4.feet
    val kLevel2HabitatBottomY           = 8.feet
    val kLevel1HabitatRightX            = 7.feet
    val kLevel1HabitatPlatform          = Rectangle2d(Translation2d(4.feet, 7.feet), Translation2d(8.feet, 20.feet))
    val kHypotenuseDifferenceForRamp    = 0.433.inch


    // ROBOT
    val kRobotMass                  = 123.lb
    const val kRobotMomentOfInertia = 9.0 // kg m^2
    const val kRobotAngularDrag     = 6.0 // Nm per rad/s

    val kRobotWidth         = 29.inch
    val kRobotLength        = 30.inch
    val kBumperThickness    = 4.5.inch
    val kIntakeProtrusion   = 15.inch       // Out of frame protrusion.


    // TRANSFORMATIONS
    val kFrontBumperToCenter        = Pose2d(-(kRobotLength / 2.0) - kBumperThickness, 0.meter, 0.degree)
    val kForwardIntakeToCenter      = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusion, 0.meter, 0.degree)
    val kBackwardIntakeToCenter     = Pose2d(kRobotLength / 2.0 + kIntakeProtrusion, 0.meter, 0.degree)
    val kCenterToCamera             = Pose2d(1.inch, -(17).inch, 0.degree)
    val kGroundToCamera             = 30.inch


    // VISION
    val kMaxTargetTrackingDistance = 10.inch
    val kMaxTargetTrackingLifetime = 1.second


    // DRIVE
    val kDriveSensorUnitsPerRotation    = 1440.STU
    val kDriveWheelRadius               = 2.998657.inch
    val kDriveTrackWidth                = 27.75.inch

    val kDriveNativeUnitModel = NativeUnitLengthModel(
        kDriveSensorUnitsPerRotation,
        kDriveWheelRadius
    )

    val kDriveCurrentLimit = 38.amp

    const val kDriveKp = 1.2 // Talon SRX Units
    const val kDriveKd = 0.0

    const val kStaticFrictionVoltage    = 0.8 // Volts
    const val kDriveKv                  = 0.135 // Volts per radians per second
    const val kDriveKa                  = 0.012 // Volts per radians per second per second

    const val kDriveBeta = 2.0 // Inverse meters squared
    const val kDriveZeta = 0.7 // Unitless dampening co-efficient

    private val kDriveDCTransmission = DCMotorTransmission(
        1 / kDriveKv,
        kDriveWheelRadius.value.pow(2) * kRobotMass.value / (2.0 * kDriveKa),
        kStaticFrictionVoltage
    )

    val kDriveModel = DifferentialDrive(
        kRobotMass.value,
        kRobotMomentOfInertia,
        kRobotAngularDrag,
        kDriveWheelRadius.value,
        kDriveTrackWidth.value / 2.0,
        kDriveDCTransmission,
        kDriveDCTransmission
    )


    // ELEVATOR
    // from elevator zero (measured from bottom of second stage)
    val kElevatorSwitchHeight                   = 8.inch
    val kElevatorSwitchNativeUnit               = 2310.STU
    val kElevatorAfterSwitchHeightSample        = 63.inch
    val kElevatorAfterSwitchNativeUnitSample    = 10845.STU

    val kElevatorNativeUnitModel = SpringCascadeNativeUnitModel(
        switchHeight = kElevatorSwitchHeight,
        switchNativeUnit = kElevatorSwitchNativeUnit,
        afterSwitchHeightSample = kElevatorAfterSwitchHeightSample,
        afterSwitchNativeUnitSample = kElevatorAfterSwitchNativeUnitSample
    )

    val kMaxElevatorHeightFromZero      = 67.inch
    val kElevatorSecondStageToArmShaft  = 8.inch
    val kElevatorHeightFromGround       = 6.inch

    val kElevatorCurrentLimit = 25.amp

    val kElevatorClosedLoopVelocityTolerance    = 1.inch.velocity
    val kElevatorClosedLoopTolerance            = 1.inch
    val kElevatorAcceleration                   = 175.inch.acceleration
    val kElevatorCruiseVelocity                 = 200.inch.velocity

    const val kElevatorKp               = 1.0
    const val kElevatorKf               = 0.864 / 2.0
    const val kElevatorBelowSwitchKg    = 0.35 / 12
    const val kElevatorAfterSwitchKg    = 0.7 / 12


    // ARM
    val kArmSensorUnitsPerRotation  = 1024.STU
    val kArmUpTicks                 = 512.STU

    val kArmNativeUnitModel = ArmNativeUnitModel(
        kArmUpTicks,
        90.degree,
        kArmSensorUnitsPerRotation
    )

    val kArmLength          = 30.inch
    val kArmFlipTolerance   = 40.degree

    val kArmCurrentLimit = 15.amp

    val kArmClosedLoopTolerance = 5.degree
    val kArmAcceleration        = 0.1.radian.acceleration
    val kArmCruiseVelocity      = 1.5.radian.velocity

    const val kArmKp = 1.0
    const val kArmKg = 0.005


    // CLIMB
    val kClimbWinchRadius                   = 1.25.inch / 2.0
    val kClimbWinchNativeUnitsPerRotation   = 4096.STU

    val kClimbWinchNativeUnitModel = NativeUnitLengthModel(
        kClimbWinchNativeUnitsPerRotation,
        kClimbWinchRadius
    )

    val kClimbWinchCurrentLimit = 20.amp

    val kClimbWinchClosedLoopTolerance          = 2.inch
    val kClimbWinchClosedLoopVelocityTolerance  = 1.inch.velocity
    val kClimbWinchCruiseVelocity               = 1.5.feet.velocity
    val kClimbWinchAcceleration                 = 1.5.feet.acceleration

    val kClimbWinchKp = 0.3
}
