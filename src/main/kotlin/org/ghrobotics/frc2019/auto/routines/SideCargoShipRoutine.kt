package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.frc2019.auto.*
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.ConditionalCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.map
import org.ghrobotics.lib.utils.withEquals

fun sideCargoShipRoutine() = autoRoutine {

    val cargoShip1Adjusted = Trajectories.Waypoint(
        trueLocation = Field.kCargoShipS2,
        transform = Constants.kForwardIntakeToCenter
    )
    val cargoShip2Adjusted = Trajectories.Waypoint(
        trueLocation = Field.kCargoShipS2,
        transform = Constants.kForwardIntakeToCenter
    )
    val cargoShip3Adjusted = Trajectories.Waypoint(
        trueLocation = Field.kCargoShipS3,
        transform = Constants.kForwardIntakeToCenter
    )
    val cargoDepotAdjusted = Trajectories.Waypoint(
        trueLocation = Field.kDepotBRCorner,
        transform = Constants.kBackwardIntakeToCenter
    )
    val loadingStationAdjusted = Trajectories.Waypoint(
        trueLocation = Field.kLoadingStation,
        transform = Constants.kBackwardIntakeToCenter
    )

    val intermediatePoint = Trajectories.Waypoint(Pose2d(15.feet, 4.951.feet, 17.degree))


    val path1 = Trajectories.waypoints(Trajectories.kSideStart.asWaypoint(), cargoShip1Adjusted)
        .generateTrajectory(reversed = false)

    val path2 = Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO).map(
        Trajectories.waypoints(cargoShip1Adjusted, intermediatePoint, cargoDepotAdjusted).generateTrajectory(true),
        Trajectories.waypoints(cargoShip1Adjusted, intermediatePoint, loadingStationAdjusted).generateTrajectory(true)
    )

    val path3 = Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO).map(
        Trajectories.waypoints(cargoDepotAdjusted, intermediatePoint, cargoShip2Adjusted).generateTrajectory(false),
        Trajectories.waypoints(loadingStationAdjusted, intermediatePoint, cargoShip2Adjusted).generateTrajectory(false)
    )

    +ConditionalCommand(
        Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
        IntakeCargoCommand(IntakeSubsystem.Direction.HOLD),
        IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
    )

    +parallel {
        +DriveSubsystem.followTrajectory(path1)
        +sequential {
            +executeFor(path1.duration - 2.second, ClosedLoopArmCommand(30.degree))
            +ConditionalCommand(
                Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
                Superstructure.kFrontCargoFromLoadingStation,
                Superstructure.kFrontHatchFromLoadingStation
            )
        }
    }

    +ConditionalCommand(
        Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
        IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE),
        IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    )

    +DelayCommand(0.1.second)

    +parallel {
        +DriveSubsystem.followTrajectory(path2)
        +sequential {
            +DelayCommand(.2.second)
            +ConditionalCommand(
                Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO),
                Superstructure.kBackCargoIntake,
                Superstructure.kBackHatchFromLoadingStation
            )
        }
    }

    +ConditionalCommand(
        Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO),
        IntakeCargoCommand(IntakeSubsystem.Direction.HOLD),
        IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
    )

    +parallel {
        +DriveSubsystem.followTrajectory(path3())
        +ConditionalCommand(
            Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
            Superstructure.kFrontCargoFromLoadingStation,
            Superstructure.kFrontHatchFromLoadingStation
        )
    }

    +ConditionalCommand(
        Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
        IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE),
        IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
    )
}