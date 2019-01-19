package org.ghrobotics.frc2019.subsystems

import org.ghrobotics.frc2019.Constants
import org.ghrobotics.lib.commands.FalconCommand
import org.ghrobotics.lib.mathematics.units.Length
import org.ghrobotics.lib.mathematics.units.Rotation2d

object Superstructure {

    fun goToHeightWithArmAngle(heightAboveGround: Length, armAngle: Rotation2d) = set(
        heightAboveGround - (Constants.kArmLength * armAngle.sin) - Constants.kElevatorHeightFromGround,
        armAngle
    )

    private fun set(elevatorHeightFromZero: Length, armAngle: Rotation2d): FalconCommand {
        TODO("very nice state stuff here")
    }

}