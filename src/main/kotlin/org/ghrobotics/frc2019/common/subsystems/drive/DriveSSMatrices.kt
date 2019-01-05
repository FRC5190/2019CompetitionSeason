package org.ghrobotics.frc2019.common.subsystems.drive

import org.ghrobotics.lib.mathematics.statespace.Matrix

object DriveSSMatrices {
    val A = Matrix(
        arrayOf(
            doubleArrayOf(9.93926969e-01, -2.74732423e-05),
            doubleArrayOf(-2.74732423e-05, 9.93926969e-01)
        )
    )
    val B = Matrix(
        arrayOf(
            doubleArrayOf(4.37763528e-03, 1.98035934e-05),
            doubleArrayOf(1.98035934e-05, 4.37763528e-03)
        )
    )
    val C = Matrix(
        arrayOf(
            doubleArrayOf(1.0, 0.0),
            doubleArrayOf(0.0, 1.0)
        )
    )
    val D = Matrix(2, 2)

    val K = Matrix(
        arrayOf(
            doubleArrayOf(1.10083556e+01, -1.39732352e-03),
            doubleArrayOf(-1.39732352e-03, 1.10083556e+01)
        )
    )

    val Kff = Matrix(
        arrayOf(
            doubleArrayOf(220.44918245, -0.92751936),
            doubleArrayOf(-0.92751936, 220.44918245)
        )
    )

    val L = Matrix(
        arrayOf(
            doubleArrayOf(9.93827596e-01, -2.74704960e-05),
            doubleArrayOf(-2.74704960e-05, 9.93827596e-01)
        )
    )

    val initialState = Matrix(
        arrayOf(
            doubleArrayOf(0.0),
            doubleArrayOf(0.0)
        )
    )
}