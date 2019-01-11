@file:Suppress("MatchingDeclarationName")

package org.ghrobotics.frc2019.robot.vision

import com.github.salomonbrys.kotson.fromJson
import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonParseException
import edu.wpi.first.wpilibj.SerialPort
import edu.wpi.first.wpilibj.Timer
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.SendChannel
import kotlinx.coroutines.channels.produce
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.*
import kotlin.coroutines.CoroutineContext

fun CoroutineScope.createJeVois(
    port: SerialPort.Port,
    visionDataChannel: SendChannel<VisionData>
) = JeVois(port, visionDataChannel, this.coroutineContext)

class JeVois(
    private val port: SerialPort.Port,
    private val visionDataChannel: SendChannel<VisionData>,
    parent: CoroutineContext
) {

    private val job = Job(parent[Job])
    private val scope = CoroutineScope(parent + job)

    init {
        scope.launch {
            while (isActive) {
                try {
                    coroutineScope {
                        run()
                    }
                } catch (@Suppress("TooGenericExceptionCaught") e: Throwable) {
                    e.printStackTrace()
                    println("[JeVois-${port.name}] Failure in connection... retrying in 5 seconds...")
                    delay(5000)
                }
            }
        }
    }

    private suspend fun CoroutineScope.run() {
        val serialPort = SerialPort(115200, port)
        val readChannel = spiReadToChannel(serialPort)

        serialPort.writeString("setpar serout USB\n")
        serialPort.writeString("date 0101000070\n")
        val fpgaOffset = Timer.getFPGATimestamp()

        serialPort.writeString("streamon\n")

        for (dataString in readChannel) {
            if (!dataString.startsWith('{')) continue
            try {
                val jsonData = kJevoisGson.fromJson<JsonObject>(dataString)

                val timestamp = (jsonData["Epoch Time"].asDouble + fpgaOffset).second
                val contours = jsonData["Targets"].asJsonArray
                    .asSequence()
                    .filterIsInstance<JsonObject>()
                    .map { contourData ->
                        VisionTarget(
                            contourData["angle"].asDouble.degree,
                            contourData["distance"].asDouble.inch
                        )
                    }.toList()

                visionDataChannel.send(VisionData(timestamp, contours))
            } catch (e: JsonParseException) {
                e.printStackTrace()
                println("[JeVois-${port.name}] Got Invalid Data: $dataString")
            }
        }
    }

    fun free() {
        job.cancel()
    }

    companion object {
        private val kJevoisGson = Gson()
    }

}

data class VisionData(
    val timestamp: Time,
    val targets: List<VisionTarget>
)

data class VisionTarget(
    val angle: Rotation2d,
    val distance: Length
) {
    val cameraRelativePose = Pose2d(Translation2d(distance, angle), 0.degree)
}

fun CoroutineScope.spiReadToChannel(
    serialPort: SerialPort
) = produce {
    val messageBuffer = StringBuilder()
    while (isActive) {
        for (byteReceived in serialPort.read(serialPort.bytesReceived)) {
            val charReceived = byteReceived.toChar()
            if (charReceived == '\n') {
                send(messageBuffer.toString())
                messageBuffer.clear()
            } else {
                messageBuffer.append(charReceived)
            }
        }
        delay(1)
    }
}