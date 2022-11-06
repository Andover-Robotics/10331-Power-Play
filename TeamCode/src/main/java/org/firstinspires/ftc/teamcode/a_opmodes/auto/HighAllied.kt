package org.firstinspires.ftc.teamcode.a_opmodes.auto


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.TemplateDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt



class HighAllied(val opMode: LinearOpMode)  {//TODO: possibly add the TeleOpPaths functionality to this

    //TODO: reverse this


    sealed class AutoPathElement(open val name: String) {
        //class AutoPaths(val opMode: LinearOpMode) {
        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        // Command
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()



    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action {
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
        override fun onMarkerReached() = func()
    }

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action(
                "Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                        "to ${Math.toDegrees(to).roundToInt()}deg"
        ) {
            bot.roadRunner.turn(to - from)
        }
    }


    fun p2d(x: Double, y: Double, h: Double): Pose2d {
        return Pose2d(
                if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else x,
                y,
                if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) h else h
        )
    }


    class FollowTrajectory(val bot: Bot, val trajectory: Trajectory) : CommandBase() {
        override fun initialize() = bot.roadRunner.followTrajectoryAsync(trajectory)
        override fun execute() = bot.roadRunner.update()
        override fun isFinished() = !bot.roadRunner.isBusy
    }

    fun v2d(x: Double, y: Double): Vector2d {
        return Vector2d(
                if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) x else x,
                if (GlobalConfig.alliance == GlobalConfig.Alliance.RED) y else y
        )
    }


//    val open = AutoPathElement.Action("Clamp open") {
//        bot.outtake.open();
//    }


    //TODO: insert action vals here

//    val runCarousel = AutoPathElement.Action("Run carousel motor") {
//        bot.carousel.run()
//        Thread.sleep(2500)
//        bot.carousel.stop()
//    }




// to go edge while... clamp, restArm, flip,


//    fun edgeInCarousel (): AutoPathElement.Path{
//        return AutoPathElement.Path ("edge", bot.roadRunner.trajectoryBuilder(p2D(63.0, -70.0, -PI / 2))
//                //.lineToSplineHeading(p2dSame(65.5, 0.0, PI / 2))
//                .strafeLeft(60.0)
//                .addTemporalMarker(0.01, clamp.runner)
//                .addTemporalMarker(0.2, restArm.runner)
//                .addTemporalMarker(0.2,flip.runner)
//                .build())
//    }






    //                                                                  =======================================================

    //example
    //private val shootRings = AutoPathElement.Action("Shoot 3 rings") {
    //        bot.shooter.shootRings(opMode, 3, 0.8)
    //        bot.shooter.turnOff()
    //        Thread.sleep(1000)
    //    }


    //Copy from here on into AutoPathVisualizer ==============================================================================

    //TODO: Insert pose/vector vals here


    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )


    val startPose = p2d(72.0,-36.0,0.0)

    //TODO: Make Trajectories in trajectorySets

    //
    //
    //                                                                        ====================================================
//    val moveToCaroselAndLowerArm = AutoPathElement.Action("move to carosel while lowering arm :)") {
//        ParellelCommandGroup(
//                makePath(
//            "move to edge (hub to warehouse)",
//            drive.trajectoryBuilder(p2dToHub(63.0,-61.0,0.0))
//                .lineToSplineHeading(p2dSame(65.5,  0.0,PI/2)).build()
//        ),
//        clamp,
//        downArm).
//    }


    val trajectorySets: Map<TemplateDetector.PipelineResult, List<Any>> = mapOf(
            TemplateDetector.PipelineResult.LEFT to run {
                listOf(
                                drive.trajectoryBuilder(p2d(72.0,-36.0,0.0))
                                    .splineToLinearHeading(p2d(24.0, 0.0, Math.toRadians(180.0)), Math.toRadians(0.0))
                                    .splineToLinearHeading(p2d(16.0,-16.0, Math.toRadians(90.0)), Math.toRadians(0.0))
                                    .strafeTo(v2d(24.0, -72.0,))
                                    .build()
                                    //goes to farther high junction and to cones, imcomplete for sake of testing

                )
            },
            TemplateDetector.PipelineResult.MIDDLE to run {
                listOf(
                )
            },
            TemplateDetector.PipelineResult.RIGHT to run {
                listOf(//add intake when inside wrehouse and open claw when awt shipping hub
                )
            }
    )


//    fun park(result: TemplateDetector.PipelineResult): List<AutoPathElement> {
//        return run {
//            listOf(
//                    makePath(
//                            "drive into warehouse",
//                            drive.trajectoryBuilder(startPose)
//                                    .forward(72.0)
//                                    .build()
//                    )
//            )
//        }
//    }


    fun getTrajectories(a: TemplateDetector.PipelineResult): List<AutoPathElement> {
        return (trajectorySets[a] as List<AutoPathElement>?)!!
    }


}