using System;
using Hazel;

namespace Soccer
{
	// Combines the locomotion controller (turn-to-face + walk) with the
	// two-handed push action: walks the G1 to WalkTarget, and once it arrives
	// (within StandDistance), executes the same windup -> push -> retract
	// motion as G1Push. See G1WalkForward and G1Push for the isolated pieces
	// this was assembled from.
	//
	// Attach to the robot root entity (RobotControllerComponent + MujocoSceneComponent).
	// Replaces any other Script on that entity - a script component slot is
	// exclusive per entity.
	public class G1WalkThenPush : Entity
	{
		// ── Target ────────────────────────────────────────────────────────
		[Group("Target")]
		[Tooltip("The ball (or other pushable object). Leave null to stand idle.")]
		public Entity? WalkTarget;

		[Group("Target")]
		[Tooltip("Where to push the ball toward. If set, the robot walks to the point on the OPPOSITE side of the ball from this entity (StandDistance back), then turns to face this entity before pushing - so the push always drives the ball toward it. If left null, falls back to the original behaviour: walk straight up to the ball and push in whatever direction you approached from.")]
		public Entity? Goal;

		// ── Locomotion ────────────────────────────────────────────────────
		[Group("Locomotion")] [Units("m/s")] [Slider(0.0f, 1.5f)]
		[Tooltip("Forward speed while travelling. The walker policy was trained around 0.5 m/s.")]
		public float WalkSpeed = 0.8f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 2.0f)]
		[Tooltip("Stop this far (XZ) from the target, then push.")]
		public float StandDistance = 1.002f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 0.5f)]
		[Tooltip("Distance dead-zone around StandDistance - prevents creeping/oscillation at arrival.")]
		public float ArriveTolerance = 0.10f;

		[Group("Locomotion")] [Slider(0.0f, 10.0f)]
		[Tooltip("Proportional gain: yawRate = clamp(headingError * YawGain, ±MaxYawRate).")]
		public float YawGain = 4f;

		[Group("Locomotion")] [Units("rad/s")] [Slider(0.0f, 3.0f)]
		[Tooltip("Cap on commanded yaw rate. The walker destabilises if asked to turn faster than it was trained for.")]
		public float MaxYawRate = 2f;

		[Group("Locomotion")]
		[Tooltip("With a Goal set, walking straight at the approach point (opposite side of the ball from the Goal) can cut straight across the ball if the robot starts out on the Goal's side of it. When enabled, the robot instead sweeps around the ball at StandDistance - from its current bearing around to the approach bearing - producing a curving, ball-clearing path. Disable to walk straight at the approach point.")]
		public bool CurveAroundBall = true;

		[Group("Locomotion")] [Units("s")] [Slider(0.2f, 5.0f)]
		[Tooltip("If the robot makes no meaningful XZ progress for this long while trying to walk, treat it as stuck - e.g. wedged against a wall between it and a steering/approach point that's unreachable from here - and fall back to just turning to face the ball directly (dropping the curve-around steering) until roughly aligned, then resume walking normally.")]
		public float StuckTimeout = 1.5f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 0.3f)]
		[Tooltip("Minimum XZ movement within StuckTimeout to NOT be considered stuck.")]
		public float StuckMoveThreshold = 0.05f;

		// ── Facing (turn-to-face before walking) ──────────────────────────
		[Group("Facing")]
		[Tooltip("Turn in place to face the target BEFORE walking, instead of arcing toward it. The walker steers poorly from a large initial heading error.")]
		public bool TurnToFaceFirst = true;
		[Group("Facing")]
		[Tooltip("Rotator policy slot for a clean turn-in-place (the walker fights pure-yaw commands). If the robot has NO rotator policy registered, the controller falls back to turning with the walker.")]
		public uint RotatorSlotId = 2u;
		[Group("Facing")] [Units("rad")] [Slider(0.02f, 0.5f)]
		[Tooltip("Stop turning once the heading error is below this (~0.12 rad ≈ 7°).")]
		public float FaceYawTolerance = 0.224f;
		[Group("Facing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Settle the walker at a standstill before switching to the rotator, so it doesn't inherit forward momentum.")]
		public float PreTurnSettle = 0.679f;
		[Group("Facing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Ramp the turn rate in over this long to avoid a lurch on the walker→rotator handoff.")]
		public float TurnRampIn = 0.877f;
		[Group("Facing")] [Units("s")] [Slider(1.0f, 20.0f)]
		public float FaceTimeout = 4.675f;
		[Group("Facing")] [Units("m/s")] [Slider(0.0f, 0.5f)]
		[Tooltip("This robot has no rotator policy registered, so the turn-in-place falls back to the walker with a pure yaw command - which often produces no visible turn at a dead stop. This adds a small forward nudge during the fallback turn so the walker actually steps and pivots. Only used when the rotator is unavailable.")]
		public float WalkerFallbackTurnVx = 0.157f;

		// ── Push shape ────────────────────────────────────────────────────
		[Group("Push")] [Units("m")] [Slider(0.0f, 0.3f)]
		[Tooltip("How far back (opposite the push direction) the hands pull during windup.")]
		public float WindupPullBack = 0.10f;

		[Group("Push")] [Units("m")] [Slider(-0.2f, 0.3f)]
		[Tooltip("Vertical offset of the hands during windup, relative to resting position (raises them to roughly chest height).")]
		public float WindupLift = 0.3f;

		[Group("Push")] [Units("m")] [Slider(0.0f, 0.7f)]
		[Tooltip("How far forward the hands extend at full push.")]
		public float PushReach = 0.7f;

		[Group("Push")] [Units("m")] [Slider(-0.2f, 0.3f)]
		[Tooltip("Vertical offset of the hands at full push, relative to resting position.")]
		public float PushHeight = 0.3f;

		// ── Push timing ───────────────────────────────────────────────────
		[Group("Push Timing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Stand still and let the walker settle after arriving, before starting the push.")]
		public float HoldBeforePush = 0.5f;

		[Group("Push Timing")] [Units("s")] [Slider(0.1f, 1.5f)]
		public float WindupDuration = 0.4f;

		[Group("Push Timing")] [Units("s")] [Slider(0.05f, 1.0f)]
		public float PushDuration = 0.3f;

		[Group("Push Timing")] [Units("s")] [Slider(0.1f, 1.5f)]
		public float RetractDuration = 0.4f;

		[Group("Push Timing")]
		[Tooltip("Repeat the push (in place, without re-walking) after retracting.")]
		public bool Loop = false;

		[Group("Push Timing")] [Units("s")] [Slider(0.2f, 5.0f)]
		public float LoopInterval = 1.5f;

		// ── Policy wiring ─────────────────────────────────────────────────
		[Group("Policy")]
		[Tooltip("Walker policy slot id. Engine default is 1 (Hazel.PolicyIds.Walker).")]
		public uint WalkerSlotId = 1u;

		[Group("Policy")]
		[Tooltip("MuJoCo body read for the robot's position + heading. 'pelvis' for the G1.")]
		public string PelvisBodyName = "pelvis";

		private static readonly Identifier RightArmTargetPositionID   = new Identifier("RightArm_Target_Position");
		private static readonly Identifier LeftArmTargetPositionID    = new Identifier("LeftArm_Target_Position");
		private static readonly Identifier RightArmSolveOrientationID = new Identifier("RightArm_Solve_Orientation");
		private static readonly Identifier LeftArmSolveOrientationID  = new Identifier("LeftArm_Solve_Orientation");
		private static readonly Identifier DurationID                 = new Identifier("Duration");
		private static readonly Identifier StartID                    = new Identifier("Start");

		// All actuator joint names on this robot, used to build the
		// "everything except both arms" driven-joints mask.
		private static readonly string[] AllActuatorJoints =
		{
			"left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
			"right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
			"waist_yaw_joint","waist_roll_joint","waist_pitch_joint",
			"left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint",
			"left_hand_thumb_0_joint","left_hand_thumb_1_joint","left_hand_thumb_2_joint","left_hand_middle_0_joint","left_hand_middle_1_joint","left_hand_index_0_joint","left_hand_index_1_joint",
			"right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint",
			"right_hand_thumb_0_joint","right_hand_thumb_1_joint","right_hand_thumb_2_joint","right_hand_index_0_joint","right_hand_index_1_joint","right_hand_middle_0_joint","right_hand_middle_1_joint",
		};

		private static readonly string[] BothArmsJoints =
		{
			"left_shoulder_pitch_joint","left_shoulder_roll_joint","left_shoulder_yaw_joint","left_elbow_joint","left_wrist_roll_joint","left_wrist_pitch_joint","left_wrist_yaw_joint",
			"right_shoulder_pitch_joint","right_shoulder_roll_joint","right_shoulder_yaw_joint","right_elbow_joint","right_wrist_roll_joint","right_wrist_pitch_joint","right_wrist_yaw_joint",
		};

		private const uint k_SetVx      = 1u;
		private const uint k_SetVy      = 2u;
		private const uint k_SetYawRate = 3u;

		// FaceSettle/FaceTurn/Walk = locomotion (from G1WalkForward), aimed at
		// the MOVE TARGET (the ball itself, or - with a Goal set - the
		// standoff point on the far side of the ball from the Goal).
		// FaceGoalSettle/FaceGoalTurn = a second turn-in-place once standing
		// at the move target, aiming at the Goal (or the ball, with no Goal
		// set) so the push direction is correct.
		// PushSettle/Windup/Push/Retract/Cooldown/Done = the push action (from G1Push),
		// entered once that final facing step completes.
		private enum Phase { FaceSettle, FaceTurn, Walk, FaceGoalSettle, FaceGoalTurn, PushSettle, Windup, Push, Retract, Cooldown, Done }

		[HideFromEditor] private RobotControllerComponent? m_Robot;
		[HideFromEditor] private MujocoSceneComponent?     m_Mujoco;
		[HideFromEditor] private uint    m_PelvisBodyId    = uint.MaxValue;
		[HideFromEditor] private uint    m_LeftHandBodyId  = uint.MaxValue;
		[HideFromEditor] private uint    m_RightHandBodyId = uint.MaxValue;
		// Home hand offsets are stored in PELVIS-LOCAL space (captured once at
		// OnCreate) rather than as fixed world positions. The push targets are
		// re-derived from these offsets against the CURRENT pelvis pose each
		// time a move is sent - otherwise, after the robot walks away from its
		// spawn point, "home" would still point at the stale world-space spot
		// where the hands started, and the arm IK would reach for empty air.
		[HideFromEditor] private Vector3 m_HomeLeftLocalOffset;
		[HideFromEditor] private Vector3 m_HomeRightLocalOffset;
		[HideFromEditor] private Phase   m_Phase;
		[HideFromEditor] private float   m_PhaseElapsed;
		[HideFromEditor] private bool    m_RotatorActive;
		[HideFromEditor] private string[] m_FullBodyMask  = Array.Empty<string>();
		[HideFromEditor] private string[] m_ArmsFreedMask = Array.Empty<string>();
		// Current sweep angle (radians, ball-relative bearing) of the walk
		// waypoint while curving around the ball toward the approach point -
		// see CurveAroundBall / the orbit logic in Phase.Walk. Only meaningful
		// while m_OrbitAngleValid; re-initialised fresh each time Walk begins.
		[HideFromEditor] private float   m_OrbitAngle;
		[HideFromEditor] private bool    m_OrbitAngleValid;
		// Stuck detection (see StuckTimeout) - tracks the robot's own XZ
		// position over time to notice when driving forward isn't actually
		// producing movement, and m_Recovering switches Phase.Walk into
		// "just turn to face the ball" mode when that happens.
		[HideFromEditor] private Vector3 m_StuckRefPos;
		[HideFromEditor] private float   m_StuckTimer;
		[HideFromEditor] private bool    m_StuckRefPosValid;
		[HideFromEditor] private bool    m_Recovering;

		protected override void OnCreate()
		{
			m_Robot  = GetComponent<RobotControllerComponent>();
			m_Mujoco = GetComponent<MujocoSceneComponent>();
			if (m_Mujoco != null)
			{
				m_PelvisBodyId    = m_Mujoco.GetBodyID(PelvisBodyName);
				m_LeftHandBodyId  = m_Mujoco.GetBodyID("left_wrist_yaw_link");
				m_RightHandBodyId = m_Mujoco.GetBodyID("right_wrist_yaw_link");

				Vector3 pelvisPos = m_Mujoco.GetPosition(m_PelvisBodyId);
				Quaternion pelvisRot = m_Mujoco.GetOrientation(m_PelvisBodyId);
				Vector3 leftHandPos  = m_Mujoco.GetPosition(m_LeftHandBodyId);
				Vector3 rightHandPos = m_Mujoco.GetPosition(m_RightHandBodyId);
				m_HomeLeftLocalOffset  = pelvisRot.Conjugate * (leftHandPos  - pelvisPos);
				m_HomeRightLocalOffset = pelvisRot.Conjugate * (rightHandPos - pelvisPos);
			}

			m_ArmsFreedMask = Array.FindAll(AllActuatorJoints, j => Array.IndexOf(BothArmsJoints, j) < 0);
			m_FullBodyMask  = Array.Empty<string>(); // empty = unrestricted, policy drives every joint

			m_Robot?.SetPolicyActive(WalkerSlotId, true);
			m_Phase = TurnToFaceFirst ? Phase.FaceSettle : Phase.Walk;
			m_PhaseElapsed = 0f;
			m_OrbitAngleValid = false;
			m_StuckRefPosValid = false;
			m_Recovering = false;
		}

		protected override void OnUpdate(float ts)
		{
			if (m_Robot == null || m_Mujoco == null)
				return;
			m_PhaseElapsed += ts;

			if (WalkTarget == null || m_PelvisBodyId == uint.MaxValue)
			{
				Drive(WalkerSlotId, 0f, 0f);
				return;
			}

			switch (m_Phase)
			{
				case Phase.FaceSettle:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= PreTurnSettle)
					{
						EngageRotator();
						Advance(Phase.FaceTurn);
					}
					break;

				case Phase.FaceTurn:
				{
					// Face the ball itself here, not the (possibly far-side) approach
					// point - that's what the robot will initially walk toward as it
					// closes distance, before any curving in Phase.Walk kicks in.
					float err = HeadingErrorTo(WalkTarget!.Transform.WorldTranslation);
					if (Mathf.Abs(err) <= FaceYawTolerance || m_PhaseElapsed >= FaceTimeout)
					{
						DisengageRotator();
						Advance(Phase.Walk);
						break;
					}
					float ramp = TurnRampIn > 0f ? Clamp01(m_PhaseElapsed / TurnRampIn) : 1f;
					float yawRate = Mathf.Clamp(err * YawGain, -MaxYawRate, MaxYawRate) * ramp;
					uint slot = m_RotatorActive ? RotatorSlotId : WalkerSlotId;
					float vx = m_RotatorActive ? 0f : WalkerFallbackTurnVx;
					Drive(slot, vx, yawRate);
					break;
				}

				case Phase.Walk:
				{
					Vector3 pelvis = GetPelvisWorldPosition();
					Vector3 ball   = WalkTarget!.Transform.WorldTranslation;
					Vector3 arriveTarget = GetMoveTargetPosition();

					// Recovery: a prior stuck-detection (below) tripped, meaning the
					// robot wasn't making progress - most likely wedged against
					// geometry between it and an unreachable steering/approach point
					// (e.g. the far-side approach point sitting behind a wall).
					// Drop the curve-around steering entirely and just turn in place
					// to face the ball, which is always a reachable direction, until
					// roughly aligned - then hand back to normal walking.
					if (m_Recovering)
					{
						float recoverErr = HeadingErrorTo(ball);
						if (Mathf.Abs(recoverErr) <= FaceYawTolerance)
						{
							m_Recovering = false;
							m_StuckRefPos = pelvis;
							m_StuckTimer = 0f;
						}
						else
						{
							float recoverYawRate = Mathf.Clamp(recoverErr * YawGain, -MaxYawRate, MaxYawRate);
							Drive(WalkerSlotId, 0f, recoverYawRate);
							break;
						}
					}

					Vector3 steerTarget;
					float   arriveDistance;

					if (Goal != null && CurveAroundBall)
					{
						// Walking straight at arriveTarget would cut across the ball
						// whenever the robot starts out on the Goal's side of it.
						// Instead steer toward a waypoint that sweeps around the ball
						// at StandDistance, from the robot's current bearing around to
						// the approach bearing, at a rate the walker can keep up with
						// (WalkSpeed / StandDistance - the angular speed of actually
						// walking that circle). The waypoint converges onto
						// arriveTarget once the sweep catches up, so arrival is judged
						// against the true fixed approach point, not the moving waypoint.
						if (!m_OrbitAngleValid)
						{
							Vector3 rel = pelvis - ball; rel.Y = 0f;
							m_OrbitAngle = rel.Length() > 1e-4f ? DirXZToAngle(rel) : GetPelvisYaw();
							m_OrbitAngleValid = true;
						}

						float targetAngle = DirXZToAngle(GetStandoffDirection());
						float angleDiff = Mathf.WrapToPi(targetAngle - m_OrbitAngle);
						float maxRate = StandDistance > 1e-3f ? WalkSpeed / StandDistance : MaxYawRate;
						float maxStep = maxRate * ts;
						m_OrbitAngle += Mathf.Clamp(angleDiff, -maxStep, maxStep);

						steerTarget = ball + AngleToDirXZ(m_OrbitAngle) * StandDistance;
						arriveDistance = 0f;
					}
					else
					{
						m_OrbitAngleValid = false;
						steerTarget = arriveTarget; // ball itself when Goal == null
						arriveDistance = Goal != null ? 0f : StandDistance;
					}

					float dx = arriveTarget.X - pelvis.X;
					float dz = arriveTarget.Z - pelvis.Z;
					float distance = new Vector3(dx, 0f, dz).Length();

					float yawRate = Mathf.Clamp(HeadingErrorTo(steerTarget) * YawGain, -MaxYawRate, MaxYawRate);
					if (distance > arriveDistance + ArriveTolerance)
					{
						Drive(WalkerSlotId, WalkSpeed, yawRate);

						// Stuck detection: no meaningful XZ progress in StuckTimeout
						// seconds despite driving forward. Trip recovery mode (handled
						// at the top of this case) starting next frame.
						if (!m_StuckRefPosValid)
						{
							m_StuckRefPos = pelvis;
							m_StuckTimer = 0f;
							m_StuckRefPosValid = true;
						}
						else
						{
							Vector3 moved = pelvis - m_StuckRefPos; moved.Y = 0f;
							if (moved.Length() >= StuckMoveThreshold)
							{
								m_StuckRefPos = pelvis;
								m_StuckTimer = 0f;
							}
							else
							{
								m_StuckTimer += ts;
								if (m_StuckTimer >= StuckTimeout)
								{
									m_Recovering = true;
									m_StuckTimer = 0f;
								}
							}
						}
					}
					else
					{
						Drive(WalkerSlotId, 0f, 0f);
						m_OrbitAngleValid = false;
						m_StuckRefPosValid = false;
						m_Recovering = false;
						Advance(Phase.FaceGoalSettle);
					}
					break;
				}

				case Phase.FaceGoalSettle:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= PreTurnSettle)
					{
						EngageRotator();
						Advance(Phase.FaceGoalTurn);
					}
					break;

				case Phase.FaceGoalTurn:
				{
					float err = HeadingErrorTo(GetGoalFaceTarget());
					if (Mathf.Abs(err) <= FaceYawTolerance || m_PhaseElapsed >= FaceTimeout)
					{
						DisengageRotator();
						Advance(Phase.PushSettle);
						break;
					}
					float ramp = TurnRampIn > 0f ? Clamp01(m_PhaseElapsed / TurnRampIn) : 1f;
					float yawRate = Mathf.Clamp(err * YawGain, -MaxYawRate, MaxYawRate) * ramp;
					uint slot = m_RotatorActive ? RotatorSlotId : WalkerSlotId;
					float vx = m_RotatorActive ? 0f : WalkerFallbackTurnVx;
					Drive(slot, vx, yawRate);
					break;
				}

				case Phase.PushSettle:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= HoldBeforePush)
					{
						m_Robot.SetDrivenJoints(WalkerSlotId, m_ArmsFreedMask);
						Vector3 fwd = GetPelvisForward();
						Vector3 homeLeft, homeRight;
						GetHomeHandWorldPositions(out homeLeft, out homeRight);
						SendArmsMove(
							homeLeft  + Vector3.Up * WindupLift - fwd * WindupPullBack,
							homeRight + Vector3.Up * WindupLift - fwd * WindupPullBack,
							WindupDuration);
						Advance(Phase.Windup);
					}
					break;

				case Phase.Windup:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= WindupDuration)
					{
						Vector3 fwd = GetPelvisForward();
						Vector3 homeLeft, homeRight;
						GetHomeHandWorldPositions(out homeLeft, out homeRight);
						SendArmsMove(
							homeLeft  + Vector3.Up * PushHeight + fwd * PushReach,
							homeRight + Vector3.Up * PushHeight + fwd * PushReach,
							PushDuration);
						Advance(Phase.Push);
					}
					break;

				case Phase.Push:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= PushDuration)
					{
						Vector3 homeLeft, homeRight;
						GetHomeHandWorldPositions(out homeLeft, out homeRight);
						SendArmsMove(homeLeft, homeRight, RetractDuration);
						Advance(Phase.Retract);
					}
					break;

				case Phase.Retract:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= RetractDuration)
					{
						m_Robot.SetDrivenJoints(WalkerSlotId, m_FullBodyMask);
						Advance(Loop ? Phase.Cooldown : Phase.Done);
					}
					break;

				case Phase.Cooldown:
					Drive(WalkerSlotId, 0f, 0f);
					if (m_PhaseElapsed >= LoopInterval)
						Advance(Phase.PushSettle);
					break;

				case Phase.Done:
					Drive(WalkerSlotId, 0f, 0f);
					Advance(Phase.FaceSettle);
					break;
			}
		}

		// ──────────────────────────────────────────────────────────────────
		private void EngageRotator()
		{
			if (m_Robot == null)
				return;
			Drive(WalkerSlotId, 0f, 0f);
			m_Robot.SetPolicyActive(WalkerSlotId, false);
			m_Robot.SetPolicyActive(RotatorSlotId, true);
			m_RotatorActive = m_Robot.IsPolicyActive(RotatorSlotId);
			if (!m_RotatorActive)
				m_Robot.SetPolicyActive(WalkerSlotId, true);
		}

		private void DisengageRotator()
		{
			if (m_Robot == null || !m_RotatorActive)
				return;
			m_Robot.SetFloat(RotatorSlotId, k_SetYawRate, 0f);
			m_Robot.SetPolicyActive(RotatorSlotId, false);
			m_Robot.SetPolicyActive(WalkerSlotId, true);
			m_RotatorActive = false;
		}

		// Both arms share a single Duration/Start pair on this robot's motion
		// graph, so setting both targets then firing ONE trigger moves them
		// together in the same motion step.
		private void SendArmsMove(Vector3 leftWorldPos, Vector3 rightWorldPos, float duration)
		{
			if (m_Robot == null)
				return;
			m_Robot.SetInputBool(LeftArmSolveOrientationID,  false);
			m_Robot.SetInputBool(RightArmSolveOrientationID, false);
			m_Robot.SetInputVector3(LeftArmTargetPositionID,  leftWorldPos);
			m_Robot.SetInputVector3(RightArmTargetPositionID, rightWorldPos);
			m_Robot.SetInputFloat(DurationID, duration);
			m_Robot.SetInputTrigger(StartID);
		}

		private void Drive(uint slot, float vx, float yawRate)
		{
			m_Robot?.SetFloat(slot, k_SetVx,      vx);
			m_Robot?.SetFloat(slot, k_SetVy,      0f);
			m_Robot?.SetFloat(slot, k_SetYawRate, yawRate);
		}

		private void Advance(Phase next)
		{
			m_Phase = next;
			m_PhaseElapsed = 0f;
		}

		// Heading error (radians, wrapped to [-pi, pi]) to an arbitrary
		// world-space point.
		private float HeadingErrorTo(Vector3 targetWorldPos)
		{
			Vector3 pelvis = GetPelvisWorldPosition();
			float dx = targetWorldPos.X - pelvis.X;
			float dz = targetWorldPos.Z - pelvis.Z;
			return Mathf.WrapToPi(Mathf.Atan2(-dz, dx) - GetPelvisYaw());
		}

		// Convert an angle (radians, same atan2(-z,x) convention as
		// GetPelvisYaw/HeadingErrorTo) to a unit XZ direction, and back.
		// Used for the ball-orbit sweep in Phase.Walk.
		private static Vector3 AngleToDirXZ(float angle) => new Vector3(Mathf.Cos(angle), 0f, -Mathf.Sin(angle));
		private static float DirXZToAngle(Vector3 dir) => Mathf.Atan2(-dir.Z, dir.X);

		// Direction (unit, XZ) from the ball to where the robot should stand:
		// the opposite side of the ball from the Goal. Falls back to the
		// robot's current facing if no Goal is set (GetMoveTargetPosition
		// doesn't use this fallback branch, since it returns the ball
		// directly with no Goal - this only matters if some other caller
		// asks for a direction with no Goal configured).
		private Vector3 GetStandoffDirection()
		{
			Vector3 ball = WalkTarget!.Transform.WorldTranslation;
			if (Goal == null)
				return GetPelvisForward();
			Vector3 goal = Goal.Transform.WorldTranslation;
			Vector3 diff = goal - ball;
			diff.Y = 0f;
			float len = diff.Length();
			return len > 1e-5f ? -(diff / len) : GetPelvisForward();
		}

		// The point the robot walks to before pushing. With a Goal configured,
		// this is the point on the OPPOSITE side of the ball from the Goal
		// (StandDistance back along the ball->Goal line), so that pushing
		// forward from here drives the ball toward the Goal. With no Goal set,
		// this is just the ball itself - the original WalkThenPush behaviour
		// of walking up to the ball and pushing in whatever direction you
		// happened to approach from.
		private Vector3 GetMoveTargetPosition()
		{
			Vector3 ball = WalkTarget!.Transform.WorldTranslation;
			if (Goal == null)
				return ball;
			return ball + GetStandoffDirection() * StandDistance;
		}

		// Where to aim once standing at the move target, immediately before
		// pushing. Facing the Goal from the move target also means facing
		// straight through the ball (the move target, ball, and Goal are
		// collinear by construction), so the push lands toward the Goal.
		// Falls back to facing the ball if no Goal is configured.
		private Vector3 GetGoalFaceTarget()
		{
			if (Goal != null)
				return Goal.Transform.WorldTranslation;
			return WalkTarget!.Transform.WorldTranslation;
		}

		// MuJoCo positions/orientations for this robot are expressed in the
		// simulation's own local frame, which is anchored wherever the physics
		// scene starts (effectively this entity's spawn transform) - NOT at
		// the engine's world origin. WalkTarget, being a normal scene entity,
		// is expressed in world space, and the arm IK targets sent to
		// RobotControllerComponent also turn out to be consumed in world
		// space. Comparing/sending MuJoCo-local values directly (as the
		// original code did) only happened to work when this entity's
		// Transform was identity (position 0,0,0, no rotation); moving the
		// robot off-centre broke both the walk/heading math and the arm
		// targets. These helpers rotate + translate MuJoCo-local pelvis pose
		// into world space so it can be safely compared/sent as world-space.
		private Vector3 GetPelvisWorldPosition()
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
				return Transform.WorldTranslation;
			Vector3 local = m_Mujoco.GetPosition(m_PelvisBodyId);
			return Transform.WorldTranslation + Transform.WorldRotationQuat * local;
		}

		private Quaternion GetPelvisWorldOrientation()
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
				return Transform.WorldRotationQuat;
			return Transform.WorldRotationQuat * m_Mujoco.GetOrientation(m_PelvisBodyId);
		}

		// G1 pelvis heading: local +X is forward. yaw = atan2(-fwd.Z, fwd.X).
		// Rotated into world space (see GetPelvisWorldOrientation) so it can
		// be compared against the world-space bearing to WalkTarget.
		private float GetPelvisYaw()
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
				return 0f;
			Vector3 fwd = GetPelvisWorldOrientation() * new Vector3(1f, 0f, 0f);
			return Mathf.Atan2(-fwd.Z, fwd.X);
		}

		// Returns a unit horizontal forward vector (world +X rotated by
		// pelvis yaw), used to offset the arm targets during windup/push.
		private Vector3 GetPelvisForward()
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
				return Transform.WorldRotationQuat * new Vector3(1f, 0f, 0f);
			Vector3 fwd = GetPelvisWorldOrientation() * new Vector3(1f, 0f, 0f);
			fwd.Y = 0f;
			float len = fwd.Length();
			return len > 1e-5f ? fwd / len : new Vector3(1f, 0f, 0f);
		}

		// Re-derives the resting hand WORLD positions from the pelvis-local
		// offsets captured at OnCreate, against the pelvis's CURRENT world
		// pose (position + orientation, both converted from MuJoCo-local via
		// this entity's Transform). Must be called fresh at each push phase
		// transition (not cached), since the robot may have walked/turned
		// since the last push - and these feed straight into
		// RobotControllerComponent's world-space arm IK targets.
		private void GetHomeHandWorldPositions(out Vector3 homeLeft, out Vector3 homeRight)
		{
			Vector3 pelvisWorld = GetPelvisWorldPosition();
			Quaternion pelvisWorldRot = GetPelvisWorldOrientation();
			homeLeft  = pelvisWorld + pelvisWorldRot * m_HomeLeftLocalOffset;
			homeRight = pelvisWorld + pelvisWorldRot * m_HomeRightLocalOffset;
		}

		private static float Clamp01(float v) => v < 0f ? 0f : (v > 1f ? 1f : v);
	}
}