using System;
using Hazel;

namespace FlySwatter
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
		[Tooltip("Entity to walk toward. The robot faces it, walks to StandDistance, then pushes. Leave null to stand idle.")]
		public Entity? WalkTarget;

		// ── Locomotion ────────────────────────────────────────────────────
		[Group("Locomotion")] [Units("m/s")] [Slider(0.0f, 1.5f)]
		[Tooltip("Forward speed while travelling. The walker policy was trained around 0.5 m/s.")]
		public float WalkSpeed = 0.632f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 2.0f)]
		[Tooltip("Stop this far (XZ) from the target, then push.")]
		public float StandDistance = 0.70f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 0.5f)]
		[Tooltip("Distance dead-zone around StandDistance - prevents creeping/oscillation at arrival.")]
		public float ArriveTolerance = 0.10f;

		[Group("Locomotion")] [Slider(0.0f, 10.0f)]
		[Tooltip("Proportional gain: yawRate = clamp(headingError * YawGain, ±MaxYawRate).")]
		public float YawGain = 2.0f;

		[Group("Locomotion")] [Units("rad/s")] [Slider(0.0f, 3.0f)]
		[Tooltip("Cap on commanded yaw rate. The walker destabilises if asked to turn faster than it was trained for.")]
		public float MaxYawRate = 1.0f;

		// ── Facing (turn-to-face before walking) ──────────────────────────
		[Group("Facing")]
		[Tooltip("Turn in place to face the target BEFORE walking, instead of arcing toward it. The walker steers poorly from a large initial heading error.")]
		public bool TurnToFaceFirst = true;
		[Group("Facing")]
		[Tooltip("Rotator policy slot for a clean turn-in-place (the walker fights pure-yaw commands). If the robot has NO rotator policy registered, the controller falls back to turning with the walker.")]
		public uint RotatorSlotId = 2u;
		[Group("Facing")] [Units("rad")] [Slider(0.02f, 0.5f)]
		[Tooltip("Stop turning once the heading error is below this (~0.12 rad ≈ 7°).")]
		public float FaceYawTolerance = 0.12f;
		[Group("Facing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Settle the walker at a standstill before switching to the rotator, so it doesn't inherit forward momentum.")]
		public float PreTurnSettle = 0.4f;
		[Group("Facing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Ramp the turn rate in over this long to avoid a lurch on the walker→rotator handoff.")]
		public float TurnRampIn = 0.5f;
		[Group("Facing")] [Units("s")] [Slider(1.0f, 20.0f)]
		public float FaceTimeout = 8.0f;
		[Group("Facing")] [Units("m/s")] [Slider(0.0f, 0.5f)]
		[Tooltip("This robot has no rotator policy registered, so the turn-in-place falls back to the walker with a pure yaw command - which often produces no visible turn at a dead stop. This adds a small forward nudge during the fallback turn so the walker actually steps and pivots. Only used when the rotator is unavailable.")]
		public float WalkerFallbackTurnVx = 0.091f;

		// ── Push shape ────────────────────────────────────────────────────
		[Group("Push")] [Units("m")] [Slider(0.0f, 0.3f)]
		[Tooltip("How far back (opposite the push direction) the hands pull during windup.")]
		public float WindupPullBack = 0.10f;

		[Group("Push")] [Units("m")] [Slider(-0.2f, 0.3f)]
		[Tooltip("Vertical offset of the hands during windup, relative to resting position (raises them to roughly chest height).")]
		public float WindupLift = 0.15f;

		[Group("Push")] [Units("m")] [Slider(0.0f, 0.7f)]
		[Tooltip("How far forward the hands extend at full push.")]
		public float PushReach = 0.5f;

		[Group("Push")] [Units("m")] [Slider(-0.2f, 0.3f)]
		[Tooltip("Vertical offset of the hands at full push, relative to resting position.")]
		public float PushHeight = 0.10f;

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

		// FaceSettle/FaceTurn/Walk = locomotion (from G1WalkForward).
		// PushSettle/Windup/Push/Retract/Cooldown/Done = the push action (from G1Push),
		// entered once the walk arrives.
		private enum Phase { FaceSettle, FaceTurn, Walk, PushSettle, Windup, Push, Retract, Cooldown, Done }

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
					float err = HeadingErrorToTarget();
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
					Vector3 pelvis = m_Mujoco.GetPosition(m_PelvisBodyId);
					Vector3 target = WalkTarget.Transform.WorldTranslation;
					float dx = target.X - pelvis.X;
					float dz = target.Z - pelvis.Z;
					float distance = new Vector3(dx, 0f, dz).Length();

					float yawRate = Mathf.Clamp(HeadingErrorToTarget() * YawGain, -MaxYawRate, MaxYawRate);
					if (distance > StandDistance + ArriveTolerance)
					{
						Drive(WalkerSlotId, WalkSpeed, yawRate);
					}
					else
					{
						Drive(WalkerSlotId, 0f, 0f);
						Advance(Phase.PushSettle);
					}
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

		private float HeadingErrorToTarget()
		{
			if (WalkTarget == null)
				return 0f;
			Vector3 pelvis = m_Mujoco!.GetPosition(m_PelvisBodyId);
			Vector3 target = WalkTarget.Transform.WorldTranslation;
			float dx = target.X - pelvis.X;
			float dz = target.Z - pelvis.Z;
			return Mathf.WrapToPi(Mathf.Atan2(-dz, dx) - GetPelvisYaw());
		}

		// G1 pelvis heading: local +X is forward. yaw = atan2(-fwd.Z, fwd.X).
		private float GetPelvisYaw()
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
				return 0f;
			Quaternion q = m_Mujoco.GetOrientation(m_PelvisBodyId);
			Vector3 fwd = q * new Vector3(1f, 0f, 0f);
			return Mathf.Atan2(-fwd.Z, fwd.X);
		}

		// Returns a unit horizontal forward vector (local +X rotated by pelvis yaw).
		private Vector3 GetPelvisForward()
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
				return new Vector3(1f, 0f, 0f);
			Quaternion q = m_Mujoco.GetOrientation(m_PelvisBodyId);
			Vector3 fwd = q * new Vector3(1f, 0f, 0f);
			fwd.Y = 0f;
			float len = fwd.Length();
			return len > 1e-5f ? fwd / len : new Vector3(1f, 0f, 0f);
		}

		// Re-derives the resting hand world positions from the pelvis-local
		// offsets captured at OnCreate, against the pelvis's CURRENT pose.
		// Must be called fresh at each push phase transition (not cached),
		// since the robot may have walked/turned since the last push.
		private void GetHomeHandWorldPositions(out Vector3 homeLeft, out Vector3 homeRight)
		{
			if (m_Mujoco == null || m_PelvisBodyId == uint.MaxValue)
			{
				homeLeft = m_HomeLeftLocalOffset;
				homeRight = m_HomeRightLocalOffset;
				return;
			}
			Vector3 pelvisPos = m_Mujoco.GetPosition(m_PelvisBodyId);
			Quaternion pelvisRot = m_Mujoco.GetOrientation(m_PelvisBodyId);
			homeLeft  = pelvisPos + pelvisRot * m_HomeLeftLocalOffset;
			homeRight = pelvisPos + pelvisRot * m_HomeRightLocalOffset;
		}

		private static float Clamp01(float v) => v < 0f ? 0f : (v > 1f ? 1f : v);
	}
}
