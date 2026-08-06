using System;
using Hazel;

namespace FlySwatter
{
	// Makes a G1-class humanoid perform a two-handed forward push using the
	// motion graph's per-arm IK (RightArm_Target_Position / LeftArm_Target_Position),
	// while the walker policy keeps driving the legs/waist for balance - same
	// "active limb(s) on IK, everything else on policy" trick used elsewhere,
	// applied to both arms at once.
	//
	// This robot's motion graph shares a single Duration/Start pair across ALL
	// limb targets (there's only one "Start" trigger and one "Duration" input,
	// not one per limb) - so both arms are driven to their new targets by ONE
	// SetInputTrigger call per phase, moving them together in the same motion.
	//
	// Attach to the robot root entity (RobotControllerComponent + MujocoSceneComponent).
	// Replaces any other Script on that entity - a script component slot is
	// exclusive per entity.
	public class G1Push : Entity
	{
		// ── Push shape ────────────────────────────────────────────────────
		[Group("Push")] [Units("m")] [Slider(0.0f, 0.3f)]
		[Tooltip("How far back (opposite the push direction) the hands pull during windup.")]
		public float WindupPullBack = 0.10f;

		[Group("Push")] [Units("m")] [Slider(-0.2f, 0.3f)]
		[Tooltip("Vertical offset of the hands during windup, relative to resting position (raises them to roughly chest height).")]
		public float WindupLift = 0.15f;

		[Group("Push")] [Units("m")] [Slider(0.0f, 0.7f)]
		[Tooltip("How far forward the hands extend at full push.")]
		public float PushReach = 0.40f;

		[Group("Push")] [Units("m")] [Slider(-0.2f, 0.3f)]
		[Tooltip("Vertical offset of the hands at full push, relative to resting position.")]
		public float PushHeight = 0.10f;

		// ── Timing ────────────────────────────────────────────────────────
		[Group("Timing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Stand still and let the walker settle before starting the push.")]
		public float HoldBeforePush = 0.5f;

		[Group("Timing")] [Units("s")] [Slider(0.1f, 1.5f)]
		public float WindupDuration = 0.4f;

		[Group("Timing")] [Units("s")] [Slider(0.05f, 1.0f)]
		public float PushDuration = 0.3f;

		[Group("Timing")] [Units("s")] [Slider(0.1f, 1.5f)]
		public float RetractDuration = 0.4f;

		[Group("Timing")]
		[Tooltip("Repeat the push after retracting.")]
		public bool Loop = false;

		[Group("Timing")] [Units("s")] [Slider(0.2f, 5.0f)]
		public float LoopInterval = 1.5f;

		// ── Policy wiring ─────────────────────────────────────────────────
		[Group("Policy")]
		public uint WalkerSlotId = 1u;

		[Group("Policy")]
		public string PelvisBodyName = "pelvis";

		private static readonly Identifier RightArmTargetPositionID   = new Identifier("RightArm_Target_Position");
		private static readonly Identifier LeftArmTargetPositionID    = new Identifier("LeftArm_Target_Position");
		private static readonly Identifier RightArmSolveOrientationID = new Identifier("RightArm_Solve_Orientation");
		private static readonly Identifier LeftArmSolveOrientationID  = new Identifier("LeftArm_Solve_Orientation");
		private static readonly Identifier DurationID                 = new Identifier("Duration");
		private static readonly Identifier StartID                    = new Identifier("Start");

		// All actuator joint names on this robot (from MujocoSceneComponent.Actuators),
		// used to build the "everything except both arms" driven-joints mask.
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

		private enum Phase { Settle, Windup, Push, Retract, Cooldown, Done }

		[HideFromEditor] private RobotControllerComponent? m_Robot;
		[HideFromEditor] private MujocoSceneComponent?     m_Mujoco;
		[HideFromEditor] private uint    m_PelvisBodyId    = uint.MaxValue;
		[HideFromEditor] private uint    m_LeftHandBodyId  = uint.MaxValue;
		[HideFromEditor] private uint    m_RightHandBodyId = uint.MaxValue;
		[HideFromEditor] private Vector3 m_HomeLeftHandPos;
		[HideFromEditor] private Vector3 m_HomeRightHandPos;
		[HideFromEditor] private Phase   m_Phase;
		[HideFromEditor] private float   m_PhaseElapsed;
		[HideFromEditor] private string[] m_FullBodyMask = Array.Empty<string>();
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
				m_HomeLeftHandPos  = m_Mujoco.GetPosition(m_LeftHandBodyId);
				m_HomeRightHandPos = m_Mujoco.GetPosition(m_RightHandBodyId);
			}

			m_ArmsFreedMask = Array.FindAll(AllActuatorJoints, j => Array.IndexOf(BothArmsJoints, j) < 0);
			m_FullBodyMask  = Array.Empty<string>(); // empty = unrestricted, policy drives every joint

			m_Robot?.SetPolicyActive(WalkerSlotId, true);
			m_Phase = Phase.Settle;
			m_PhaseElapsed = 0f;
		}

		protected override void OnUpdate(float ts)
		{
			if (m_Robot == null || m_Mujoco == null || m_PelvisBodyId == uint.MaxValue
				|| m_LeftHandBodyId == uint.MaxValue || m_RightHandBodyId == uint.MaxValue)
				return;
			m_PhaseElapsed += ts;

			switch (m_Phase)
			{
				case Phase.Settle:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= HoldBeforePush)
					{
						m_Robot.SetDrivenJoints(WalkerSlotId, m_ArmsFreedMask);
						Vector3 fwd = GetPelvisForward();
						SendArmsMove(
							m_HomeLeftHandPos  + Vector3.Up * WindupLift - fwd * WindupPullBack,
							m_HomeRightHandPos + Vector3.Up * WindupLift - fwd * WindupPullBack,
							WindupDuration);
						Advance(Phase.Windup);
					}
					break;

				case Phase.Windup:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= WindupDuration)
					{
						Vector3 fwd = GetPelvisForward();
						SendArmsMove(
							m_HomeLeftHandPos  + Vector3.Up * PushHeight + fwd * PushReach,
							m_HomeRightHandPos + Vector3.Up * PushHeight + fwd * PushReach,
							PushDuration);
						Advance(Phase.Push);
					}
					break;

				case Phase.Push:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= PushDuration)
					{
						SendArmsMove(m_HomeLeftHandPos, m_HomeRightHandPos, RetractDuration);
						Advance(Phase.Retract);
					}
					break;

				case Phase.Retract:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= RetractDuration)
					{
						m_Robot.SetDrivenJoints(WalkerSlotId, m_FullBodyMask);
						Advance(Loop ? Phase.Cooldown : Phase.Done);
					}
					break;

				case Phase.Cooldown:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= LoopInterval)
						Advance(Phase.Settle);
					break;

				case Phase.Done:
					Drive(0f, 0f);
					break;
			}
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

		private void Drive(float vx, float yawRate)
		{
			m_Robot?.SetFloat(WalkerSlotId, k_SetVx,      vx);
			m_Robot?.SetFloat(WalkerSlotId, k_SetVy,      0f);
			m_Robot?.SetFloat(WalkerSlotId, k_SetYawRate, yawRate);
		}

		private void Advance(Phase next)
		{
			m_Phase = next;
			m_PhaseElapsed = 0f;
		}

		// G1 pelvis heading: local +X is forward. Returns a unit horizontal vector.
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
	}
}
