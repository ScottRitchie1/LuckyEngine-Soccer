using System;
using Hazel;

namespace FlySwatter
{
	// Makes a G1-class humanoid perform a single-leg kicking motion using the
	// motion graph's per-leg IK (RightLeg_Target_Position / LeftLeg_Target_Position),
	// while the walker policy keeps driving the rest of the body (stance leg,
	// waist, arms) for balance - same "active limb on IK, everything else on
	// policy" trick used for arm reaches, generalised to a leg.
	//
	// CAVEAT: the walker policy here was trained for two-footed locomotion, not
	// single-leg stance. Freeing one leg for IK removes it from the policy's
	// control but the policy was never trained to balance on the other leg alone
	// - the robot may wobble or topple during the kick. This is the best
	// available approach with a walker-only policy set (no croucher/stance
	// policy on this pack); tune WindupLift/KickHeight down and durations up if
	// it's unstable.
	//
	// Attach to the robot root entity (RobotControllerComponent + MujocoSceneComponent).
	// Replaces any other Script on that entity (e.g. a walk driver) - a script
	// component slot is exclusive per entity.
	public class G1Kick : Entity
	{
		// ── Kick shape ────────────────────────────────────────────────────
		[Group("Kick")]
		[Tooltip("Kick with the right leg (true) or left leg (false).")]
		public bool KickWithRightLeg = true;

		[Group("Kick")] [Units("m")] [Slider(0.0f, 0.4f)]
		[Tooltip("How far back (opposite the kick direction) the foot pulls during windup.")]
		public float WindupPullBack = 0.15f;

		[Group("Kick")] [Units("m")] [Slider(0.0f, 0.3f)]
		[Tooltip("How high the foot lifts off the ground during windup.")]
		public float WindupLift = 0.057f;

		[Group("Kick")] [Units("m")] [Slider(0.0f, 0.6f)]
		[Tooltip("How far forward the foot swings at the top of the kick.")]
		public float KickReach = 0.225f;

		[Group("Kick")] [Units("m")] [Slider(0.0f, 0.5f)]
		[Tooltip("How high the foot is at the top of the kick.")]
		public float KickHeight = 0.173f;

		// ── Timing ────────────────────────────────────────────────────────
		[Group("Timing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Stand still and let the walker settle before starting the kick.")]
		public float HoldBeforeKick = 0.5f;

		[Group("Timing")] [Units("s")] [Slider(0.1f, 1.5f)]
		public float WindupDuration = 0.4f;

		[Group("Timing")] [Units("s")] [Slider(0.05f, 1.0f)]
		public float KickDuration = 0.25f;

		[Group("Timing")] [Units("s")] [Slider(0.1f, 1.5f)]
		public float RetractDuration = 0.4f;

		[Group("Timing")]
		[Tooltip("Repeat the kick after returning to stance.")]
		public bool Loop = true;

		[Group("Timing")] [Units("s")] [Slider(0.2f, 5.0f)]
		public float LoopInterval = 1.5f;

		// ── Policy wiring ─────────────────────────────────────────────────
		[Group("Policy")]
		public uint WalkerSlotId = 1u;

		[Group("Policy")]
		public string PelvisBodyName = "pelvis";

		private static readonly Identifier RightLegTargetPositionID    = new Identifier("RightLeg_Target_Position");
		private static readonly Identifier LeftLegTargetPositionID     = new Identifier("LeftLeg_Target_Position");
		private static readonly Identifier RightLegSolveOrientationID  = new Identifier("RightLeg_Solve_Orientation");
		private static readonly Identifier LeftLegSolveOrientationID   = new Identifier("LeftLeg_Solve_Orientation");
		private static readonly Identifier DurationID                  = new Identifier("Duration");
		private static readonly Identifier StartID                    = new Identifier("Start");

		// All actuator joint names on this robot (from MujocoSceneComponent.Actuators),
		// used to build the "everything except the kicking leg" driven-joints mask.
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

		private static readonly string[] RightLegJoints =
		{
			"right_hip_pitch_joint","right_hip_roll_joint","right_hip_yaw_joint","right_knee_joint","right_ankle_pitch_joint","right_ankle_roll_joint",
		};

		private static readonly string[] LeftLegJoints =
		{
			"left_hip_pitch_joint","left_hip_roll_joint","left_hip_yaw_joint","left_knee_joint","left_ankle_pitch_joint","left_ankle_roll_joint",
		};

		private const uint k_SetVx      = 1u;
		private const uint k_SetVy      = 2u;
		private const uint k_SetYawRate = 3u;

		private enum Phase { Settle, Windup, Kick, Retract, Cooldown, Done }

		[HideFromEditor] private RobotControllerComponent? m_Robot;
		[HideFromEditor] private MujocoSceneComponent?     m_Mujoco;
		[HideFromEditor] private uint    m_PelvisBodyId = uint.MaxValue;
		[HideFromEditor] private uint    m_FootBodyId   = uint.MaxValue;
		[HideFromEditor] private Vector3 m_HomeFootPos;
		[HideFromEditor] private Phase   m_Phase;
		[HideFromEditor] private float   m_PhaseElapsed;
		[HideFromEditor] private string[] m_FullBodyMask = Array.Empty<string>();
		[HideFromEditor] private string[] m_LegFreedMask = Array.Empty<string>();

		protected override void OnCreate()
		{
			m_Robot  = GetComponent<RobotControllerComponent>();
			m_Mujoco = GetComponent<MujocoSceneComponent>();
			if (m_Mujoco != null)
			{
				m_PelvisBodyId = m_Mujoco.GetBodyID(PelvisBodyName);
				m_FootBodyId   = m_Mujoco.GetBodyID(KickWithRightLeg ? "right_ankle_roll_link" : "left_ankle_roll_link");
				m_HomeFootPos  = m_Mujoco.GetPosition(m_FootBodyId);
			}

			string[] kickLeg = KickWithRightLeg ? RightLegJoints : LeftLegJoints;
			m_LegFreedMask = Array.FindAll(AllActuatorJoints, j => Array.IndexOf(kickLeg, j) < 0);
			m_FullBodyMask = Array.Empty<string>(); // empty = unrestricted, policy drives every joint

			m_Robot?.SetPolicyActive(WalkerSlotId, true);
			m_Phase = Phase.Settle;
			m_PhaseElapsed = 0f;
		}

		protected override void OnUpdate(float ts)
		{
			if (m_Robot == null || m_Mujoco == null || m_PelvisBodyId == uint.MaxValue || m_FootBodyId == uint.MaxValue)
				return;
			m_PhaseElapsed += ts;

			switch (m_Phase)
			{
				case Phase.Settle:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= HoldBeforeKick)
					{
						m_Robot.SetDrivenJoints(WalkerSlotId, m_LegFreedMask);
						Vector3 fwd = GetPelvisForward();
						SendLegMove(m_HomeFootPos + Vector3.Up * WindupLift - fwd * WindupPullBack, WindupDuration);
						Advance(Phase.Windup);
					}
					break;

				case Phase.Windup:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= WindupDuration)
					{
						Vector3 fwd = GetPelvisForward();
						SendLegMove(m_HomeFootPos + Vector3.Up * KickHeight + fwd * KickReach, KickDuration);
						Advance(Phase.Kick);
					}
					break;

				case Phase.Kick:
					Drive(0f, 0f);
					if (m_PhaseElapsed >= KickDuration)
					{
						SendLegMove(m_HomeFootPos, RetractDuration);
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

		private void SendLegMove(Vector3 worldPos, float duration)
		{
			if (m_Robot == null)
				return;
			Identifier posId       = KickWithRightLeg ? RightLegTargetPositionID   : LeftLegTargetPositionID;
			Identifier solveOrient = KickWithRightLeg ? RightLegSolveOrientationID : LeftLegSolveOrientationID;
			m_Robot.SetInputBool(solveOrient, false);
			m_Robot.SetInputVector3(posId, worldPos);
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
