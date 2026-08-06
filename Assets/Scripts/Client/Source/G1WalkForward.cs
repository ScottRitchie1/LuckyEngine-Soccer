using System;
using Hazel;

namespace FlySwatter
{
	// Calibrated "make the humanoid walk to a target" controller for a G1-class
	// robot. First turns in place to FACE the target (using the rotator policy,
	// which is trained to turn cleanly from a standstill), then activates the
	// walker and steers it there with a heading controller. The policy handles
	// balance + gait; this script only supplies high-level intent (vx, yawRate).
	//
	// Attach to the robot root entity (the one with RobotControllerComponent +
	// MujocoSceneComponent), assign WalkTarget, press Play.
	//
	// This is the LOCOMOTION-ONLY path. For walking up to an object and grasping
	// it, use the HumanoidPickPlaceDriver template instead.
	public class G1WalkForward : Entity
	{
		// ── Target ────────────────────────────────────────────────────────
		[Group("Target")]
		[Tooltip("Entity to walk toward. The robot faces it, then advances until within StandDistance. Leave null to stand idle.")]
		public Entity? WalkTarget;

		// ── Locomotion ────────────────────────────────────────────────────
		[Group("Locomotion")] [Units("m/s")] [Slider(0.0f, 1.5f)]
		[Tooltip("Forward speed while travelling. The walker policy was trained around 0.5 m/s.")]
		public float WalkSpeed = 0.5f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 2.0f)]
		[Tooltip("Stop this far (XZ) from the target.")]
		public float StandDistance = 0.26f;

		[Group("Locomotion")] [Units("m")] [Slider(0.0f, 0.5f)]
		[Tooltip("Distance dead-zone around StandDistance — prevents creeping/oscillation at arrival.")]
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
		[Tooltip("Rotator policy slot for a clean turn-in-place (the walker fights pure-yaw commands). Engine default 2 = Hazel.PolicyIds.Rotator. If the robot has NO rotator policy registered, the controller falls back to turning with the walker.")]
		public uint RotatorSlotId = 2u;
		[Group("Facing")] [Units("rad")] [Slider(0.02f, 0.5f)]
		[Tooltip("Stop turning once the heading error is below this (~0.12 rad ≈ 7°).")]
		public float FaceYawTolerance = 0.12f;
		[Group("Facing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Settle the walker at a standstill before switching to the rotator, so it doesn't inherit forward momentum.")]
		public float PreTurnSettle = 0.4f;
		[Group("Facing")] [Units("s")] [Slider(0.0f, 2.0f)]
		[Tooltip("Ramp the turn rate in over this long to avoid a lurch on the walker→rotator handoff.")]
		public float TurnRampIn = 0.131f;
		[Group("Facing")] [Units("s")] [Slider(1.0f, 20.0f)]
		public float FaceTimeout = 8.0f;
		[Group("Facing")] [Units("m/s")] [Slider(0.0f, 0.5f)]
		[Tooltip("This robot has no rotator policy registered, so the turn-in-place falls back to the walker with a pure yaw command - which often produces no visible turn at a dead stop. This adds a small forward nudge during the fallback turn so the walker actually steps and pivots. Only used when the rotator is unavailable.")]
		public float WalkerFallbackTurnVx = 0.18f;

		// ── Policy wiring ─────────────────────────────────────────────────
		[Group("Policy")]
		[Tooltip("Walker policy slot id. Engine default is 1 (Hazel.PolicyIds.Walker). Confirm via get_entity_properties → RobotControllerComponent → Policy Slots.")]
		public uint WalkerSlotId = 1u;

		[Group("Policy")]
		[Tooltip("MuJoCo body read for the robot's position + heading. 'pelvis' for the G1.")]
		public string PelvisBodyName = "pelvis";

		// Walker/rotator command ids (1-based descriptor order: SetVx, SetVy, SetYawRate).
		private const uint k_SetVx      = 1u;
		private const uint k_SetVy      = 2u;
		private const uint k_SetYawRate = 3u;

		// ── Phase machine ─────────────────────────────────────────────────
		private enum Phase { FaceSettle, FaceTurn, Walk }

		[HideFromEditor] private RobotControllerComponent? m_Robot;
		[HideFromEditor] private MujocoSceneComponent?     m_Mujoco;
		[HideFromEditor] private uint  m_PelvisBodyId = uint.MaxValue;
		[HideFromEditor] private Phase m_Phase;
		[HideFromEditor] private float m_PhaseElapsed;
		[HideFromEditor] private bool  m_RotatorActive;

		protected override void OnCreate()
		{
			m_Robot  = GetComponent<RobotControllerComponent>();
			m_Mujoco = GetComponent<MujocoSceneComponent>();
			m_PelvisBodyId = m_Mujoco != null ? m_Mujoco.GetBodyID(PelvisBodyName) : uint.MaxValue;

			m_Robot?.SetPolicyActive(WalkerSlotId, true);
			m_Phase = TurnToFaceFirst ? Phase.FaceSettle : Phase.Walk;
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
					float vx = distance > StandDistance + ArriveTolerance ? WalkSpeed : 0f;
					Drive(WalkerSlotId, vx, yawRate);
					break;
				}
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

		private static float Clamp01(float v) => v < 0f ? 0f : (v > 1f ? 1f : v);
	}
}
