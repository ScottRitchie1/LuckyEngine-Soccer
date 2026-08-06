using System;
using Hazel;

namespace Soccer
{
	public class Ball : Entity
	{
	    public int RedScore = 0;
	    public int BlueScore = 0;
	    
	    public float RedGoalDistance = -7.5f;
	    public float BlueGoalDistance = 7.5f;
	    
	    public Entity? scoreText;
	    
	    
	    
	    internal Vector3 ballStart;
	
		/// <summary>
		/// OnCreate is called once when the Entity that this script is attached to
		/// is instantiated in the scene at runtime
		/// </summary>
		protected override void OnCreate()
		{
		    ballStart = Transform.WorldTranslation;
		}

		/// <summary>
		/// OnUpdate is called once every frame while this script is active in the scene
		/// </summary>
		protected override void OnUpdate(float deltaTime)
		{
		    bool scored = false;
		
		    if(Transform.WorldTranslation.X < RedGoalDistance)
		    {
		        BlueScore++;
		        scored = true;
		    }
		    
		    if(Transform.WorldTranslation.X > BlueGoalDistance)
		    {
		        RedScore++;
		        scored = true;
		    }
		    
		    if(scored)
		    {
		        Transform.WorldTranslation = ballStart;
		        
		        TextComponent textC = scoreText.GetComponent<TextComponent>();
		        
		        if(textC != null){
		           textC.Text = $"Blue:{BlueScore}\nRed:{RedScore}}}";
		        }
		    }
		}

	}
}
