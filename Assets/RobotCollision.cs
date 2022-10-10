using UnityEngine;

public class RobotCollision : MonoBehaviour
{
  public RobotMovement movement;

  void OnCollisionEnter(Collision collisionInfo) {
      if (collisionInfo.collider.tag == "Obstacle") {
          Debug.Log("We hit an obstacle: " + collisionInfo.collider.name);
      }
      if (collisionInfo.collider.tag == "Finish") {
        Debug.Log("Arrived at " + collisionInfo.collider.name + "!");
      }
  }
}
