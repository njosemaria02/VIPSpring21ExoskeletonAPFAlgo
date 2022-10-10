using UnityEngine;

public class RobotMovement : MonoBehaviour
{
  public Rigidbody rb;

  public float forwardForce = 25f;
  public float sidewaysForce = 25f;

  // Start is called before the first frame update
  void Start() {

  }

  // Update is called once per frame
  void FixedUpdate() {
    // up
    if (Input.GetKey("w")) {
      rb.AddForce(0, 0, forwardForce);
    }

    // down
    if (Input.GetKey("s")) {
      rb.AddForce(0, 0, -forwardForce);
    }

    // left
    if (Input.GetKey("d")) {
      rb.AddForce(sidewaysForce, 0, 0);
    }

    // right
    if (Input.GetKey("a")) {
      rb.AddForce(-sidewaysForce, 0, 0);
    }

  }
}
