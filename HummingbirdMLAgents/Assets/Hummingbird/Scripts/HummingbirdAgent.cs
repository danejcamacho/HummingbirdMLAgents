using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;

/// <summary>
/// A hummingbird Machine Learning Agent
/// </summary>
public class HummingbirdAgent : Agent
{
    [Tooltip("Force to apply when moving")]
    public float moveForce = 2f;

    [Tooltip("Speed to pitch up or down")]
    public float pitchSpeed = 100f;

    [Tooltip("Speed to rotate around the up acxis")]
    public float yawSpeed = 100f;

    [Tooltip("Transform at the tip of the beak")]
    public Transform beakTip;

    [Tooltip("The agent's camera")]
    public Camera agentCamera;

    [Tooltip("Whether this is training mode or gameplay mode")]
    public bool trainingMode;

    // The rigidbody fo the agent
    new private Rigidbody rigidbody;

    // The flowwer area that the agent is in
    private FlowerArea flowerArea;

    //The nearest flower to the agent
    private Flower nearestFlower;

    //Allows for smoother pitch changes
    private float smoothPitchChange = 0f;

    // Allows for smoother yaw changes
    private float smoothYawChange = 0f;

    // The maximum  angle that the bird can pitch up or down
    private const float MaxPitchAngle = 80f;

    // The maximum  distnace from the beak tip to accept nectar collision
    private const float BeakTipRadius = 0.008f;

    // Whether the agent is frozen (intentionally not flying)
    private bool frozen = false;


    /// <summary>
    /// The amount of nectar the agent has obtained this episode
    /// </summary>
    public float NectarObtained { get; private set;}

    /// <summary>
    /// Initialize the agent
    /// </summary>
    public override void Initialize()
    {
        rigidbody = GetComponent<Rigidbody>();
        flowerArea = GetComponentInParent<FlowerArea>();

        // If not traingin mode, no max step, play forever
        if(!trainingMode) MaxStep = 0;

    }
    /// <summary>
    /// Reset the agent when an episode begins
    /// </summary>
    public override void OnEpisodeBegin()
    {
        if (trainingMode){
            //only reset flowers in training when there is one agent per area
            flowerArea.ResetFlowers();
        }

        // Reset nectar obtained
        NectarObtained = 0f;

        // Zero out velocities tso that movement stops before a new episode begins
        rigidbody.velocity = Vector3.zero;
        rigidbody.angularVelocity = Vector3.zero;

        // Default to spawing in front of a flower
        bool inFrontOfFlower = true;
        if (trainingMode) {
            // spawn in front of flwoer 50% of the time in traingin
            inFrontOfFlower = UnityEngine.Random.value > .5f;
        }

        // Move the agent to a new random posiiton
        MoveToSafeRandomPosition(inFrontOfFlower);

        // Recalculate the nearest flower now that agent moved
        UpdateNearestFlower();
    }

    /// <summary>
    /// Called when an action is received from either the player input or the neural network
    /// actions.ContinuousActions[i] represents:
    /// Index 0: move vector x (+1 = right, -1 = left)
    /// Index 1: move vector y (+1 = up, -1 = down)
    /// Index 2: move vector z (+1 = forward, -1 = backward)
    /// Index 3: pitch angle (+1 = pitch up, -1 = pitch down)
    /// Index 4: yaw angle (+1 = turn right, -1 = turn left)
    /// 
    /// </summary>
    /// <param name="actions"></param>
    public override void OnActionReceived(ActionBuffers actions)
    {
        //Don't take actions if frozen
        if (frozen) return;

        // Calculate movement vector
        Vector3 move = new Vector3(actions.ContinuousActions[0], actions.ContinuousActions[1], actions.ContinuousActions[2]);

        // Add force in the direction of the move vector
        rigidbody.AddForce(move * moveForce);

        // Get current rotation
        Vector3 rotationVector = transform.rotation.eulerAngles;

        // Calculate pitch and yaw rotation
        float pitchChange = actions.ContinuousActions[3];
        float yawChange = actions.ContinuousActions[4];

        // Calculate smooth rotation changes
        smoothPitchChange = Mathf.MoveTowards(smoothPitchChange, pitchChange, 2f * Time.fixedDeltaTime);
        smoothYawChange = Mathf.MoveTowards(smoothYawChange, yawChange, 2f * Time.fixedDeltaTime);

        // Calculate new pitch and yaw based on smoothed values
        // Clamp pitch to avoid flipping upside down
        float pitch = rotationVector.x + smoothPitchChange * Time.fixedDeltaTime * pitchSpeed;
        if (pitch > 180f) pitch -= 360f;
        pitch = Mathf.Clamp(pitch, -MaxPitchAngle, MaxPitchAngle);

        float yaw = rotationVector.y + smoothYawChange * Time.fixedDeltaTime * yawSpeed;

        // Apply the new rotation
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);

    }


    /// <summary>
    /// Collect vector observations from the environment
    /// </summary>
    /// <param name="sensor"> The vector sensor </param>
    public override void CollectObservations(VectorSensor sensor)
    {
        // If nearestFlower is null, observe an empty array and return early
        if (nearestFlower == null){
            sensor.AddObservation(new float[10]);
            return;
        }

        // Observe agent's local rotation (4 observations)
        sensor.AddObservation(transform.localRotation.normalized);

        // Get a vector to the nearest flower from beak tip
        Vector3 toFlower = nearestFlower.FlowerCenterPosition - beakTip.position;

        // Observe a normalized vector to the nearest flower (3 observations)
        sensor.AddObservation(toFlower.normalized);

        // Observe a dot product that indicates whether the beak is in front of the flower (1 observation)
        // +1 means directly in front, -1 means directly behind
        sensor.AddObservation(Vector3.Dot(toFlower.normalized, -nearestFlower.FlowerUpVector.normalized));

        // Observe a dot product that indicates whether the beak is pointing at the flower (1 observation)
        // +1 means directly pointing at flower, -1 means directly away
        sensor.AddObservation(Vector3.Dot(beakTip.forward.normalized, -nearestFlower.FlowerUpVector.normalized));

        // Observe the relative distance from the beak tip to the flower (1 observation)
        sensor.AddObservation(toFlower.magnitude / FlowerArea.AreaDiameter);

        // 10 total observations
    }


    /// <summary>
    /// When behavior type is set to "Heuristic Only" on the agent's Behavior Parameters,
    /// this function will be called. Its return values will be fed into the
    /// <see cref="OnActionReceived(ActionBuffer)"/> function.
    /// </summary>
    /// <param name="actionsOut">an output action array</param>
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //Create placeholders for the actions
        Vector3 forward = Vector3.zero;
        Vector3 left = Vector3.zero;
        Vector3 up = Vector3.zero;
        float pitch = 0f;
        float yaw = 0f;

        // Convert keyboard inputs to movement and rotation

        // Forward/back
        if (Input.GetKey(KeyCode.W)){
            forward = transform.forward;
        } else if (Input.GetKey(KeyCode.S)){
            forward = -transform.forward;
        }

        // Left/right
        if (Input.GetKey(KeyCode.A)){
            left = -transform.right;
        } else if (Input.GetKey(KeyCode.D)){
            left = transform.right;
        }

        // Up/down
        if (Input.GetKey(KeyCode.LeftShift)){
            up = -transform.up;
        } else if (Input.GetKey(KeyCode.Space)){
            up = transform.up;
        }

        // Pitch up/down
        if (Input.GetKey(KeyCode.UpArrow)){
            pitch = 1f;
        } else if (Input.GetKey(KeyCode.DownArrow)){
            pitch = -1f;
        }

        // Yaw left/right
        if (Input.GetKey(KeyCode.RightArrow)){
            yaw = 1f;
        } else if (Input.GetKey(KeyCode.LeftArrow)){
            yaw = -1f;
        }

        // Combine the movement vectors and normalize
        Vector3 combined = (forward + left + up).normalized;
    
        //add the 3 movement values, pitch, and yaw to the actionsOut
        actionsOut.ContinuousActions.Array[0] = combined.x;
        actionsOut.ContinuousActions.Array[1] = combined.y;
        actionsOut.ContinuousActions.Array[2] = combined.z;
        actionsOut.ContinuousActions.Array[3] = pitch;
        actionsOut.ContinuousActions.Array[4] = yaw;



    }
    /// <summary>
    /// Prevent the agent from moving and taking actions
    /// </summary>
    public void FreezeAgent()
    {
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supported in training");
        frozen = true;
        rigidbody.Sleep();
    }

    /// <summary>
    /// Resume agent movement and actions
    /// </summary>
    public void UnfreezeAgent()
    {
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supported in training");
        frozen = false;
        rigidbody.WakeUp();
    }

    /// <summary>
    /// Move the agent to a safe random postioin that doesnt collide with anything
    /// if infront of flower also point beak at flower
    /// </summary>
    /// <param name="inFrontOfFlower"> Whether to choose a spot in front of flower</param>
    private void MoveToSafeRandomPosition(bool inFrontOfFlower)
    {
        bool safePositionFound = false;
        int attemptsRemaining = 100;
        Vector3 potentialPosition = Vector3.zero;
        Quaternion potentialRotation = new Quaternion();

        // Loop until a safe position is found or we run out of attempts
        while(safePositionFound == false && attemptsRemaining > 0){
            attemptsRemaining--;
            if (inFrontOfFlower){
                // Pick a random flower
                Flower randomFlower = flowerArea.Flowers[UnityEngine.Random.Range(0, flowerArea.Flowers.Count)];

                // Position 10 to 20 cm in front of the flower
                float distanceFromFlower = UnityEngine.Random.Range(.1f, .2f);
                potentialPosition = randomFlower.transform.position + randomFlower.FlowerUpVector * distanceFromFlower;

                // Point beak at flower (bird's head is center of transform)
                Vector3 toFlower = randomFlower.FlowerCenterPosition - potentialPosition;
                potentialRotation = Quaternion.LookRotation(toFlower, Vector3.up);
            } else {
                //Pick a random height from the ground
                float height = UnityEngine.Random.Range(1.2f, 2.5f);

                // Pick a random radius from the center of the area to spawn
                float radius = UnityEngine.Random.Range(2f, 7f);

                // Pick a random direction rotated around the y axis
                Quaternion direction = Quaternion.Euler(0f, UnityEngine.Random.Range(-180f, 180f), 0f);

                // Combine the height, radius, and direction to pick a potential position

                potentialPosition = flowerArea.transform.position + Vector3.up * height + direction * Vector3.forward * radius;

                //Choose and set random starting pitch and yaw
                float pitch = UnityEngine.Random.Range(-60f, 60f);
                float yaw = UnityEngine.Random.Range(-180f, 180f);

                potentialRotation = Quaternion.Euler(pitch, yaw, 0f);

            }

            // Check to see if the agent collides with anything
            Collider[] colliders = Physics.OverlapSphere(potentialPosition, 0.05f);
            safePositionFound = colliders.Length == 0;

        }
        Debug.Assert(safePositionFound, "Could not find a safe position to spawn");

        // set position and rotation
        transform.position = potentialPosition;
        transform.rotation = potentialRotation;
    }

    /// <summary>
    /// Called when an action is received from either the player input or the neural network
    /// </summary>
    /// <exception cref="NotImplementedException"></exception>
        private void UpdateNearestFlower()
    {
        foreach(Flower flower in flowerArea.Flowers){
            if (nearestFlower == null && flower.HasNectar){
                nearestFlower = flower;
            } else if (flower.HasNectar){
                // Calculate distance to this flower and distance to the current nearest flower
                float distanceToFlower = Vector3.Distance(flower.FlowerCenterPosition, beakTip.position);
                float distanceToCurrentNearestFlower = Vector3.Distance(nearestFlower.FlowerCenterPosition, beakTip.position);

                // if current nearest flower is empty or this flower is closer, update the nearest flower
                if (!nearestFlower.HasNectar || distanceToFlower < distanceToCurrentNearestFlower){
                    nearestFlower = flower;
                }
            }
        }
    }

    /// <summary>
    /// Called when the agent collides with trigger
    /// </summary>
    /// <param name="other"></param>
    private void OnTriggerEnter(Collider other)
    {
        TriggerEnterOrStay(other);   
    }


    /// <summary>
    /// Called when the agent stays in a trigger
    /// </summary>
    /// <param name="other"></param>
    private void OnTriggerStay(Collider other)
    {
        TriggerEnterOrStay(other);
    }


    /// <summary>
    /// Called when the agent enters or stays in a trigger collider
    /// </summary>
    /// <param name="collider"></param>
    private void TriggerEnterOrStay(Collider collider)
    {
        if (collider.CompareTag("nectar")){
            Vector3 closestPointToBeakTip = collider.ClosestPoint(beakTip.position);

            // Check if the closest point is close to the beak tip
            // NOTE: a collision with anything but the beak tip should not count
            if (Vector3.Distance(beakTip.position, closestPointToBeakTip) < BeakTipRadius){
                // Look up the flower for this nectar collider
                Flower flower = flowerArea.GetFlowerFromNectar(collider);

                // Attempt to take nectar from the flower
                // note: this is per fixed timestep meaning that the agent can only collect nectar once per .02seconds 50x a second
                float nectarReceived = flower.Feed(.01f);

                // Keep track of nectar obtained
                NectarObtained += nectarReceived;

                if (trainingMode){
                    //Calculate reward for getting nectar
                    float bonus = .02f * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, -nearestFlower.FlowerUpVector.normalized));
                    AddReward(.01f + bonus);

                }

                // If flower is empty, update nearest flower
                if (!flower.HasNectar){
                    UpdateNearestFlower();
                }

            }
        }
    }
    

    /// <summary>
    /// Called when the agent collides with something solid
    /// </summary>
    /// <param name="collision">The collision info</param>
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("boundary")){
            if (trainingMode){
                AddReward(-.5f);
            }
        }
    }

    /// <summary>
    /// Called every frame
    /// </summary>
    private void Update()
    {
        // Draw a line from beak tip to the nearest flower
        if (nearestFlower != null){
            Debug.DrawLine(beakTip.position, nearestFlower.FlowerCenterPosition, Color.green);
        }
    }

    /// <summary>
    /// Called every .02 seconds
    /// </summary>
    private void FixedUpdate()
    {   
        // fixes bug if nectar stolen by player
        if(nearestFlower != null && !nearestFlower.HasNectar){
            UpdateNearestFlower();
        }
    }
}
