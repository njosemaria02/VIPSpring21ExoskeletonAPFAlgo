/*
   @author Nathaly Jose-Maria

   APF implementation for Unity visualization based off of the following Python implementation:

*/

using UnityEngine;
using System.Collections.Generic;

public class APF : MonoBehaviour
{
    // constants used throughout the code
    private float KP = 5.0F;
    private float ETA = 100.0F;
    private float AREA_WIDTH = 30.0f;
    private int OSCILLATIONS_DETECTION_LENGTH = 3;

    // Unity objects incorporated into implementation
    public GameObject robot;
    public GameObject goal;
    public GameObject[] obstacles;

    // private instance variables to hold return information since C# doesn't support multiple returns
    private float[,] Fpmap;
    private float Fminx;
    private float Fminy;
    private List<float> Frx = new List<float>();
    private List<float> Fry = new List<float>();

    // <summary>
    // everything in the main method of the Python implementation found here
    // </summary>
    
    // Notes:
    // - to edit position, have to add a vector to current value
    // - Start is called before the first frame update
    public void Start()
    {
        Debug.Log("potential_field_planning start");

        // obstacles list initialization -- finds anything in Unity with an "Obstacle" tag
        obstacles = GameObject.FindGameObjectsWithTag("Obstacle");

        // variable initialization -- gets starting x,y position of robot in Unity
        Vector3 rInitPos = robot.transform.position;
        float sx = rInitPos.x;
        float sy = rInitPos.z;
        // hardcoded values for testing:
        // float sx = 0.0F;
        // float sy = 10.0F;

        // gets starting x,y position of goal in Unity
        Vector3 gInitPos = goal.transform.position;
        float gx = gInitPos.x;
        float gy = gInitPos.z;
        // float gx = 30.0F;
        // float gy = 30.0F;

        float gridSize = .5F;
        // float gridSize = .5F;

        // gets radius of robot in Unity
        Vector3 rScale = robot.transform.localScale;
        float rRadius = rScale.x / 2.0f;
        // float rRadius = 5.0F;

        // gets x,y position of all obstacles within Unity
        float[] ox = {obstacles[0].transform.position.x, obstacles[1].transform.position.x};
        float[] oy = {obstacles[0].transform.position.z, obstacles[1].transform.position.z};
        // hardcoded values for testing:
        // float[] ox = {15.0F, 5.0F, 20.0F, 25.0F};
        // float[] oy = {25.0F, 15.0F, 26.0F, 25.0F};

        potentialFieldPlanning(sx, sy, gx, gy, ox, oy, gridSize, rRadius);
        // return x[], y[]

    }

    // <summmary>
    // Method that finds calculates a potential field based on the visualization inputs.
    // </summary>
    private void calcPotentialField(float sx, float sy, float gx, float gy, float[] ox, float[] oy, float gridSize, float rRadius) {
        float minx = Mathf.Min(Mathf.Min(ox), sx, gx) - AREA_WIDTH / 2.0f;
        float miny = Mathf.Min(Mathf.Min(oy), sy, gy) - AREA_WIDTH / 2.0f;
        float maxx = Mathf.Max(Mathf.Max(ox), sx, gx) + AREA_WIDTH / 2.0f;
        float maxy = Mathf.Max(Mathf.Max(oy), sy, gy) + AREA_WIDTH / 2.0f;
        // creates dimensions of the Python visual -- don't know how to incorporate into Unity visual though
        int xw = (int)(Mathf.Round((maxx - minx) / gridSize));
        int yw = (int) (Mathf.Round((maxy - miny) / gridSize));

        // initializes potential map with completely zero values for entire grid
        float[,] pmap = new float[xw, yw];
        for(int i = 0; i < xw; i++) {
            for(int j = 0; j < yw; j++) {
                pmap[i, j] = 0F;
            }
        }

        // for each x array within the 2d array, find the sum potential force of that square in the grid
        // sum meaning attractive potential + repulsive potential
        for(int ix = 0; ix < xw; ix++) {
            float x = ix * gridSize + minx;

            for(int iy = 0; iy < yw; iy++) {
                float y = iy * gridSize + miny;
                float ug = calcAttractivePotential(x, y, gx, gy);
                float uo = calcRepulsivePotential(x, y, ox, oy, rRadius);
                float uf = ug + uo;
                pmap[ix, iy] = uf;
            }
        }

        // 'return' the values by updating the private instance variables above
        Fpmap = pmap;
        Fminx = minx;
        Fminy = miny;
    }

    // <summmary>
    // Method that calculates the attractive potential of a coordinate in the potential field.
    // </summary>
    private float calcAttractivePotential(float x, float  y, float gx, float gy) {
        return 0.5F * KP * hypo(x - gx, y - gy);
    }

    // <summmary>
    // Method that calculates the repulsive potential of a coordinate in the potential field.
    // </summary>
    private float calcRepulsivePotential(float x, float  y, float[] ox, float[] oy, float rRadius) {
        int minid = -1;
        float dmin = Mathf.Infinity;

        for (int coor = 0; coor < ox.Length; coor++) {
            // distance = hypotenuse(x - obstacle x[coor], y - obstacle y[coor])
            float d = hypo(x - ox[coor], y - oy[coor]);
            if (dmin >= d) { // if current distance <= min distance
                dmin = d; // reassign min distance to be this distance
                minid = coor; // make min id this index
            }
        }

        // distance between robot and shortest distance to an obstacle x, shortest distance to an obstacle y
        float dq = hypo(x - ox[minid], y - oy[minid]);

        // check if will collide
        if (dq <= rRadius) {
            // going to collide
            if (dq <= 0.1F) {
                dq = 0.1F;
            }
            return (0.5F * ETA * (1.0F / dq - 1.0F / rRadius)) * (0.5F * ETA * (1.0F / dq - 1.0F / rRadius));
        } else {
            // not going to collide
            return 0.0F;
        }
    }


    
    // <summmary>
    // Method that makes it easier to loop through the available directions of movement for the robot.
    // </summary>
    private float[,] getMotionModel() {
        float[,] motion = new float[,] { {1,0}, {0,1}, {-1,0}, {0,-1}, {-1,-1}, {-1,1}, {1,-1}, {1,1} };
        // directions of movement are as follows using compass directions:
        // E, N, W, S, NW, SW, SE, NE with E = East, N = North, W = West, S = South
        return motion;
    }

    /*

    // <summmary>
    // Method that finds oscillations within a path to ensure robot doesn't 'wobble' and follows path linearly.
    // </summary>
    private bool oscillationsDetection(Queue<float> previousIds, float ix, float iy) {
        previousIds.Enqueue((ix, iy));

        if (previousIds.Count > OSCILLATIONS_DETECTION_LENGTH) {
            previousIds.Dequeue();
        }

        List<float> previousIdsSet = new List<float>();
        foreach (float index in previousIds) {
            if (previousIdsSet.Contains(index)) {
                return true;
            } else {
                previousIdsSet.Add(index);
            }
        }
        return false;
    }

    */

    // <summmary>
    // Method that creates a path for the robot to follow based on the potential field calculations.
    // </summary>
    private void potentialFieldPlanning(float sx, float sy, float gx, float gy, float[] ox, float[] oy, float gridSize, float rRadius) {
        calcPotentialField(sx, sy, gx, gy, ox, oy, gridSize, rRadius);
        // return pmap, minx, miny -- get assigned to private personal instances instead
        StartCoroutine(waiter(sx, sy, gx, gy, ox, oy, gridSize, rRadius));

        Debug.Log("Goal!!");
        
    }

    IEnumerator<WaitForSeconds> waiter(float sx, float sy, float gx, float gy, float[] ox, float[] oy, float gridSize, float rRadius) {
        // compute distance between robot start coordinates and goal coordinates
        float d = hypo(sx - gx, sy - gy);

        // all these calculations find the coordinate of start and goal with respect to size of grid
        int ix = (int) (Mathf.Round((sx - Fminx) / gridSize));
        int iy = (int) (Mathf.Round((sy - Fminy) / gridSize));
        int gix = (int) (Mathf.Round((gx - Fminx) / gridSize));
        int giy = (int) (Mathf.Round((gy - Fminy) / gridSize));

        List<float> rx = new List<float>(); // all robot x coordinates throughout the simulation
        List<float> ry = new List<float>(); // all robot y coordinates throughout the simulation
        rx.Add(sx);
        ry.Add(sy);
        float[,] motion = getMotionModel();
        var previousIds = new Queue<float>();

        Debug.Log("d: " + d.ToString() + " | gridSize: " + gridSize.ToString());
        
        while (d >= gridSize) {
            float minp = Mathf.Infinity;
            int minix = -1;
            int miniy = -1;
            int i = 0; // holds index of iteration when going through motion model
        
            for (int j = 0; j < motion.Length / 2; j++) {
                int inx = (int) (ix + motion[j, 0]); // coordinate for this potential step
                int iny = (int) (iy + motion[j, 1]);
                float p;

                p = Fpmap[inx, iny];
                Debug.Log("inx: " + inx.ToString() + " | iny: " + iny.ToString() + " | p: " + p.ToString());
                if (minp > p) {
                    minp = p;
                    minix = inx;
                    miniy = iny;
                }
                i++;
            }
            ix = minix; // x of step with the smallest potential
            iy = miniy; // y of step with the smallest potential
            float xp = ix * gridSize + Fminx;
            float yp = iy * gridSize + Fminy;

            // if (inx >= Fpmap.Length ^ iny >= Fpmap[0].Length ^ inx < 0 ^ iny < 0) {
                //     p = Mathf.Infinity;
                //     Debug.Log("outside potential!");
                // } else {
                //     p = Fpmap[inx, iny];
                // }
            d = hypo(gx - xp, gy - yp);
            rx.Add(xp);
            ry.Add(yp);

            Vector3 rCurrPos = robot.transform.position;
            rCurrPos.x = xp;
            rCurrPos.z = yp;
            robot.transform.position = rCurrPos;
            Debug.Log("d: " + d.ToString() + " | ix: " + ix.ToString() + " | iy: " + iy.ToString() + " || x: " + xp.ToString() + " | y: " + yp.ToString());

            // if (oscillationsDetection(previousIds, ix, iy)) {
            //     Debug.Log("Oscillation detected at (" + ix + ", " + iy + ")!");
            //     break;
            // }
            // wait
            yield return new WaitForSeconds(0.10F);
        }
    }
    

    // <summmary>
    // Method that finds the hypotenuse of two triangle lengths passed in.
    // </summary>
    private float hypo(float x, float y) {
        return Mathf.Sqrt(Mathf.Pow(x, 2) + Mathf.Pow(y, 2));
    }

    // Update is called once per frame
    public void Update() {

    }
}
