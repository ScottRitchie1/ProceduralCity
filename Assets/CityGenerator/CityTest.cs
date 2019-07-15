using EzySlice;
using System.Collections.Generic;
using UnityEngine;
//https://github.com/OskarSigvardsson/unity-delaunay
//https://wiki.unity3d.com/index.php/SetPivot
//https://github.com/DavidArayan/ezy-slice
public class CityTest : MonoBehaviour
{
    public static CityTest instance;

    //seed for the city
    public int citySeed = 1;
    //the size of the noise used to generate the city
    public float noiseScale = 1;
    //the size of the city
    public Vector2 citySize;
    //the spacing between the suburbs
    public int BlockSpacing = 20;

    //the curve used to space out and define the suburb type placement
    public AnimationCurve curve;

    // This is where we will store the resulting data
    // Use this for initialization

    public LotType residentialSettings;
    public LotType urbanSettings;
    public LotType citySettings;

    public City myCity = null;


    private List<Vector2> CreateCityPoint(Vector2 offset)
    {

        //DisplayNoise();
        List<Vector2> points = new List<Vector2>();
        for (int x = 0; x <= citySize.x;)
        {
            for (int y = 0; y <= citySize.y;)//generate voroni sites within the city size, offset depending on noise weight
            {
                float xCoord = myCity.cityNoiseOffset.x + x / citySize.x * noiseScale;
                float yCoord = myCity.cityNoiseOffset.x + y / citySize.y * noiseScale;
                float noisePos = curve.Evaluate(Mathf.PerlinNoise(xCoord, yCoord));
                points.Add(new Vector2(
                    Mathf.Clamp(x + (int)Mathf.Lerp(0, (Random.Range(-BlockSpacing / 2, BlockSpacing / 2)), noisePos), 0, (citySize.x)),
                    Mathf.Clamp(y + (int)Mathf.Lerp(0, (Random.Range(-BlockSpacing / 2, BlockSpacing / 2)), noisePos), 0, (citySize.y))));
                points[points.Count - 1] = Vector3.MoveTowards(points[points.Count - 1], Vector3.zero, 1);
                y += BlockSpacing;

            }
            x += BlockSpacing;

        }

        return points;
    }

    // Update is called once per frame
    void Update()
    {
        GenerateNext();//update our map to add a block each frame
    }

    [ContextMenu("New Generation")]
    void GenerateNext()
    {
        if (myCity != null)
        {
            
            if (myCity.loadedSuburbIndex < myCity.suburbs.Count)
            {
                LotType suburbLotType = myCity.suburbs[myCity.loadedSuburbIndex].suburbType;
                // if we need to load a block
                if (myCity.suburbs[myCity.loadedSuburbIndex].loadedBlockIndex < myCity.suburbs[myCity.loadedSuburbIndex].myBlocks.Count)
                {
                    //load block
                    myCity.suburbs[myCity.loadedSuburbIndex].loadedBlockIndex++;
                    myCity.suburbs[myCity.loadedSuburbIndex].myBlocks[myCity.suburbs[myCity.loadedSuburbIndex].loadedBlockIndex-1].Initilize(suburbLotType);
                    
        
                    // if suburb has completed next frame lets load the next suburb
                    if(myCity.suburbs[myCity.loadedSuburbIndex].loadedBlockIndex > myCity.suburbs[myCity.loadedSuburbIndex].myBlocks.Count-1)
                    {
                        myCity.loadedSuburbIndex++;
                    }
                }
                else// if we finished loading the block load the next suburb
                {
                    myCity.suburbs[myCity.loadedSuburbIndex].GenerateBlocks(suburbLotType.blockSize, suburbLotType.blockDeviation, 0);
                }
               
            }
        }
    }

    [ContextMenu("New City")]
    public void GenerateCity()
    {
        // use our seed for generation
        Random.State currentRNGState = Random.state;
        Random.InitState(citySeed);

        myCity = new City();
        myCity.thisCity = new GameObject("NewCity");
        myCity.cityNoiseOffset = new Vector2(Random.Range(-10000, 10000), Random.Range(-10000, 10000));

        GK.VoronoiDiagram cityDiagram = new GK.VoronoiDiagram();
        //generate points
        List<Vector2> points = CreateCityPoint(myCity.cityNoiseOffset);// calulate voroni fracture sites

        //calculate city
        cityDiagram = new GK.VoronoiCalculator().CalculateDiagram(points);//calculate vorino diagram (ie roads seperating suburbs) using out sites

        //bounding
        List<Vector2> Polygon = new List<Vector2>();
        Polygon.Add(new Vector2(0, 0));
        Polygon.Add(new Vector2(citySize.x, 0));
        Polygon.Add(new Vector2(citySize.x, citySize.y));
        Polygon.Add(new Vector2(0, citySize.y));


        List<Vector2> clippedSite = new List<Vector2>();

        for (int siteIndex = 0; siteIndex < cityDiagram.Sites.Count; siteIndex++)
        {
            new GK.VoronoiClipper().ClipSite(cityDiagram, Polygon, siteIndex, ref clippedSite);// clip each of our suburbs

            if (clippedSite.Count > 0)
            {
                // assign and create a suburb object

                Suburb newSuburb = new Suburb();
                myCity.suburbs.Add(newSuburb);

                newSuburb.thisSuburb = new GameObject("Suburb");
                newSuburb.sitePosition = cityDiagram.Sites[siteIndex];
                newSuburb.thisSuburb.transform.position = new Vector3(newSuburb.sitePosition.x, 0, newSuburb.sitePosition.y);
                newSuburb.thisSuburb.transform.SetParent(myCity.thisCity.transform);
                newSuburb.borders = new List<Vector2>(clippedSite);

                float xCoord = myCity.cityNoiseOffset.x + newSuburb.sitePosition.x / citySize.x * noiseScale;
                float yCoord = myCity.cityNoiseOffset.x + newSuburb.sitePosition.y / citySize.y * noiseScale;
                float noiseValue = curve.Evaluate(Mathf.PerlinNoise(xCoord, yCoord));
                for (int i = 0; i < curve.keys.Length; i++)//set which type of suburb will be generated bassed off our noise weight at the suburbs position
                {
                    
                    if (noiseValue == curve.keys[i].value)
                    {
                        switch (i)
                        {
                            case 0:
                                {
                                    newSuburb.suburbType = residentialSettings;
                                    break;
                                }
                            case 1:
                                {
                                    newSuburb.suburbType = urbanSettings;
                                    break;
                                }
                            default:
                                {
                                    newSuburb.suburbType = citySettings;
                                    break;
                                }
                        }
                       

                    }
                }

                newSuburb.seed = Random.Range(int.MinValue, int.MaxValue);
            }
        }
        Random.state = currentRNGState;
    }
    

    //used for debugging, displays out noise
    [ContextMenu("DisplayNoise")]
    private void DisplayNoise()
    {
        Texture2D tx = new Texture2D((int)citySize.x, (int)citySize.y);
        for (int x = (int)citySize.x; x > 0 - 1; x--)
        {
            for (int y = (int)citySize.y; y > 0; y--)
            {
                float xCoord = myCity.cityNoiseOffset.x + x / citySize.x * noiseScale;
                float yCoord = myCity.cityNoiseOffset.x + y / citySize.y * noiseScale;
                float noisePos = curve.Evaluate(Mathf.PerlinNoise(xCoord, yCoord));
                tx.SetPixel(x, y, new Color(noisePos, noisePos, noisePos, 1));

            }
        }
        tx.Apply();

        GameObject.CreatePrimitive(PrimitiveType.Plane).GetComponent<Renderer>().material.mainTexture = tx;
    }

    //shrinks a polygon
    public static List<Vector2> Shrink(IList<Vector2> polygon, float amount)
    {
        List<Vector2> newPoly = new List<Vector2>(polygon);

        amount = Mathf.Abs(amount);

        var count = polygon.Count;

        for (int i = 0; i < count; i++)//moves each vertes/edge to its left (shrink the vertcies)
        {
            int next = i + 1 == count ? 0 : i + 1;
            int prev = i - 1 < 0 ? count - 1 : i - 1;
            Vector2 nextDir = (polygon[next] - polygon[i]);
            Vector2 prevDir = (polygon[i] - polygon[prev]);
            Vector2 nextCross = GK.Geom.RotateRightAngle(nextDir.normalized) * amount;
            Vector2 prevCross = GK.Geom.RotateRightAngle(prevDir.normalized) * amount;

            newPoly[i] = GK.Geom.LineLineIntersection(polygon[prev] + prevCross, prevDir, polygon[i] + nextCross, nextDir);
        }

        int offsetTally = 0;// used to track if the object cant be shrunk
        if (polygon.Count > 3)// we only need to check if verticies are out of the clock wise range if we arnt a triangle
        {
            polygon = new List<Vector2>(newPoly);
            newPoly.Clear();
            int beginRight = -1;
            int beginRightPrev = -1;
            for (int i = 0; i < polygon.Count; i++)//disolve overlapping edges
            {
                int prev = i - 1 < 0 ? polygon.Count - 1 : i - 1;
                int next = i + 1 == polygon.Count ? 0 : i + 1;



                if (beginRight > -1)// if we are waiting to rejoin the vertex thats outsude the polygon
                {
                    if (GK.Geom.ToTheRight(polygon[next], polygon[prev], polygon[i]))// if the vertex is not outside the correct polygon
                    {
                        //find the corner which the 2 out side polygon vertcies intersect 
                        Vector2 intersectionPoint = GK.Geom.LineLineIntersection(polygon[i], polygon[next] - polygon[i], polygon[beginRight], polygon[beginRight] - polygon[beginRightPrev]);
                        newPoly.Add(intersectionPoint);
                        beginRight = -1;
                    }
                }
                else
                {
                    if (GK.Geom.ToTheRight(polygon[next], polygon[prev], polygon[i]))// if the vertex is our side the polygon
                    {
                        if (i == 0)// we need this, becasue we cannon start on a vertci thats next vertecy is to the right. so we will keep ofsetting them untillwe find one that is valid, this is ou starting vertex
                        {
                            polygon.Add(polygon[i]);
                            polygon.RemoveAt(i);
                            i--;
                            offsetTally++;
                            if (offsetTally == polygon.Count)// if we have looped through everything lets break out
                            {
                                break;
                            }
                            continue;
                        }
                        beginRight = i;
                        beginRightPrev = prev;
                    }
                    else
                    {
                        // if the vertex is valid and should be part of the shrunken poly just add it
                        newPoly.Add(polygon[i]);
                    }
                }
            }
        }

        return newPoly;
    }

    //used for debugging, draws a polygon using a line renderer
    public static LineRenderer LineRenderPoly(List<Vector2> list, GameObject go = null)
    {
        LineRenderer lr;
        if (go)
        {
            lr = go.AddComponent<LineRenderer>();
        }
        else
        {
            lr = new GameObject().AddComponent<LineRenderer>();
        }
        lr.loop = true;
        lr.positionCount = list.Count;
        for (int i = 0; i < list.Count; i++) { lr.SetPosition(i, new Vector3(list[i].x, 0, list[i].y)); }

        return lr;
    }
}


[System.Serializable]
public class LotType// used to define how a type of suburb will be constructed
{
    //the buldings that the suburb will be made out of
    public List<GameObject> buildings;
    // the foot print for each building
    public List<Vector2> buildingSizes;
    //the colour of the ground
    public Color groundColour;
    //the size of the subdivided blocks
    public float blockSize;
    //the "scatteredness" or how much the blocks are unaligned
    public float blockDeviation;
    //the foot path object
    public GameObject footPath;
    public float maxDepth
    {
        get//updates our max depth to be currect
        {
            if (_maxDepth == 0)
            {
                foreach(Vector2 vec in buildingSizes)
                {
                    if(_maxDepth <= 0 || vec.y > _maxDepth)
                    {
                        _maxDepth = vec.y;
                    }
                }
            }
            return _maxDepth;
        }
    }
    float _maxDepth = 0;
}

public class City
{
    // the offset on our noise plane for the city
    public Vector2 cityNoiseOffset;
    // used for loading suburbs incrementially
    public int loadedSuburbIndex = 0;
    // the list of suburbs our city is made out of
    public List<Suburb> suburbs = new List<Suburb>();
    // the game object for this city
    public GameObject thisCity;
}
public class Suburb
{
    // the seed for this suburb
    public int seed = 0;
    // the 2d position for this suburb
    public Vector2 sitePosition;
    // the borders for the suburb
    public List<Vector2> borders =  new List<Vector2>();
    //used for incrementually loading
    public int loadedBlockIndex = 0;
    // the list of blocks in this suburb
    public List<Block> myBlocks = new List<Block>();
    // the game object for this suburb
    public GameObject thisSuburb;
    //the parent city
    public City parentCity;
    //the type of suburb this is
    public LotType suburbType;

    //subdivide our suburb into blocks
    public void GenerateBlocks(float spacing, float deviation = 0, float alignmentRotation = 0)
    {
        //set our seed for the suburb
        Random.State currentRNGState = Random.state;
        Random.InitState(seed);
        
        float minX = float.MaxValue, maxX = float.MinValue, minY = float.MaxValue, maxY = float.MinValue;
        Vector2 rotationPivot = new Vector2(0,0);
        List<Vector2> shrunkBorders = new List<Vector2>(borders);
        List<Vector2> sites = new List<Vector2>();

        // shrink our borders so our points arnt created to close to the edge, this will result in blocks that are to small
        shrunkBorders = CityTest.Shrink(shrunkBorders, spacing / 2);
        if (shrunkBorders.Count >= 3)
        {
            for (int i = 0; i < shrunkBorders.Count; i++)//find anchorPoint
            {
                if (shrunkBorders[i].x < rotationPivot.x) rotationPivot.x = shrunkBorders[i].x;
                if (shrunkBorders[i].y < rotationPivot.y) rotationPivot.y = shrunkBorders[i].y;

            }

            //find rotate boarders for alignment and find bounding volume
            for (int i = 0; i < shrunkBorders.Count; i++)
            {
                shrunkBorders[i] = shrunkBorders[i].RotateAroundPoint(rotationPivot, alignmentRotation);// Vector2.MoveTowards(shrunkBorders[i].RotateAroundPoint(rotationPivot, alignmentRotation), borderCenter, spacing/2);// spacing/2);
                if (shrunkBorders[i].x > maxX) maxX = shrunkBorders[i].x;
                else if (shrunkBorders[i].x < minX) minX = shrunkBorders[i].x;
                if (shrunkBorders[i].y > maxY) maxY = shrunkBorders[i].y;
                else if (shrunkBorders[i].y < minY) minY = shrunkBorders[i].y;
            }

            //triangulate out polygon so we can check if created points are within our poly
            GK.DelaunayTriangulation triangulation = new GK.DelaunayCalculator().CalculateTriangulation(shrunkBorders);

            float xCenterOffset = ((maxX - minX) % spacing) / 2;
            float yCenterOffset = ((maxY - minY) % spacing) / 2;
            for (float x = minX - xCenterOffset; x < maxX; x += spacing)
            {
                for (float y = minY - yCenterOffset; y < maxY; y += spacing)
                {
                    Vector2 newPoint = new Vector2(x + Random.Range(-deviation, deviation), y + Random.Range(-deviation, deviation));// generate a points i a grid

                    for (int i = 0; i < triangulation.Triangles.Count; i += 3)// if the point is contained within our polygon
                    {
                        if (GK.Geom.PointInTriangle(newPoint,
                            triangulation.Vertices[triangulation.Triangles[i]],
                            triangulation.Vertices[triangulation.Triangles[i + 1]],
                            triangulation.Vertices[triangulation.Triangles[i + 2]]))
                        {
                            Vector2 rotatedPoint = newPoint.RotateAroundPoint(rotationPivot, -alignmentRotation);
                            sites.Add(rotatedPoint);// rotate our aligned point to its original rotation and add it to the block site list

                            break;


                        }
                    }
                }
            }
        }

        if (sites.Count >= 3)// we can only create a diagram if we have more then 2 verts
        {
            GK.VoronoiDiagram blockVoronoiDiagram = new GK.VoronoiCalculator().CalculateDiagram(sites);// generate our streets and blocks in our suburb
        
            for (int i = 0; i < blockVoronoiDiagram.Sites.Count; i++)
            {
                List<Vector2> clippedSite = null;
       
                if (blockVoronoiDiagram.FirstEdgeBySite.Count > 0)////THIS IS NEEDED BECAUSE THE DIAGRAM CAN BE INVALID SOMETIMES
                {

                    new GK.VoronoiClipper().ClipSite(blockVoronoiDiagram, borders, i, ref clippedSite);// clip our block by our suburb

                    if (clippedSite.Count > 0)
                    {
                        Block newBlock = new Block();// create and assign the block
                        newBlock.borders = new List<Vector2>(clippedSite);
                        newBlock.parentSuburb = this;
                        newBlock.seed = Random.Range(int.MinValue, int.MaxValue);
                        newBlock.blockPosition = blockVoronoiDiagram.Sites[i];

                        newBlock.thisBlock = new GameObject("Block");
                        newBlock.thisBlock.transform.SetParent(thisSuburb.transform);
                        newBlock.thisBlock.transform.localPosition = new Vector3(newBlock.blockPosition.x, 0, newBlock.blockPosition.y);
                        myBlocks.Add(newBlock);
                    }
                    else
                    {
                        Debug.LogError("SUBURB SITE AFTER CLIPPED WASNT VALID");
                    }


                }
                else////THIS IS NEEDED BECAUSE THE DIAGRAM CAN BE INVALID SOMETIMES if its not valid make the entire subrb a block
                {
                    Debug.LogWarning("SITE COULTNT BE CREATED BECASUE DIAGRAM DOESNT HAVE STARTING EDGES");
                    Block newBlock = new Block();
                    newBlock.borders = new List<Vector2>(borders);
                    newBlock.parentSuburb = this;
                    newBlock.seed = Random.Range(int.MinValue, int.MaxValue);
                    newBlock.blockPosition = sitePosition;
                
                    newBlock.thisBlock = new GameObject("Block");
                    newBlock.thisBlock.transform.SetParent(thisSuburb.transform);
                    newBlock.thisBlock.transform.localPosition = new Vector3(newBlock.blockPosition.x, 0, newBlock.blockPosition.y);
                    myBlocks.Add(newBlock);
                    break;
                }
            }
        }
        else// if the suburb has less then 3 verts it cant be split so we make the entire suburb a block
        {

            Block newBlock = new Block();
            newBlock.borders = new List<Vector2>(borders);
            newBlock.parentSuburb = this;
            newBlock.seed = Random.Range(int.MinValue, int.MaxValue);
            newBlock.blockPosition = sitePosition;
        
            newBlock.thisBlock = new GameObject("Block");
            newBlock.thisBlock.transform.SetParent(thisSuburb.transform);
            newBlock.thisBlock.transform.localPosition = new Vector3(newBlock.blockPosition.x, 0, newBlock.blockPosition.y);
            myBlocks.Add(newBlock);
        }
        Random.state = currentRNGState;
    }
}
public class Block
{
    // the seed to generate the block
    public int seed = 0;
    //the borders / streets for the blcok
    public List<Vector2> borders = new List<Vector2>();
    // the gameobject for this block
    public GameObject thisBlock;
    // the parent suburb
    public Suburb parentSuburb;
    // the position for the block
    public Vector2 blockPosition;
    // the size of the road 
    public float roadSize = 5;
    //the size of the footpath
    public float footPathSize = 5;
    // the type of block this is
    public LotType _lotType;

    //generate our block
    public void Initilize(LotType lotType)
    {
        //set the bolock seed
        Random.State currentRNGState = Random.state;
        Random.InitState(seed);

        _lotType = lotType;

        
        MeshRenderer renderer = thisBlock.AddComponent<MeshRenderer>();
        renderer.material = Resources.Load<Material>("CityMat");
        renderer.material.color = lotType.groundColour;
        MeshFilter filter = thisBlock.AddComponent<MeshFilter>();
        filter.mesh = new Mesh();


        List<Vector2> blockGroundPoly;

        //generate our block ground and foot paths
        if (_lotType.footPath)
        {
            blockGroundPoly = CityTest.Shrink(borders, roadSize + footPathSize);
            BuildFootPath(_lotType.footPath);
        }
        else
        {
            blockGroundPoly = CityTest.Shrink(borders, roadSize);
        }

        //generate our buildings
        BuildLots(ref lotType);
        //generate our roads
        BuildRoads();

        Vector3[] verticies = new Vector3[blockGroundPoly.Count];
        int[] triangles = new int[blockGroundPoly.Count * 3];
        Color[] vertColours = new Color[verticies.Length];

        for (int i = 0; i < blockGroundPoly.Count; i++){// create our ground mesh
            verticies[i] = new Vector3(blockGroundPoly[i].x, 0, blockGroundPoly[i].y) - thisBlock.transform.position;

            triangles[i * 3] = 0;
            if (i + 1 > verticies.Length - 1)
            {
                triangles[i * 3 + 1] = 0;
            }
            else
            {
                triangles[i * 3 + 1] = i + 1;
            }
            triangles[i * 3 + 2] = i;

        }

        filter.mesh.vertices = verticies;
        filter.mesh.triangles = triangles;

        Random.state = currentRNGState;
    }

    //build the roads
    void BuildRoads()
    {
        for(int currBorderIndex = 0; currBorderIndex < borders.Count; currBorderIndex++)// for each road
        {
            //create a container object
            GameObject road = new GameObject("Road: " + currBorderIndex.ToString());
            road.transform.position = new Vector3(borders[currBorderIndex].x, 0, borders[currBorderIndex].y) + thisBlock.transform.position;
            road.transform.SetParent(thisBlock.transform);

            //get our prefabs
            GameObject roadEmpty = Resources.Load<GameObject>("RoadEmpty");
            GameObject roadMiddle = Resources.Load<GameObject>("RoadStraight2");

            // find the index for the next roads in the poly
            int nextBorderIndex = currBorderIndex+1;
            if (nextBorderIndex >= borders.Count) nextBorderIndex = 0;
            int prevBorderIndex = currBorderIndex - 1;
            if (prevBorderIndex < 0) prevBorderIndex = borders.Count-1;
            int nextNextBorderIndex = nextBorderIndex + 1;
            if (nextNextBorderIndex >= borders.Count) nextNextBorderIndex = 0;

            // the front and end position of the road we are creating
            Vector3 roadFrontWorldPosition = new Vector3(borders[currBorderIndex].x, 0, borders[currBorderIndex].y);
            Vector3 roadEndWorldPosition = new Vector3(borders[nextBorderIndex].x, 0, borders[nextBorderIndex].y);

            //the front position of the next road
            Vector3 nextRoadPosition = new Vector3(borders[nextNextBorderIndex].x, 0, borders[nextNextBorderIndex].y);
            //the back position of the previous road
            Vector3 prevRoadPosition = new Vector3(borders[prevBorderIndex].x, 0, borders[prevBorderIndex].y);
            //the direction the roads travel
            Vector3 thisRoadDir = (roadEndWorldPosition - roadFrontWorldPosition).normalized;
            Vector3 nextRoadDir = (nextRoadPosition - roadEndWorldPosition).normalized;
            Vector3 prevRoadDir = (roadFrontWorldPosition - prevRoadPosition).normalized;
            // the slicing directions to slice the end of the roads. this is half the angle between the 2 roads
            Vector3 frontSliceDirection = (thisRoadDir + prevRoadDir) * 0.5f;
            Vector3 endSliceDirection = -(thisRoadDir + nextRoadDir) * 0.5f;
            float roadSegmentLenght = roadSize;

            //roads are split into 3, front, middle end. the front and the end are slices on an angle and dont have lines, the middle are only sliced on the end and perp from the road direction

            //the end point is total distance of the road
            float endRoadLenght = Vector3.Distance(roadEndWorldPosition, roadFrontWorldPosition);
            // the begin point is where the front road ends and the middle begins
            float beginRoadLenght = Mathf.Max(Mathf.Tan((90 - Vector3.Angle(Vector3.Cross(frontSliceDirection, Vector3.up), thisRoadDir)) * Mathf.Deg2Rad) * roadSegmentLenght, Mathf.Tan((Vector3.Angle(Vector3.Cross(frontSliceDirection, Vector3.up), thisRoadDir)) * Mathf.Deg2Rad) * roadSegmentLenght);
            beginRoadLenght = Mathf.Clamp(beginRoadLenght, 0, endRoadLenght / 2);
            // the middle is where the middle road ends the and end road begins
            float middleRoadLenght = Mathf.Max((Mathf.Tan((90 - Vector3.Angle(Vector3.Cross(endSliceDirection, Vector3.up), thisRoadDir)) * Mathf.Deg2Rad) * roadSegmentLenght), (Mathf.Tan((Vector3.Angle(Vector3.Cross(endSliceDirection, Vector3.up), thisRoadDir)) * Mathf.Deg2Rad) * roadSegmentLenght));
            middleRoadLenght = endRoadLenght - Mathf.Clamp(middleRoadLenght, 0, endRoadLenght / 2);

            GameObject roadObj;
            //beginRoad ---- CHANGE TO GO OTHER WAY AND AVOID CUTTING OF THE BACK
            for (float lenght = 0; lenght < beginRoadLenght; lenght += roadSegmentLenght)
            {
                //create a road tile along our road
                roadObj = GameObject.Instantiate(roadEmpty, roadFrontWorldPosition + thisRoadDir * lenght + Vector3.up * 0.001f, Quaternion.LookRotation(thisRoadDir, Vector3.up));

                //slice the front road angle
                EzySlice.SlicedHull hull = roadObj.Slice(roadFrontWorldPosition, frontSliceDirection);
                if (hull != null && hull.upperHull)
                {
                    MonoBehaviour.Destroy(roadObj);
                    roadObj = hull.CreateUpperHull(roadObj);
                }

                hull = roadObj.Slice(roadEndWorldPosition, endSliceDirection);
                if (hull != null && hull.upperHull)
                {
                    MonoBehaviour.Destroy(roadObj);
                    roadObj = hull.CreateUpperHull(roadObj);
                }
                //slice the back off the road
                if (lenght + roadSegmentLenght > beginRoadLenght)
                {
                    hull = roadObj.Slice(roadFrontWorldPosition + thisRoadDir * beginRoadLenght, -thisRoadDir);

                    if (hull != null && hull.upperHull)
                    {
                        MonoBehaviour.Destroy(roadObj);
                        roadObj = hull.CreateUpperHull(roadObj);
                    }
                }
                roadObj.name = "Begin Part";

                roadObj.transform.SetParent(road.transform);

            }


            //middleRoad
            for (float lenght = beginRoadLenght; lenght < middleRoadLenght; lenght += roadSegmentLenght)
            {
                //create a road tile along our road
                roadObj = GameObject.Instantiate(roadMiddle, roadFrontWorldPosition + thisRoadDir * lenght + Vector3.up*0.001f, Quaternion.LookRotation(thisRoadDir, Vector3.up));

                //slice back off
                if(lenght + roadSegmentLenght > middleRoadLenght)
                {
                    EzySlice.SlicedHull hull = roadObj.Slice(roadFrontWorldPosition + thisRoadDir * middleRoadLenght, -thisRoadDir);
                    
                    if (hull != null && hull.upperHull)
                    {
                        MonoBehaviour.Destroy(roadObj);
                        roadObj = hull.CreateUpperHull(roadObj);
                    }
                }

                roadObj.name = "Middle Part";
                roadObj.transform.SetParent(road.transform);
            }

            //endRoad
            for (float lenght = middleRoadLenght; lenght < endRoadLenght; lenght += roadSegmentLenght)
            {
                GameObject roadObj1 = GameObject.Instantiate(roadEmpty, roadFrontWorldPosition + thisRoadDir * lenght + Vector3.up * 0.001f, Quaternion.LookRotation(thisRoadDir, Vector3.up));
                //slice the road angle
                EzySlice.SlicedHull hull = roadObj1.Slice(roadEndWorldPosition, endSliceDirection);
                if (hull != null && hull.upperHull)
                {
                    MonoBehaviour.Destroy(roadObj1);
                    roadObj1 = hull.CreateUpperHull(roadObj1);
                }

                roadObj1.name = "End Part";

                roadObj1.transform.SetParent(road.transform);
            }
        }
    }

    void BuildFootPath(GameObject footPathPrefab)
    {
        List<Vector2> footPathPoly = CityTest.Shrink(borders, roadSize);
        //GameObject footPathPrefab = Resources.Load<GameObject>("FootPath");
        float footPathSegmentLenght = footPathSize;

        for (int currBorderIndex = 0; currBorderIndex < footPathPoly.Count; currBorderIndex++)
        {
            GameObject footPath = new GameObject("FootPath: " + currBorderIndex.ToString());
            footPath.transform.position = new Vector3(footPathPoly[currBorderIndex].x, 0, footPathPoly[currBorderIndex].y) + thisBlock.transform.position;
            footPath.transform.SetParent(thisBlock.transform);

            // this works the same as the roads for the most part, referance that
            int nextBorderIndex = currBorderIndex + 1;
            if (nextBorderIndex >= footPathPoly.Count) nextBorderIndex = 0;
            int prevBorderIndex = currBorderIndex - 1;
            if (prevBorderIndex < 0) prevBorderIndex = footPathPoly.Count - 1;
            int nextNextBorderIndex = nextBorderIndex + 1;
            if (nextNextBorderIndex >= footPathPoly.Count) nextNextBorderIndex = 0;
            Vector3 FrontWorldPosition = new Vector3(footPathPoly[currBorderIndex].x, 0, footPathPoly[currBorderIndex].y);
            Vector3 EndWorldPosition = new Vector3(footPathPoly[nextBorderIndex].x, 0, footPathPoly[nextBorderIndex].y);
            Vector3 nextPosition = new Vector3(footPathPoly[nextNextBorderIndex].x, 0, footPathPoly[nextNextBorderIndex].y);
            Vector3 prevPosition = new Vector3(footPathPoly[prevBorderIndex].x, 0, footPathPoly[prevBorderIndex].y);
            Vector3 thisDir = (EndWorldPosition - FrontWorldPosition).normalized;
            Vector3 nextDir = (nextPosition - EndWorldPosition).normalized;
            Vector3 prevDir = (FrontWorldPosition - prevPosition).normalized;
            Vector3 frontSliceDirection = (thisDir + prevDir) * 0.5f;
            Vector3 endSliceDirection = -(thisDir + nextDir) * 0.5f;
            float endRoadLenght = Vector3.Distance(EndWorldPosition, FrontWorldPosition);

            for (float lenght = 0; lenght < endRoadLenght; lenght += footPathSegmentLenght)
            {
                GameObject footPathObj = GameObject.Instantiate(footPathPrefab, FrontWorldPosition + thisDir * lenght + Vector3.up * 0.001f, Quaternion.LookRotation(thisDir, Vector3.up));
                EzySlice.SlicedHull hull = footPathObj.Slice(FrontWorldPosition, frontSliceDirection);
                if (hull != null && hull.upperHull)
                {
                    MonoBehaviour.Destroy(footPathObj);
                    footPathObj = hull.CreateUpperHull(footPathObj);

                }

                hull = footPathObj.Slice(EndWorldPosition, endSliceDirection);
                if (hull != null && hull.upperHull)
                {
                    MonoBehaviour.Destroy(footPathObj);
                    footPathObj = hull.CreateUpperHull(footPathObj);

                }

                

                //THis is required to remove the empty material array elements the slicer leave there for some reason, bug with ezyslice
                Material[] newShared = new Material[1];
                newShared[0] = footPathObj.GetComponent<Renderer>().sharedMaterials[0];
                footPathObj.GetComponent<Renderer>().sharedMaterials = newShared;

                footPathObj.name = "Foot Path";
                footPathObj.transform.SetParent(footPath.transform);
            }
        }
    }

    void BuildLots(ref LotType _lot)
    {
        //shrink poly
        List<Vector2> lotPoly = CityTest.Shrink(borders, footPathSize + roadSize);

        //for each edge in poly
        for(int currEdge = 0; currEdge < lotPoly.Count; currEdge++)
        {
          

            //find its width and height
            float depth = _lot.maxDepth;
            
            
            int nextEdge = currEdge + 1 == lotPoly.Count ? 0 : currEdge + 1;
            int nextNextEdge = nextEdge + 1 == lotPoly.Count ? 0 : nextEdge + 1;
            int prevEdge = currEdge - 1 < 0 ? lotPoly.Count - 1 : currEdge - 1;
            //get cross point in from the edge direction
            Vector2 cross = GK.Geom.RotateRightAngle(lotPoly[nextEdge] - lotPoly[currEdge]).normalized * depth;
            Vector2 nextEdgeDirection = (lotPoly[nextEdge] - lotPoly[currEdge]).normalized;
            Vector2 prevEdgeDirection = lotPoly[prevEdge] - lotPoly[currEdge];
            Vector2 nextNextEdgeDirection = lotPoly[nextEdge] - lotPoly[nextNextEdge];
            float angleBetweenPrevNext = Vector2.Angle(nextEdgeDirection, prevEdgeDirection);
            float angleBetweenNextNextNext = Vector2.Angle(nextEdgeDirection, nextNextEdgeDirection);

            //find starting lenght (sin tan) along the original edge
            float startLenght = Mathf.Tan((90 - angleBetweenPrevNext) * Mathf.Deg2Rad) * depth;
            //find the hypotinues of the previous end lenght
            startLenght += depth / Mathf.Sin(angleBetweenPrevNext * Mathf.Deg2Rad) ; //hypot

            //find ending lenght (sin tan) along the original edge starting from the back normal edge
            float endLenght = Vector3.Distance(lotPoly[currEdge], lotPoly[nextEdge]) + (Mathf.Tan((90 - angleBetweenPrevNext) * Mathf.Deg2Rad) * depth);
            for(float currentLenght = startLenght; currentLenght < endLenght;)
            {
                //randomly get a building
                int buildingIndex = Random.Range(0, _lot.buildings.Count);
                GameObject building = _lot.buildings[buildingIndex];
                float width = _lot.buildingSizes[buildingIndex].x;

                if(currentLenght + width < endLenght)// check if pointing can fit
                {
                    //add building between points;
                    Vector2 bottomLeft = lotPoly[currEdge] + nextEdgeDirection * currentLenght;
                    Vector2 bottomRight = bottomLeft + nextEdgeDirection * width;
                    Vector2 topRight = bottomRight + cross;
                    Vector2 topLeft = bottomLeft + cross;
                    currentLenght += width;

                    Vector2 centerLot = (bottomLeft + bottomRight + topRight + topLeft) / 4;

                    //crete the building in the lot
                    GameObject newBuilding = GameObject.Instantiate(building, new Vector3(centerLot.x, 0, centerLot.y), Quaternion.LookRotation(Vector3.Cross(new Vector3(nextEdgeDirection.x, 0, nextEdgeDirection.y), -Vector3.up), Vector3.up), thisBlock.transform);
                    newBuilding.name = "Building: " + currentLenght.ToString();

                    }
                else
                {
                    break;
                }

            }

        }
    }
}
