///author:huwei
///date:2018.4.5
//#define FlowFieldDebug
//#define USING_ASTAR_PROJECT
#define USING_DYNAMIC_TREE
#if USING_ASTAR_PROJECT && UNITY_EDITOR
using Pathfinding;
#endif
#if UNITY_EDITOR
using UnityEngine;
#endif
using System;
using System.Collections.Generic;

using TrueSync;
namespace PathFinding
{
    public  class FlowField
    {       
        struct Node : IResetable
        {
            public FP cost;
            public int x;
            public int z;
           
            public void Reset()
            {
                x = -1;
                z = -1;
                cost=(FP.MinValue);
            }
            public static bool operator !=(Node a,Node b)
            {
                return a.x != b.x || a.z != b.z || a.cost != b.cost;
            }
            public static bool operator ==(Node a, Node b)
            {
                return a.x == b.x && a.z == b.z && a.cost == b.cost;
            }
            public override bool Equals(System.Object o)
            {
                if (o == null) return false;
                var rhs = (Node)o;

                return x == rhs.x && z == rhs.z && cost == rhs.cost;
            }
            public override int GetHashCode()
            {
                return x * 73856093 ^ cost.AsInt() * 19349663 ^ z * 83492791;
            }

        }
        int _gridWidth = 0;
        int _gridHeight = 0;
        TSVector _destination;
        public int enabledTags = -1;
        FP[,] _gridDijkstraCost = null;
        TSVector[,] _flowField = null;
        bool[,] _gridLOS = null;

        //
        internal GridMap _gridMap = null;
        //
        public FlowField(TSVector destination, GridMap  map)
        {
            _gridMap = map; // GridMapManager.instance.GetMap(mapId);
            IniFlowField(_gridMap.Size.x, _gridMap.Size.y, destination);
        }
        //
        public void IniFlowField(int w, int h, TSVector destination)//destination  from
        {
#if USING_ASTAR_PROJECT && UNITY_EDITOR
            _gridWidth = Pathfinding.GridNode.GetGridGraph(0).width;
            _gridHeight = Pathfinding.GridNode.GetGridGraph(0).depth;
#else
            _gridWidth = w;
            _gridHeight = h;
#endif
            _destination = destination;
            if (_gridDijkstraCost == null)
            {
                _gridDijkstraCost = new FP[_gridWidth, _gridHeight];
            }
            if (_flowField == null)
            {
                _flowField = new TSVector[_gridWidth, _gridHeight];
            }
            if (_gridLOS == null)
            {
                _gridLOS = new bool[_gridWidth, _gridHeight];
            }
            UpdateFlowField();
        }
        void ResetData()
        {
            //reset cost
            for (int i = 0; i < _gridWidth; i++)
            {
                for (int j = 0; j < _gridHeight; j++)
                {
                    //check for static obstacles
                    _gridDijkstraCost[i, j] =_gridMap.IsWalkable(i, j) ? FP.MinValue : FP.MaxValue;
                    _flowField[i, j] = TSVector.zero;
                    _gridLOS[i, j] = false;
                }
            }
            ResetVisitingNodes();
           
        }
#if FlowFieldDebug && UNITY_EDITOR
        static bool bDraw = false;
#endif
        public void UpdateFlowField(bool bForceDraw=false)
        {
            TSVector destination = _destination;
            ResetData();
            generate_gridDijkstraCost(destination);
            generateFlowField(destination);
#if FlowFieldDebug && UNITY_EDITOR
           
            if(!bDraw || bForceDraw)
            {
                if(_destination.x!=0 || _destination.z!=0)
                {
                    DrawFlowField();
                    bDraw = true;
                }              
            }          
#endif
        }

        //
        static IInt2[] diagonalNeighbourOffset = new IInt2[]
        {
            new IInt2(-1,-1),
            new IInt2(1,-1),
            new IInt2(1,1),
            new IInt2(-1,1),
        };
        internal bool IsValid(int x, int y, bool SkipDynamic=false)
        {
            return x >= 0 && y >= 0 && x < _gridWidth && y < _gridHeight && _gridDijkstraCost[x,y] != FP.MaxValue;
        }
        bool IsInGridArea(int x, int y)
        {
            return x >= 0 && y >= 0 && x < _gridWidth && y < _gridHeight;
        }
        bool[] _tmpValid = new bool[4] { false,false,false,false};
        void generateFlowField(TSVector destination)
        {
            //scan from end node
            int pathEnd = _gridMap.GetGridNodeId(destination);  
            int endX = pathEnd % _gridWidth;
            int endZ = pathEnd / _gridWidth;
            //
            for (int x = 0; x < _gridWidth; x++)
            {
                for (int z = 0; z < _gridHeight;z++)
                {
                    bool bNodeCostMax = _gridDijkstraCost[x, z] == FP.MaxValue;
                    if (bNodeCostMax)
                    {
                        //
//#if USING_DYNAMIC_TREE
//                        _gridMap.AddObstacle(x,z);
//#endif
                        continue;
                    }
                    if (_gridLOS[x,z])
                    {                       
                        TSVector vec3 = TSVector.zero;
                        vec3.Set(endX-x, 0, endZ-z);
                        _flowField[x, z] = CustomMath.Normalize(vec3);
                        continue;
                    }

                    TSVector2 pos;
                    pos.x = x;
                    pos.y=z;
                    //find  the one with the lowest distance of neighbours
                    TSVector2 min=pos ;
                    min.Set(int.MaxValue, int.MaxValue);
                    FP minDist = FP.MaxValue;//
                    int count = GridMap.straightNeighbourOffset.Length;
                     
                    for (var i = 0; i < count; i++)
                    {
                        int n_x = GridMap.straightNeighbourOffset[i].x + x;
                        int n_z = GridMap.straightNeighbourOffset[i].y + z;
                        _tmpValid[i] = IsValid(n_x, n_z);
                        if (_tmpValid[i])
                        {
                            FP dist = FP.Zero;
                            dist = _gridDijkstraCost[n_x, n_z] - _gridDijkstraCost[x, z];
                            if (dist < minDist)
                            {
                                min.Set(n_x,n_z);
                                minDist = dist;
                            }                          
                        }                    
                    }
                    for (var i = 0; i < count; i++)
                    {
                        if(_tmpValid[i]&&_tmpValid[(i+1)%4])
                        {
                            int n_x = diagonalNeighbourOffset[i].x + x;
                            int n_z = diagonalNeighbourOffset[i].y + z;
                            if (IsValid(n_x, n_z))
                            {
                                FP dist = FP.Zero;
                                dist = _gridDijkstraCost[n_x, n_z] - _gridDijkstraCost[x, z];
                                if (dist < minDist)
                                {
                                    min.Set(n_x, n_z);
                                    minDist = dist;
                                }
                            }
                        }                       
                    }
                    //
                    if (minDist != FP.MaxValue)
                    {
                        TSVector2 v = min - pos;
                        TSVector vec3 = TSVector.zero;
                        vec3.Set(v.x, 0, v.y);

                        _flowField[x, z] = CustomMath.Normalize(vec3);
                    }
                    else
                    {
#if UNITY_EDITOR
                        UnityEngine.Debug.LogError("grid map has node surrounded with obstacles");
#endif
                        _flowField[x, z] = TSVector.zero;
                    }
                }
            }
            //_gridMap._obstacleHasSetUp = true;
        }
        //        
        List<Node> _visitingNodes = new List<Node>();
        void ResetVisitingNodes()
        {
            _visitingNodes.Clear();
        }

        //genterate cost from destination with Dijkstra alg
        void generate_gridDijkstraCost(TSVector destination)
        {            
            //scan from end node
            int pathEnd = _gridMap.GetGridNodeId(destination);
            Node endNode;//= _nodesPool.New();          
            endNode.cost = FP.Zero;
            endNode.x = pathEnd % _gridWidth;
            endNode.z= pathEnd / _gridWidth;
            _gridLOS[endNode.x, endNode.z] = true;
            _gridDijkstraCost[endNode.x, endNode.z] = FP.Zero;
            _visitingNodes.Add(endNode);

            //for each node we need to visit, starting with the pathEnd
            int neighbourCount =GridMap.neighbourOffset.Length;
            for(int i = 0; i < _visitingNodes.Count; i++)
            {
                Node node = _visitingNodes[i];
                if (node != endNode)
                {
                    calculateLos(node, endNode);
                }
                for (var j = 0; j < neighbourCount; j++)
                {
                   int x = node.x+ GridMap.neighbourOffset[j].x;
                   int z = node.z + GridMap.neighbourOffset[j].y; 
                   
                   if (_gridMap.IsWalkable(x,z))
                    {
                        if(_gridDijkstraCost[x, z] == FP.MinValue)
                        {
                            Node newNode;// = _nodesPool.New();
                            newNode.x = x;
                            newNode.z = z;
                            newNode.cost = GridMap.neighbourCost[1] + node.cost;//
                            _gridDijkstraCost[x, z] = newNode.cost;
                            _visitingNodes.Add(newNode);
                        }                     
                    }
                }
            }

            ResetVisitingNodes();
        }
        TSVector GetFlowField(int x, int z)
        {
            int idx = _gridMap.GetIdx(x, z);
            TSVector dir = TSVector.zero;
            dir = _flowField[x, z];
            return dir;
        }
        void UpdateNodeFlowFiled( int x, int z)
        {
            int neighbourCount = GridMap.neighbourOffset.Length;
            FP minDist = FP.MaxValue;
            TSVector min = TSVector.zero;
            bool bHasUnwalkable = false;
            for (var i = 0; i < neighbourCount; i++)
            {
                int n_x = GridMap.neighbourOffset[i].x + x;
                int n_z = GridMap.neighbourOffset[i].y + z;
                if(!IsValid(n_x,n_z))
                {
                    continue;
                }
                int idx1 = _gridMap.GetIdx(n_x, n_z);
                if( _gridMap.IsDynamicUnwalkableNode(idx1))
                {
                    bHasUnwalkable = true;
                }
                if (_gridMap.IsWalkable(n_x,n_z))
                {
                    FP dist = FP.Zero;
                    dist = _gridDijkstraCost[n_x, n_z] - _gridDijkstraCost[x, z];
                    if (dist < minDist)
                    {
                        int n_idx = _gridMap.GetIdx(n_x, n_z);
                        min = _gridMap.GetWorldPosition(n_idx);
                        minDist = dist;
                    }
                }           
            }
            // TSVector2 v = min - pos;
            int idx = _gridMap.GetIdx(x, z);
            TSVector vec3 = _gridMap.GetWorldPosition(idx);
            vec3 = min - vec3;    
            if(bHasUnwalkable)
            {
            }
            else
            {
#if UNITY_EDITOR && FlowFieldDebug
                if(_bDraw)
                {
                    Vector3 pos = CustomMath.TsVecToVector3(_gridMap.GetWorldPosition(z * _gridWidth + x) + _gridMap._startPos) / GridMap.SCALE.AsFloat();
                    Debug.DrawLine(pos, pos + CustomMath.TsVecToVector3(_flowField[x, z]) * 1, Color.cyan, 60);
                }             
#endif
            }
        }
#if UNITY_EDITOR && FlowFieldDebug
        List<GameObject> _pathValues = new List<GameObject>();
        bool _bDraw = false;
        void DrawFlowField()
        {
            int count = _pathValues.Count;
            for(int i=0;i<count;i++)
            {
                GameObject.Destroy(_pathValues[i]);
            }
            _pathValues.Clear();
            _bDraw = true;
            for (int i=0;i<_gridWidth;i++)
            {
                for (int j = 0; j < _gridHeight; j++)
                {
                    Vector3 pos =CustomMath.TsVecToVector3( _gridMap.GetWorldPosition(j*_gridWidth+i)+_gridMap._startPos)/GridMap.SCALE.AsFloat();
                    float val = _gridDijkstraCost[i, j].AsFloat();
                    //
                    GameObject go = GameObject.Instantiate<GameObject>(UnityEditor.AssetDatabase.LoadAssetAtPath<GameObject>("Assets/pathval.prefab"));
                    if (go != null)
                    {
                        _pathValues.Add(go);
                        pos.y = 0.01f;
                        go.transform.rotation = Quaternion.Euler(90, -90, 0);
                        go.transform.position = pos;
                        TextMesh tm = go.GetComponent<TextMesh>();
                        if(_gridDijkstraCost[i, j] ==FP.MaxValue)
                        {
                            tm.text ="Max";
                        }
                        else if(_gridDijkstraCost[i, j] == FP.MinValue)
                        {
                            tm.text = "Min";
                        }
                        else
                        {
                            tm.text = val.ToString();
                        }
                        //int idx = _gridMap.GetIdx(i,j);
                      //  _dicMesh[idx] = tm;
                    }
                    Debug.DrawLine(pos, pos + CustomMath.TsVecToVector3(_flowField[i, j]) * 1, Color.blue, 60);
                }
             
            }
           
        }
#endif
        void calculateLos(Node node,Node endNode)
        {
            int xDif = endNode.x - node.x;
            int zDif = endNode.z - node.z;

            int xDifAbs = Math.Abs(xDif);
            int zDifAbs = Math.Abs(zDif);

            bool hasLos = false;

            var xDifOne = Math.Sign(xDif);
            var zDifOne = Math.Sign(zDif);

            //Check in the x direction
            if (xDifAbs >= zDifAbs)
            {
                if (_gridLOS[node.x + xDifOne,node.z])
                {
                    hasLos = true;
                }
            }
            //Check in the y direction
            if (zDifAbs >= xDifAbs)
            {
                if (_gridLOS[node.x,node.z + zDifOne])
                {
                    hasLos = true;
                }
            }
            
            if (zDifAbs > 0 && xDifAbs > 0)
            {
                //diagonal doesn't have LOS
                if (!_gridLOS[node.x + xDifOne,node.z + zDifOne])
                {
                    hasLos = false;
                }
                else if (zDifAbs == xDifAbs)
                {
                    //
                    if (_gridDijkstraCost[node.x + xDifOne,node.z] == FP.MaxValue
                        || _gridDijkstraCost[node.x,node.z + zDifOne] == FP.MaxValue)
                    {
                        hasLos = false;
                    }
                }
            }
            _gridLOS[node.x,node.z] = hasLos;
        }


        internal TSVector steeringBehaviourFlowField(IAgentBehaviour agent)
        {
            //bilinear interpolation 
            TSVector vec = _gridMap.GetGridFloatCoord(agent.position);
            IInt2 floor = IInt2.zero;// GetNearestGridCoordWithoutClamp(agent.position);
            floor.x = TSMath.Floor(vec.x).AsInt();
            floor.y = TSMath.Floor(vec.z).AsInt();
            bool bIn1 = IsValid(floor.x, floor.y);
            TSVector f00 = (bIn1) ? GetFlowField(_gridMap.ClampX(floor.x), _gridMap.ClampZ(floor.y)) : TSVector.zero;//(posOrigin - pos1) ;// TSVector.zero
            int offsetX = f00.x > 0 ? 1 : -1;
            int offsetY = f00.z > 0 ? 1 : -1;
            bool bIn2 = IsValid(floor.x, floor.y + offsetY);
            bool bIn3 = IsValid(floor.x + offsetX, floor.y);
            bool bIn4 = IsValid(floor.x + offsetX, floor.y + offsetY);
            // bool bAllIn = bIn1 && bIn2 && bIn3 && bIn4;//bAllIn ||!

            //
            //TSVector posOrigin = (TSVector)vec;
            //TSVector pos1 = TSVector.zero;
            //pos1.Set((float)floor.x, 0, floor.y);
            //TSVector pos2 = TSVector.zero;
            //pos2.Set((float)floor.x, 0, floor.y + 1);
            //TSVector pos3 = TSVector.zero;
            //pos3.Set((float)floor.x + 1, 0, floor.y);
            //TSVector pos4 = TSVector.zero;
            //pos3.Set((float)floor.x + 1, 0, floor.y + 1);
            //TSVector f00 = (bAllIn) ? _flowField[ClampX(floor.x), ClampZ(floor.y)] : (bIn1?TSVector.zero: ((bIn2&& bIn3&& bIn4) ? TSVector._northEst:(bIn2?TSVector._forward:(bIn3?TSVector._right:TSVector.zero))));
            //TSVector f01 = (bAllIn) ? _flowField[ClampX(floor.x), ClampZ(floor.y + 1)] : (bIn2 ? TSVector.zero : ((bIn1 && bIn3 && bIn4) ? TSVector._southEst : (bIn1 ? -TSVector._forward : (bIn4 ? TSVector._right : TSVector.zero))));
            //TSVector f10 = (bAllIn) ? _flowField[ClampX(floor.x + 1), ClampZ(floor.y)] : (bIn3 ? TSVector.zero : ((bIn2 && bIn1 && bIn4) ? TSVector._northWest : (bIn4 ? TSVector._forward : (bIn1 ? -TSVector._right : TSVector.zero))));
            //TSVector f11 = (bAllIn) ? _flowField[ClampX(floor.x + 1), ClampZ(floor.y + 1)] : (bIn4 ? TSVector.zero : ((bIn2 && bIn3 && bIn1) ? TSVector._southWest : (bIn3 ? -TSVector._forward : (bIn2 ? -TSVector._right : TSVector.zero))));

      
           
            TSVector f01 = (bIn2) ? GetFlowField(_gridMap.ClampX(floor.x), _gridMap.ClampZ(floor.y + offsetY)) : TSVector.zero;//(posOrigin - pos2);
            TSVector f10 = (bIn3) ? GetFlowField(_gridMap.ClampX(floor.x + offsetX), _gridMap.ClampZ(floor.y)) : TSVector.zero;//(posOrigin - pos3);
            TSVector f11 = (bIn4) ? GetFlowField(_gridMap.ClampX(floor.x + offsetX), _gridMap.ClampZ(floor.y + offsetY)) : TSVector.zero;// (posOrigin - pos4);

            //Do the x interpolations
            FP fx = agent.position.x;
            FP fy = agent.position.z;
            FP w = FP.One* _gridWidth*GridMap.GetNodeSize();
            FP h= FP.One * _gridHeight * GridMap.GetNodeSize();
            FP dstX0 =TSMath.Abs(vec.x-(floor.x + CustomMath.FPHalf));
            FP dstX1 = TSMath.Abs(vec.x - (floor.x + offsetX + CustomMath.FPHalf));
            FP dstY0 = TSMath.Abs(vec.z - (floor.y + CustomMath.FPHalf));
            FP dstY1 = TSMath.Abs(vec.z - (floor.y + offsetY + CustomMath.FPHalf));

            FP xW = (dstX0) / (dstX0 + dstX1);
            FP xWeight = (fx < w && fx >= FP.Zero) ? (xW) : FP.Zero;//vec.x - floor.x

            TSVector top = f00 * (1 - xWeight) + f10 * (xWeight);
            TSVector bottom = f01 * (1 - xWeight) + f11 * xWeight;

            FP yW = (dstY0) / (dstY0 + dstY1);
            //Do the y interpolation
            FP yWeight = (fy < h && fy >= FP.Zero) ? (yW) : FP.Zero;//vec.z - floor.y
            //
            TSVector desiredDirection = (top * (1 - yWeight) + bottom * yWeight);
            //}
            // desiredDirection = desiredDirection + Boids.BoidsBehaviourTerrainSeparation(vec,IsValid);
            //desiredDirection = desiredDirection.normalized;
            // return Boids.SteerTowards(agent, desiredDirection);
            return desiredDirection;
        }

        List<IAgentBehaviour> _allAgents = new List<IAgentBehaviour>();
        internal void AddAgent(IAgentBehaviour agent)
        {
            _allAgents.Add(agent);
           // PathFindingManager.instance.AddAgent(agent);
        }
        internal void RemoveAgent(IAgentBehaviour agent)
        {
            _allAgents.Remove(agent);
           // PathFindingManager.instance.RemoveAgent(agent);
        }
        public int _updateIdx = 0;
   
        public void Loop(int frameTime)//ms
        {
            int iCount = _allAgents.Count;
           // int fDtSecond = PathFindingManager.instance.DeltaTime;// * 0.001f;
            _updateIdx = (_updateIdx+1) % PathFindingManager.C_FrameCountPerUpdate;
            for (int i = 0; i <iCount; i=i+1)
            {
                IAgentBehaviour agent = _allAgents[i];
                if(agent.agent!=null)
                {
                    agent.agent.Loop(frameTime,null,_updateIdx != i % PathFindingManager.C_FrameCountPerUpdate);
                }
            }
        }
    }
}