///author:huwei
///date:2018.4.17
//#define USING_ASTAR_PROJECT
//#define BattleInterface
#define USING_DYNAMIC_TREE
using TrueSync;
//using System.Reflection;
using AStarMachine;
using System;
using System.Collections.Generic;

namespace PathFinding
{
#if !BattleInterface
    public delegate bool IsTileUnWalkable(IInt2 pos);
#else
    public delegate bool IsTileUnWalkable(BattleInterface.Vector2 pos);
#endif
    public class GridMap : IAStarMap
    {
        public struct DynamicBlock
        {
            public int count;
            public bool isStationary;
        }
        
#if USING_DYNAMIC_TREE
        DynamicTree<int> _dynamicObstaclesTree = null;
#endif
        public TSVector _startPos = TSVector.zero;
        public int _id;
        public static FP nodeSize = 100;
        static FP _CheckFullPathDstSqr;
        static FP _blockDirDst = GridMap.SCALE * 6 * FP.EN1;
        public static FP SCALE = 1;
        public static FP _checkDeltaDst = FP.One;
        public static FP _checkDeltaDstSqr = FP.One;
        public const int c_dynamicRangeSqr = 25;
        public  static void SetNodeSize(FP value,FP scale)
        {
            nodeSize = value;
            SCALE = scale;
            _CheckFullPathDstSqr = nodeSize * AStarMachine.AStarMachine.C_MaxRevolutions*5;
            _CheckFullPathDstSqr *= _CheckFullPathDstSqr;
            _blockDirDst = nodeSize * 18 * FP.EN1;
            _checkDeltaDst = SCALE * FP.One * 2 / 10;
            _checkDeltaDstSqr = _checkDeltaDst * _checkDeltaDst;
        
        }
        public static FP GetNodeSize()
        {
            return nodeSize;
        }
        public static FP blockDirDst
        {
            get { return _blockDirDst; }
        }
        public static FP CheckFullPathDstSqr
        {
            get { return _CheckFullPathDstSqr; }
        }
        public IsTileUnWalkable _isUnWalkable = null;

        public static FP C_MaxCost = new FP(1000);
        //  public int enabledTags = -1;
        IInt2 _size = IInt2.zero;

        //Dictionary<int, int> _dicDynamicNodeBlock = new Dictionary<int, int>();
        HashSet<IInt2> _hsDynamicNodeToNodeCost = new HashSet<IInt2>();
        Dictionary<int, DynamicBlock> _hsDynamicUnwalkableNodes = new Dictionary<int, DynamicBlock>();//key:node idx;value: reference count
        public bool _obstacleHasSetUp = false;
        public static readonly IInt2[] neighbourOffset = new IInt2[]
        {
                new IInt2(-1,-1),
                new IInt2(-1,0),
                new IInt2(-1,1),
                new IInt2(0,-1),
                new IInt2(0,1),
                new IInt2(1,-1),
                new IInt2(1,0),
                new IInt2(1,1)
          };
        public static IInt2[] straightNeighbourOffset = new IInt2[]
          {
                new IInt2(-1,0),
                new IInt2(0,-1),
                new IInt2(1,0),
                new IInt2(0,1),
          };
        public static FP[] straightNeighbourCost = null;


        public static FP[] neighbourCost = null;
        static readonly FP s_ObStacle_radius=FP.One * CustomMath.FPHalf;
        public bool CheckNodeValid(AStarNode node)
        {
            return node != null && node.G < C_MaxCost;
        }


        public void Ini(int width, int height, TSVector startPos, int id, IsTileUnWalkable isWalkable)
        {
            //_obstacleHasSetUp = false;
            _size.x = width;
            _size.y = height;

            _startPos = startPos;
            _id = id;
            Reset();
            _isUnWalkable = isWalkable;
            //_pathId =pathId;
            //
            if (straightNeighbourCost == null)
            {
                straightNeighbourCost = new FP[]
                {
                     GridMap.GetNodeSize(),
                     GridMap.GetNodeSize(),
                     GridMap.GetNodeSize(),
                     GridMap.GetNodeSize()
                };
            }
            C_MaxCost = 1000 * GridMap.GetNodeSize();
            if (neighbourCost == null)
            {
                neighbourCost = new FP[]
                {
                    GridMap.GetNodeSize()*CustomMath.DiagonalCost,
                    GridMap.GetNodeSize(),
                    GridMap.GetNodeSize()*CustomMath.DiagonalCost,
                    GridMap.GetNodeSize(),
                    GridMap.GetNodeSize(),
                    GridMap.GetNodeSize()*CustomMath.DiagonalCost,
                    GridMap.GetNodeSize(),
                    GridMap.GetNodeSize()*CustomMath.DiagonalCost,
                };
            }
#if USING_DYNAMIC_TREE
            if (_dynamicObstaclesTree == null)
            {
                _dynamicObstaclesTree = new DynamicTree<int>();
                _dynamicObstaclesTree.c_aabbExtension = 0;// nodeSize;
            }
#endif
        }
        public bool IsBlockedByObstacleBetween2Point(TSVector pos1, TSVector pos2, Stack<int> stacktmp,ref TSVector blockPos)
        {
#if USING_DYNAMIC_TREE
            TSVector2 vec1 = CustomMath.TSVecToVec2(pos1);
            TSVector2 vec2 = CustomMath.TSVecToVec2(pos2);
            if (_dynamicObstaclesTree != null)
            {
                RayCastInput rc = new RayCastInput();
                rc.p1 = vec1;
                rc.p2 = vec2;
                rc.maxFraction = 1;// GridMap.GetNodeSize()*FP.One*FP.EN1*5;/// 20*nodeSize;
                stacktmp.Clear();
                int proxyId= _dynamicObstaclesTree.RayCast(null, ref rc, stacktmp, true);
                if(proxyId>=0)
                {
                   int gridNodeIdx= _dynamicObstaclesTree.GetUserData(proxyId);
                    blockPos= GetWorldPosition(gridNodeIdx);
                    return true;
                }
               
            }
            return false;
#else
            return false;
#endif
        }
        public void CalculateObstacleNeighbours(TSVector pos,FP radius, DynamicTreeQueryCallback callback, Stack<int> stacktmp)
        {
            AABB ab = new AABB();
            ab.lowerBound = new TSVector2(pos.x - radius, pos.z - radius);
            ab.upperBound = new TSVector2(pos.x + radius, pos.z + radius);
            _dynamicObstaclesTree.Query(callback, ref ab, stacktmp,false);
            stacktmp.Clear();
        }
        public CircleObstacle GetObstaclesByProxyId(int id)
        {
            int idx= _dynamicObstaclesTree.GetUserData(id);
            CircleObstacle obstacle = CircleObstacle.zero;
            obstacle._center =CustomMath.TSVecToVec2(GetWorldPosition(idx));
            obstacle._radius = s_ObStacle_radius * GridMap.GetNodeSize();
            return obstacle;
        }
        public bool CalculateObstaclesBetween2Point(TSVector pos1, TSVector pos2, DynamicTreeRayCastCallback callBack, Stack<int> stacktmp)
        {
#if USING_DYNAMIC_TREE
            TSVector2 vec1 = CustomMath.TSVecToVec2(pos1);
            TSVector2 vec2 = CustomMath.TSVecToVec2(pos2);
            if (_dynamicObstaclesTree != null)
            {
                RayCastInput rc = new RayCastInput();
                rc.p1 = vec1;
                rc.p2 = vec2;
                rc.maxFraction = FP.One;/// 20*nodeSize;
                int proxyId= _dynamicObstaclesTree.RayCast(callBack, ref rc, stacktmp, false);
               // _dynamicObstaclesTree.GetUserData(proxyId)
            }
            return false;
#else
            return false;
#endif
        }
#if USING_DYNAMIC_TREE
        public int AddObstacle(int idx)
        {
            //if(_obstacleHasSetUp)
            //{
            //    return 0;
            //}            
            TSVector pos = GetWorldPosition(idx);
            AABB ab = new AABB();
            TSVector2 vec = CustomMath.TSVecToVec2(pos);
            ab.lowerBound = vec - TSVector2.one * (s_ObStacle_radius*GridMap.GetNodeSize());
            ab.upperBound = vec + TSVector2.one * (s_ObStacle_radius * GridMap.GetNodeSize() );
            return _dynamicObstaclesTree.CreateProxy(ref ab, idx);
        }
		 public void AddObstacle(int x,int z)
        {
            int idx = GetIdx(x, z);
            AddObstacle(idx);
        }
        public void RemoveObstacle(int proxtId)
        {
            if (proxtId >= 0)
            {
                _dynamicObstaclesTree.DestroyProxy(proxtId);
            }
        }
#endif
        public int GetNeighboursCount(AStarNode aStarNode)
        {
            return neighbourOffset.Length;
        }
        public bool IsInGridArea(int x, int y)
        {
            return x >= 0 && y >= 0 && x < _size.x && y < _size.y;
        }
        public bool IsInGridArea(int idx)
        {
            int x = idx %_size.x;
            int y=  idx / _size.x;
            return x >= 0 && y >= 0 && x < _size.x && y < _size.y;
        }
#region common method
#if USING_DYNAMIC_TREE
#endif
        //
        public bool IsWalkable(TSVector pos,bool bSkip)
        {
            IInt2 iPos = GetNearestGridCoordWithoutClamp(pos);
            return IsWalkable(iPos.x, iPos.y, bSkip);
        }
        
        public bool IsExistingNeighbourWalkable(int x,int y,int startIdx,int endIdx,int curPosIdx)
        {
            int count = neighbourOffset.Length;
            int idx0 = GetIdx(x,y);
#if !DYNAMIC_FORCE
            IInt2 curPos = GetGridPos(curPosIdx);
#endif
            for (short i = 0; i < count; i++)
            {
                int idx = GetIdx(x + neighbourOffset[i].x, y + neighbourOffset[i].y);
                bool bSkip = false;
#if !DYNAMIC_FORCE
                IInt2 pos = IInt2.zero;
                pos.x = x + neighbourOffset[i].x;
                pos.y = y + neighbourOffset[i].y;
                IInt2 diff = curPos - pos;

                EDynamicBlockStatus status1 = DynamicBlockedStatus(idx, startIdx, endIdx, curPosIdx);
                bSkip = (status1 == EDynamicBlockStatus.blocked && diff.sqrMagnitudeInt > GridMap.c_dynamicRangeSqr)
                    ||(status1== EDynamicBlockStatus.ignore)
                    || (status1 == EDynamicBlockStatus.none);
#endif
                bool bWalkable = IsWalkable(x + neighbourOffset[i].x, y + neighbourOffset[i].y
                    , bSkip);
                if (!PathFindingManager.StopToNodeCenter)
                {
                    if (!IsDiagonalBlocked(idx0,idx, startIdx, endIdx, curPosIdx)  &&bWalkable)
                    {
                        return true;
                    }
                }
                else
                {
                    if(bWalkable )
                    {
                        return true;
                    }
                }                
            }
            return false;
        }
        public bool IsDiagonalBlocked(int idx1, int idx2,int startIdx,int endIdx,int curPosIdx)
        {
            //if(PathFindingManager.c_useAvoidUnit)
            //{
            //    return false;
            //}
            //return false;
            IInt2 pos1 = GetGridPos(idx1);
            IInt2 pos2 = GetGridPos(idx2);
            IInt2 diff = pos1 - pos2;
            if (Math.Abs(diff.x) <= 1 && Math.Abs(diff.y)<=1)
            {
                int diagonalIdx1 = GetIdx(pos1.x, pos2.y);
                int diagonalIdx2 = GetIdx(pos2.x, pos1.y);
               

                EDynamicBlockStatus status1 = DynamicBlockedStatus(diagonalIdx1, startIdx, endIdx, curPosIdx);
                EDynamicBlockStatus status2 = DynamicBlockedStatus(diagonalIdx2, startIdx, endIdx, curPosIdx);
              
                if(status1== EDynamicBlockStatus.stationaryBlocked
                    ||status2== EDynamicBlockStatus.stationaryBlocked)
                {
                    return true;
                }
                IInt2 curPos = GetGridPos(curPosIdx);
                IInt2 diff1 = curPos - pos1;

                bool bSkip = diff1.sqrMagnitudeInt > c_dynamicRangeSqr;
                bool bSkip1 = bSkip || status1== EDynamicBlockStatus.ignore || (status1 == EDynamicBlockStatus.none);
                bool bSkip2 = bSkip || status2== EDynamicBlockStatus.ignore || (status2 == EDynamicBlockStatus.none);
                return !IsWalkable(pos1.x,pos2.y,bSkip1)
                    || (!IsWalkable(pos2.x, pos1.y, bSkip2));
            }
            return false;
        }
       
        static readonly FP r_f21= CustomMath.FPHalf;
        static readonly FP r_f41 = FP.One/4;
        /// <summary>
        /// 
        /// </summary>
        /// <param name="position"></param>
        /// <param name="radius"></param>
        /// <param name="nodes"></param>
        //        public void AddDynamicNodesCost(TSVector position,FP radius, HashSet<int> nodes)
        //        {
        //            //position = position - _startPos;
        //            FP val = (position.x + radius) / nodeSize;
        //            FP rVal = TSMath.Floor(val);
        //            //bool bMaxXHalf=
        //            if (val-rVal< r_f21)
        //            {
        //                rVal = rVal - FP.One;
        //            }
        //            int maxX = ClampX(rVal.AsInt());
        //            //
        //            val = (position.z + radius) / nodeSize;
        //            rVal = TSMath.Floor(val);
        //            if (val - rVal < r_f21)
        //            {
        //                rVal = rVal - FP.One;
        //            }
        //            int maxZ = ClampZ(rVal.AsInt());
        //            //
        //            val = (position.x - radius) / nodeSize;
        //            rVal = TSMath.Floor(val);
        //            if (val-rVal > FP.One - r_f21)
        //            {
        //                rVal = rVal + FP.One;
        //            }
        //            int minX = ClampX(rVal.AsInt());
        //            //
        //            val = (position.z - radius) / nodeSize;
        //            rVal = TSMath.Floor(val);
        //            if (val - rVal >FP.One- r_f21)
        //            {
        //                rVal = rVal + FP.One;
        //            }
        //            int minZ = ClampZ(rVal.AsInt());
        //            for(int i=minX;i<=maxX;i++)
        //            {
        //                for(int j=minZ;j<=maxZ;j++)
        //                {
        //                    int idx = GetIdx(i, j);
        //                    nodes.Add(idx);
        //                    _dicDynamicNodeBlock[idx] = 1;
        //#if UNITY_EDITOR
        //                    if(PathFindingManager.DEBUG)
        //                    {
        //                        TSVector vec = GetWorldPosition(idx);
        //                        UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(vec);
        //                        UnityEngine.Debug.DrawLine(pos, pos + UnityEngine.Vector3.forward * 0.5f, UnityEngine.Color.black, 60);
        //                    }              
        //#endif
        //                }
        //            }
        //        }
        public void UpdateDynamicUnwalkableNodes(int node1, bool bAdd, bool isStationary)
        {
            if (PathFindingManager.c_useAvoidUnit)
            {
                return;
            }
          
            DynamicBlock rCount;
            if (bAdd)
            {
                if (!_hsDynamicUnwalkableNodes.TryGetValue(node1, out rCount))
                {
                    rCount = new DynamicBlock();
                }
                rCount.count++;
                if (isStationary)
                {
                    rCount.isStationary = isStationary;
                }
                _hsDynamicUnwalkableNodes[node1] = rCount;
#if UNITY_EDITOR && !MULTI_THREAD
                if (PathFindingManager.DEBUG)
                {
                    TSVector position = GetWorldPosition(node1);
                    UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(position + _startPos) / nodeSize.AsFloat();
                    UnityEngine.Debug.DrawLine(pos - UnityEngine.Vector3.forward * 0.25f, pos + UnityEngine.Vector3.forward * 0.25f, UnityEngine.Color.red, 60);
                }
#endif
            }
            else
            {
                if (_hsDynamicUnwalkableNodes.TryGetValue(node1, out rCount))
                {
                    if (rCount.count > 1)
                    {
                        rCount.count--;
                        _hsDynamicUnwalkableNodes[node1] = rCount;
                    }
                    else
                    {
                        _hsDynamicUnwalkableNodes.Remove(node1);
#if UNITY_EDITOR && !MULTI_THREAD
                        if (PathFindingManager.DEBUG)
                        {
                            TSVector position = GetWorldPosition(node1);
                            UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(position + _startPos) / nodeSize.AsFloat();
                            UnityEngine.Debug.DrawLine(pos - UnityEngine.Vector3.forward * 0.25f, pos + UnityEngine.Vector3.forward * 0.25f, UnityEngine.Color.white, 60);
                        }
#endif
                        // IInt2 pos0 = GetGridPos(node1);
                        //  FlowFieldManager.instance.UpdateBlockedNodeNeighbours(pos0.x, pos0.y, bAdd);
                    }
                }
            }

        }
        public void UpdateDynamicUnwalkableNodes(TSVector position,  bool bAdd,bool isStationary)
        {
            if(PathFindingManager.c_useAvoidUnit)
            {
                return;
            }      
            int node1 = GetGridNodeId(position);
            UpdateDynamicUnwalkableNodes(node1, bAdd, isStationary);
        }
        public bool IsDynamicUnwalkableNode(int node1)
        {
            if(PathFindingManager.c_useAvoidUnit)
            {
                return false;
            }
            // int node1 = GetGridNodeId(position);
            return _hsDynamicUnwalkableNodes.ContainsKey(node1);
        }
        void UpdateNodeToNodeCost(int node1,int node2, bool bAdd)
        {
          //  if ()//unit occupy mid pos
            {
                IInt2 nodeToNode = IInt2.zero;
                nodeToNode.x = node1;// GetIdx(iI, iJ);
                nodeToNode.y = node2;// GetIdx(iI, iJ - 1);
                IInt2 nodeToNode1 = IInt2.zero;
                nodeToNode1.x = node2;// GetIdx(iI, iJ);
                nodeToNode1.y = node1;
                if (bAdd)
                {
                    _hsDynamicNodeToNodeCost.Add(nodeToNode);
                    _hsDynamicNodeToNodeCost.Add(nodeToNode1);
#if UNITY_EDITOR && !MULTI_THREAD
                    if(PathFinding.PathFindingManager.DEBUG)
                    {
                        TSVector pos1= GetWorldPosition(node1);
                        TSVector pos2 = GetWorldPosition(node2);
                        UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(pos1) , CustomMath.TsVecToVector3(pos2), UnityEngine.Color.yellow, 60);
                    }
#endif
                }
                else
                {
                    _hsDynamicNodeToNodeCost.Remove(nodeToNode);
                    _hsDynamicNodeToNodeCost.Remove(nodeToNode1);
                }
            }
        }

        //        public void UpdateDynamicNodesCostPre(TSVector position, FP radius,bool bAdd)
        //        {
        //            FP maxX = (position.x + radius) / nodeSize;
        //            FP rMaxX = TSMath.Floor(maxX);
        //            //
        //            FP maxZ = (position.z + radius) / nodeSize;
        //            FP rMaxZ = TSMath.Floor(maxZ);

        //            FP minX = (position.x - radius) / nodeSize;
        //            FP rMinX = TSMath.Floor(minX);
        //            //
        //            FP minZ = (position.z - radius) / nodeSize;
        //            FP rMinZ = TSMath.Floor(minZ);
        //            FP radiusSqr = radius * radius;
        //            int mIdx=GetGridNodeId(position);
        //            for (FP i = rMinX; i <= rMaxX; i+=FP.One)
        //            {
        //                FP xDiff = FP.Zero;
        //                FP xMid = i + CustomMath.FPHalf * nodeSize;
        //                if(i==rMinX)
        //                {
        //                    xDiff = rMinX + FP.One-minX;                    
        //                }
        //                else if(i==rMaxX)
        //                {
        //                    xDiff = maxX - rMaxX;
        //                }
        //                else//
        //                {
        //                    xDiff = FP.One;
        //                }
        //                for (FP j = rMinZ; j <= rMaxZ; j += FP.One)
        //                {
        //                    FP zMid = j + CustomMath.FPHalf * nodeSize;
        //                    FP zDiff = FP.Zero;
        //                    if (j <= rMinZ)
        //                    {
        //                        zDiff = rMinZ + FP.One - minZ;
        //                    }
        //                    else if (j >= rMaxZ)
        //                    {
        //                        zDiff = maxZ - rMaxZ;
        //                    }
        //                    else//
        //                    {
        //                        zDiff = FP.One;
        //                    }
        //                    int iI = i.AsInt();
        //                    int iJ = j.AsInt();

        //                    FP xMDiff = position.x - xMid;
        //                    FP zMDiff = position.z - j;
        //                    int node1 = 0;
        //                    int node2 = 0;
        //                    if (xMDiff * xMDiff + zMDiff * zMDiff < radiusSqr)
        //                    {
        //                        node1 = GetIdx(iI, iJ);
        //                        node2 = GetIdx(iI, iJ - 1);
        //                        UpdateNodeToNodeCost(node1, node2, bAdd);
        //                    }
        //                    //
        //                    zMDiff = position.z - zMid;
        //                    xMDiff = position.x - i;
        //                    if (xMDiff * xMDiff + zMDiff * zMDiff < radiusSqr)
        //                    {
        //                        node1 = GetIdx(iI - 1, iJ);
        //                        node2 = GetIdx(iI, iJ);
        //                        UpdateNodeToNodeCost(node1, node2, bAdd);
        //                    }
        //                    //
        //                    xMDiff = position.x - i;
        //                    zMDiff = position.z - j;

        //                    if (xMDiff * xMDiff + zMDiff * zMDiff < radiusSqr)
        //                    {
        //                        node1 = GetIdx(iI, iJ);
        //                        node2 = GetIdx(iI - 1, iJ - 1);
        //                        UpdateNodeToNodeCost(node1, node2, bAdd);
        //                        node1 = GetIdx(iI - 1, iJ);
        //                        node2 = GetIdx(iI, iJ - 1);
        //                        UpdateNodeToNodeCost(node1, node2, bAdd);
        //                    }
        //                    //

        //                    int idx = GetIdx(i.AsInt(), j.AsInt());
        //                    FP cost = (xDiff * zDiff);
        //                    //if (xDiff >=CustomMath.FPHalf && zDiff >= CustomMath.FPHalf)
        //                    //{
        //                    //    cost = C_MaxCost;
        //                    //}
        //                    if (mIdx == idx)
        //                    {
        //                        cost = C_MaxCost;
        //                    }
        //                    FP preCost = FP.Zero;
        //                    _dicDynamicNodeCost.TryGetValue(idx, out preCost);
        //                    cost = preCost+ (bAdd?cost:-cost);
        //                    cost = TSMath.Max(FP.Zero, cost);
        //                    if(cost==FP.Zero && preCost!=FP.Zero)
        //                    {
        //                        _dicDynamicNodeCost.Remove(idx);
        //                    }
        //                    else
        //                    {
        //                        _dicDynamicNodeCost[idx] = cost;
        //                    }
        //#if UNITY_EDITOR
        //                    if(cost>900 && cost<1000)
        //                    {
        //                        UnityEngine.Debug.Log("cost error:"+ cost);
        //                    }
        //                    if(PathFindingManager.DEBUG)
        //                    {
        //                        TSVector vec = GetWorldPosition(idx);
        //                        UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(vec);
        //                        UnityEngine.TextMesh tm = null;
        //                        if (!_dicExtraCost.TryGetValue(idx, out tm) && bAdd)
        //                        {
        //                            UnityEngine.GameObject go = UnityEngine.GameObject.Instantiate<UnityEngine.GameObject>(UnityEditor.AssetDatabase.LoadAssetAtPath<UnityEngine.GameObject>("Assets/pathval.prefab"));
        //                            if (go != null)
        //                            {
        //                                pos.y = 0.01f;
        //                                //go.transform.rotation = UnityEngine.Quaternion.Euler(90, -90, 0);
        //                                go.transform.position = pos;
        //                                go.transform.SetParent(UnityEngine.GameObject.Find("path_values").transform);
        //                                tm = go.GetComponent<UnityEngine.TextMesh>();
        //                                _dicExtraCost[idx] = tm;
        //                            }
        //                        }
        //                        if (tm != null)
        //                        {
        //                            tm.text = cost > C_MaxCost ? "1000" : string.Format("{0:n1}", cost.AsFloat());
        //                            UnityEngine.Debug.DrawLine(pos - UnityEngine.Vector3.forward * 0.25f, pos + UnityEngine.Vector3.forward * 0.25f, UnityEngine.Color.black, 60);

        //                        }
        //                    }                   
        //#endif
        //                }
        //            }
        //        }
        int[] _tmp = new int[8];
        int[] _tmpIdx = new int[8];
        static readonly int[] c_blockNum =new int[]{ 1<<0, 1 << 1, 1 << 2, 1 << 3, 1 << 4, 1 << 5, 1 << 6, 1 << 7 };
        static readonly FP c_41 = FP.One / 4;
        static readonly FP c_81 = FP.One / 8;
        /// <summary>
        /// 
        /// </summary>
        /// <param name="position"></param>
        /// <param name="radius"></param>
        /// <param name="nodesCost"></param>
        //        public void UpdateDynamicNodesCost(TSVector position, FP radius, bool bAdd)
        //        {
        //            position = position - _startPos;
        //            FP maxX = (position.x + radius) / nodeSize;
        //            FP rMaxX = TSMath.Floor(maxX);
        //            //
        //            FP maxZ = (position.z + radius) / nodeSize;
        //            FP rMaxZ = TSMath.Floor(maxZ);

        //            FP minX = (position.x - radius) / nodeSize;
        //            FP rMinX = TSMath.Floor(minX);
        //            //
        //            FP minZ = (position.z - radius) / nodeSize;
        //            FP rMinZ = TSMath.Floor(minZ);
        //            FP radiusSqr = radius * radius;
        //            int mIdx = GetGridNodeId(position);
        //            for (FP i = rMinX; i <= rMaxX; i += FP.One)
        //            {               
        //                FP x41 = i + c_41 * nodeSize;
        //                FP x43 = i - c_41 * nodeSize+FP.One;
        //                FP xDiff = FP.Zero;
        //                for (FP j = rMinZ; j <= rMaxZ; j += FP.One)
        //                {
        //                    FP z41 = j + c_41 * nodeSize;
        //                    FP z43 = j - c_41 * nodeSize + FP.One;
        //                    int iI = i.AsInt();
        //                    int iJ = j.AsInt();
        //                    int idx = GetIdx(iI, iJ);

        //                    /*split node to four sub nodes
        //                     * 3  2
        //                     * 0  1
        //                     * */
        //                    int blocked =0;

        //                    if(TSMath.Abs(position.x-x41)< radius+c_81 && TSMath.Abs(position.z - z41) < radius + c_81)
        //                    {
        //                        blocked = 1;
        //                    }
        //                    if (TSMath.Abs(position.x - x43) < radius + c_81 && TSMath.Abs(position.z - z41) < radius + c_81)
        //                    {
        //                        blocked += 10;
        //                    }
        //                    if (TSMath.Abs(position.x - x43) < radius + c_81 && TSMath.Abs(position.z - z43) < radius + c_81)
        //                    {
        //                        blocked += 100;
        //                    }
        //                    if (TSMath.Abs(position.x - x41) < radius + c_81 && TSMath.Abs(position.z - z43) < radius + c_81)
        //                    {
        //                        blocked += 1000;
        //                    }
        //                    if (blocked==0)
        //                    {
        //                        continue;
        //                    }
        //                    int blockVal = 0;
        //                    _dicDynamicNodeBlock.TryGetValue(idx, out blockVal);

        //                    if(bAdd)
        //                    {
        //                        int bVal = blockVal;
        //                        while (bVal >= 1)
        //                        {
        //                            if (bVal % 10 == 9)
        //                            {

        //                                break;
        //                            }
        //                            else
        //                            {
        //                                bVal = bVal / 10;
        //                            }
        //                        }
        //                        if (bVal == 0)//not overflow
        //                        {
        //                            blockVal = blockVal + blocked;
        //                            _dicDynamicNodeBlock[idx] = blockVal;
        //                        }
        //                        else
        //                        {
        //#if UNITY_EDITOR
        //                            if (PathFinding.PathFindingManager.DEBUG)
        //                            {
        //                                UnityEngine.Debug.LogError("block over flow");
        //                            }
        //#endif
        //                            continue;
        //                        }
        //                    }
        //                    else
        //                    {
        //                        blockVal = blockVal - blocked;
        //                        if (blockVal <= 0)
        //                        {
        //                            _dicDynamicNodeBlock.Remove(idx);
        //                            _dicDynamicNodeBlock[idx] = 0;
        //                        }
        //                        else
        //                        {
        //                            _dicDynamicNodeBlock[idx] = blockVal;
        //                        }
        //                    }

        //                    if (blocked==1111)//all
        //                    {

        //                        if (bAdd)
        //                        {
        //                            _hsDynamicUnwalkableNodes.Add(idx);                           
        //                        }
        //                        else
        //                        {
        //                            _hsDynamicUnwalkableNodes.Remove(idx);
        //                        }
        //                    }
        //                    else
        //                    {
        //                        int neighbourIdx = 0;
        //                        int m, n, nIdx = 0;
        //                        for (n =  iI - 1; n <= iI + 1; n++)
        //                        {
        //                            m = iJ - 1;                            
        //                            nIdx= GetIdx(n, m);
        //                            _tmpIdx[neighbourIdx++] = nIdx;                    
        //                        }
        //                        n = iI + 1;
        //                        m = iJ;
        //                        nIdx = GetIdx(n, m);
        //                        _tmpIdx[neighbourIdx++] =nIdx;
        //                        n = iI + 1;
        //                        m = iJ+1;
        //                        nIdx = GetIdx(n, m);
        //                        _tmpIdx[neighbourIdx++] = nIdx;
        //                        for (n = iI +1; n >= iI - 1; n--)
        //                        {
        //                            m = iJ + 1;
        //                            nIdx = GetIdx(n, m);
        //                            _tmpIdx[neighbourIdx++] = nIdx;
        //                        }
        //                        int len = _tmpIdx.Length;
        //                        for(int k=0;k< len;k++)
        //                        {
        //                            int bVal;
        //                            if (_dicDynamicNodeBlock.TryGetValue(_tmpIdx[k], out bVal))
        //                            {
        //                                _tmp[k] = bVal;
        //                            }
        //                            else
        //                            {
        //                                _tmp[k] = 0;
        //                            }
        //                        }

        //                        if (bAdd)
        //                        {

        //                        }
        //                        else
        //                        {
        //#if UNITY_EDITOR
        //                            if(PathFinding.PathFindingManager.DEBUG && !_dicDynamicNodeBlock.ContainsKey(idx) && _dicDynamicNodeBlock.Count>0)
        //                            {
        //                                UnityEngine.Debug.LogError("_dicDynamicNodeBlock error:" + idx);
        //                            }
        //#endif

        //                        }
        //                       // int val;
        //                        for(int s=0;s<4;s++)
        //                        {
        //                            if(blocked / c_blockNum[s] % 10 == 0 )
        //                            {
        //                                continue;
        //                            }
        //                            int idx1 = _tmpIdx[(s*2+7)%8];                            
        //                            if ((blocked/c_blockNum[(s+3)%4] % 10) > 0 
        //                                || (_tmp[(s*2+5)% 8] > 0 && (_tmp[(s * 2 + 5) % 8]/ c_blockNum[s % 4] % 10) > 0) 
        //                                || (_tmp[(s * 2 + 6) % 8] > 0 && (_tmp[(s * 2 + 6) % 8] / c_blockNum[(s+1) % 4] % 10) > 0) 
        //                                || (_tmp[(s * 2 + 7) % 8] >0&& (_tmp[(s * 2 + 7) % 8] / c_blockNum[(s + 2) % 4] % 10) > 0 ))
        //                            {
        //                                UpdateNodeToNodeCost(idx, idx1, bAdd);
        //                            }
        //                            idx1 = _tmpIdx[(s * 2 + 8) % 8];
        //                            //if ( (_tmp[(s * 2 + 8) % 8] > 0 && (_tmp[(s * 2 + 8) % 8] / c_blockNum[(s + 2) % 4] % 10) > 0 )
        //                            //    ||( _tmp[(s * 2 + 1) % 8] > 0 && (_tmp[(s * 2 + 1) % 8] / c_blockNum[(s + 3) % 4] % 10) > 0 
        //                            //    ||_tmp[(s * 2 + 7) % 8] > 0 && (_tmp[(s * 2 + 7) % 8] / c_blockNum[(s + 1) % 4] % 10) > 0))
        //                            {
        //                                UpdateNodeToNodeCost(idx, idx1, bAdd);
        //                            }
        //                            idx1 = _tmpIdx[(s * 2 + 1) % 8];
        //                            if ((blocked / c_blockNum[(s + 1) % 4] % 10) > 0 
        //                                || (_tmp[(s * 2 + 1) % 8] > 0 && (_tmp[(s * 2 + 1) % 8] / c_blockNum[(s + 2) % 4] % 10) > 0) 
        //                                || (_tmp[(s * 2 + 2) % 8] > 0 && (_tmp[(s * 2 + 2) % 8] / c_blockNum[(s + 3) % 4] % 10) > 0) 
        //                                || (_tmp[(s * 2 + 3) % 8] > 0 && (_tmp[(s * 2 + 3) % 8]/ c_blockNum[(s) % 4] % 10) > 0))
        //                            {
        //                                UpdateNodeToNodeCost(idx, idx1, bAdd);
        //                            }                           
        //                        }

        //                    }                    

        //#if UNITY_EDITOR

        //                    if (PathFindingManager.DEBUG && bAdd)
        //                    {
        //                        TSVector vec = GetWorldPosition(idx);
        //                        UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(vec);
        //                        UnityEngine.TextMesh tm = null;
        //                        if (!_dicExtraCost.TryGetValue(idx, out tm) && bAdd)
        //                        {
        //                            UnityEngine.GameObject go = UnityEngine.GameObject.Instantiate<UnityEngine.GameObject>(UnityEditor.AssetDatabase.LoadAssetAtPath<UnityEngine.GameObject>("Assets/pathval.prefab"));
        //                            if (go != null)
        //                            {
        //                                pos.y = 0.01f;
        //                                //go.transform.rotation = UnityEngine.Quaternion.Euler(90, -90, 0);
        //                                go.transform.position = pos;
        //                                go.transform.SetParent(UnityEngine.GameObject.Find("path_values").transform);
        //                                tm = go.GetComponent<UnityEngine.TextMesh>();
        //                                _dicExtraCost[idx] = tm;
        //                            }
        //                        }
        //                        if (tm != null)
        //                        {
        //                            tm.text = blockVal.ToString();
        //                            UnityEngine.Debug.DrawLine(pos - UnityEngine.Vector3.forward * 0.25f, pos + UnityEngine.Vector3.forward * 0.25f, UnityEngine.Color.black, 60);

        //                        }
        //                    }
        //#endif
        //                }
        //            }
        //        }
        //public FP GetExtraCost(int idx)
        //{
        //    FP val = FP.Zero;
        //    _dicDynamicNodeBlock.TryGetValue(idx, out val);
        //    return val;
        //}
        //public void RemoveDynamicBlockedNodes(Dictionary<int, FP> nodesCost)
        //{
        //    foreach(KeyValuePair<int,FP> kv in _dicDynamicNodeCost)
        //    _dicDynamicNodeCost
        //}

        /// <summary>
        /// 
        /// </summary>
        /// <param name="position"></param>
        /// <param name="radius"></param>
        /// <param name="nodesCost"></param>
//        public void UpdateDynamicNodesCost(TSVector position, FP radius, bool bAdd)
//        {
//            TSVector gFpos = GetGridFloatCoord(position);
//            FP floorX = TSMath.Floor(gFpos.x);
//            FP floorZ = TSMath.Floor(gFpos.z);
//            IInt2 iPos = IInt2.zero;
//            GetGridCoord(position, ref iPos);
//            int idx = GetIdx(iPos.x,iPos.y);
//            int idxBlockVal = 0;
            
//            if(position.x-floorX==CustomMath.FPHalf && position.z - floorZ == CustomMath.FPHalf)
//            {
//                UpdateDynamicUnwalkableNodes(position, bAdd);
//            }
//            else if(position.x - floorX == CustomMath.FPHalf)
//            {
//                _dicDynamicNodeBlock.TryGetValue(idx, out idxBlockVal);
//                idxBlockVal =bAdd?(idxBlockVal + c_blockNum[1]):(idxBlockVal -c_blockNum[1]);
//                _dicDynamicNodeBlock[idx] = idxBlockVal;
//                int blockVal = 0;
//                //bottom node check
//                int bottomY = iPos.y - 1;
//                int idxBottom = GetIdx(iPos.x, bottomY);
//                UpdateNodeToNodeCost(idx, idxBottom, bAdd);
//                int idxBottomLeft = GetIdx(iPos.x - 1, bottomY);
//                UpdateNodeToNodeCost(idxBottomLeft, idx, bAdd);
//                int idxBottomRight = GetIdx(iPos.x +1, bottomY);
//                UpdateNodeToNodeCost(idxBottomRight, idx, bAdd);

//                _dicDynamicNodeBlock.TryGetValue(idxBottom, out blockVal);
//                _dicDynamicNodeBlock[idxBottom]= bAdd ? (idxBlockVal + c_blockNum[5]) : (idxBlockVal - c_blockNum[5]);
//                if (bottomY > 0)
//                {
//                    bottomY = bottomY - 1;                 
//                    idxBottom = GetIdx(iPos.x, bottomY);
//                    if (_dicDynamicNodeBlock.TryGetValue(idxBottom,out blockVal))
//                    {
//                        if((blockVal & c_blockNum[6]) > 0 || (blockVal & c_blockNum[7]) > 0)
//                        {
//                            if (iPos.x - 1 >= 0)
//                            {
//                                idxBottomLeft = GetIdx(iPos.x - 1, bottomY);
//                                UpdateNodeToNodeCost(idxBottom, idxBottomLeft, bAdd);
//                            }
//                        }
//                        if ((blockVal & c_blockNum[3]) > 0 || (blockVal & c_blockNum[4]) > 0)
//                        {
//                            if (iPos.x < _size.x)
//                            {
//                                idxBottomRight = GetIdx(iPos.x + 1, bottomY);
//                                UpdateNodeToNodeCost(idxBottom, idxBottomRight, bAdd);
//                            }
//                        }
//                    }
//                }
//                else
//                {
//                    if(iPos.x - 1>=0)
//                    {
//                        idxBottomLeft = GetIdx(iPos.x - 1, bottomY);
//                        UpdateNodeToNodeCost(idxBottom, idxBottomLeft, bAdd);               
//                    }                  
//                    else if(iPos.x < _size.x)
//                    {
//                        idxBottomRight = GetIdx(iPos.x + 1, bottomY);
//                        UpdateNodeToNodeCost(idxBottom, idxBottomRight, bAdd);
//                    }
//                }
//                //top node check
//                int topY = iPos.y;
//                int idxTop = idx;
//                int idxTopLeft = GetIdx(iPos.x - 1, topY);
//                UpdateNodeToNodeCost(idxTopLeft, idxBottom, bAdd);
//                int idxTopRight = GetIdx(iPos.x + 1, topY);
//                UpdateNodeToNodeCost(idxTopRight, idxBottom, bAdd);
//                if (topY < _size.y-1)
//                {
//                    topY = topY + 1;
//                    idxTop = GetIdx(iPos.x, topY);
//                    if (_dicDynamicNodeBlock.TryGetValue(idxTop, out blockVal))
//                    {
//                        if ((blockVal & c_blockNum[0]) > 0 || (blockVal & c_blockNum[7]) > 0)
//                        {
//                            if (iPos.x - 1 >= 0)
//                            {
//                                int idxLeft = GetIdx(iPos.x - 1, topY);
//                                UpdateNodeToNodeCost(idxTop, idxLeft, bAdd);
//                            }
//                        }
//                        if ((blockVal & c_blockNum[3]) > 0 || (blockVal & c_blockNum[2]) > 0)
//                        {
//                            if (iPos.x < _size.x)
//                            {
//                                int idxRight = GetIdx(iPos.x + 1, bottomY);
//                                UpdateNodeToNodeCost(idxTop, idxRight, bAdd);
//                            }
//                        }
//                    }
//                }
//                else
//                {
//                    if (iPos.x - 1 >= 0)
//                    {
//                        int idxLeft = GetIdx(iPos.x - 1, topY);
//                        UpdateNodeToNodeCost(idxTop, idxLeft, bAdd);
//                    }
//                    else if (iPos.x < _size.x)
//                    {
//                        int idxRight = GetIdx(iPos.x + 1, topY);
//                        UpdateNodeToNodeCost(idxTop, idxRight, bAdd);
//                    }
//                }

//            }
//            else if (position.z - floorZ == CustomMath.FPHalf)
//            {
//                _dicDynamicNodeBlock.TryGetValue(idx, out idxBlockVal);
//                idxBlockVal = bAdd ? (idxBlockVal + c_blockNum[7]) : (idxBlockVal - c_blockNum[7]);
//                _dicDynamicNodeBlock[idx] = idxBlockVal;
//                int blockVal = 0;
//                //left node check
//                int leftX = iPos.x - 1;
//                int idxLeft = GetIdx(leftX, iPos.y );
//                UpdateNodeToNodeCost(idx, idxLeft, bAdd);
//                int idxLeftBottom = GetIdx(idxLeft, iPos.y-1 );
//                UpdateNodeToNodeCost(idxLeftBottom, idx, bAdd);
//                int idxLeftTop= GetIdx(idxLeft, iPos.y +1);
//                UpdateNodeToNodeCost(idxLeftTop, idx, bAdd);

//                _dicDynamicNodeBlock.TryGetValue(idxLeft, out blockVal);
//                _dicDynamicNodeBlock[idxLeft] = bAdd ? (idxBlockVal + c_blockNum[5]) : (idxBlockVal - c_blockNum[5]);
//                if (leftX > 0)
//                {
//                    leftX = leftX - 1;
//                    idxLeft = GetIdx(iPos.x, idxLeft);
//                    if (_dicDynamicNodeBlock.TryGetValue(idxLeft, out blockVal))
//                    {
//                        if ((blockVal & c_blockNum[1]) > 0 || (blockVal & c_blockNum[2]) > 0)
//                        {
//                            if (iPos.y - 1 >= 0)
//                            {
//                                idxLeftBottom = GetIdx(leftX, iPos.y - 1);
//                                UpdateNodeToNodeCost(idxLeft, idxLeftBottom, bAdd);
//                            }
//                        }
//                        if ((blockVal & c_blockNum[5]) > 0 || (blockVal & c_blockNum[4]) > 0)
//                        {
//                            if (iPos.y < _size.y)
//                            {
//                                idxLeftTop = GetIdx(leftX, iPos.y + 1);
//                                UpdateNodeToNodeCost(idxLeft, idxLeftTop, bAdd);
//                            }
//                        }
//                    }
//                }
//                else
//                {
//                    if (iPos.x - 1 >= 0)
//                    {
//                        idxLeftBottom = GetIdx( leftX, iPos.y - 1);
//                        UpdateNodeToNodeCost(idxLeft, idxLeftBottom, bAdd);
//                    }
//                    else if (iPos.x < _size.x)
//                    {
//                        idxLeftTop = GetIdx(leftX, iPos.y + 1);
//                        UpdateNodeToNodeCost(idxLeft, idxLeftTop, bAdd);
//                    }
//                }
//                //right node check
//                int rightX = iPos.x;
//                int idxRight = idx;
//                int idxRightBottom = GetIdx(rightX, iPos.y - 1);
//                UpdateNodeToNodeCost(idxLeft, idxRightBottom, bAdd);
//                int idxRightop = GetIdx(rightX, iPos.y + 1);
//                UpdateNodeToNodeCost(idxLeft, idxRightBottom, bAdd);

//                if (rightX < _size.x - 1)
//                {
//                    rightX = rightX + 1;
//                    idxRight = GetIdx(iPos.x, rightX);
//                    if (_dicDynamicNodeBlock.TryGetValue(idxRight, out blockVal))
//                    {
//                        if ((blockVal & c_blockNum[0]) > 0 || (blockVal & c_blockNum[1]) > 0)
//                        {
//                            if (iPos.y - 1 >= 0)
//                            {
//                                int  idxBottom = GetIdx( rightX, iPos.y - 1);
//                                UpdateNodeToNodeCost(idxRight, idxBottom, bAdd);
//                            }
//                        }
//                        if ((blockVal & c_blockNum[6]) > 0 || (blockVal & c_blockNum[5]) > 0)
//                        {
//                            if (iPos.y < _size.y)
//                            {
//                                int idxTop = GetIdx(rightX, iPos.y + 1);
//                                UpdateNodeToNodeCost(idxRight, idxTop, bAdd);
//                            }
//                        }
//                    }
//                }
//                else
//                {
//                    if (iPos.y - 1 >= 0)
//                    {
//                        int idxBottom = GetIdx(rightX, iPos.y - 1);
//                        UpdateNodeToNodeCost(idxRight, idxBottom, bAdd);
//                    }
//                    else if (iPos.y < _size.y)
//                    {
//                        int idxTop = GetIdx(rightX, iPos.y + 1);
//                        UpdateNodeToNodeCost(idxRight, idxTop, bAdd);
//                    }
//                }

//            }
//            else
//            {

//            }
//#if UNITY_EDITOR
//            //if (PathFindingManager.DEBUG && bAdd)
//            //{
//            //    TSVector vec = GetWorldPosition(idx);
//            //    UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(vec);
//            //    UnityEngine.TextMesh tm = null;
//            //    if (!_dicExtraCost.TryGetValue(idx, out tm) && bAdd)
//            //    {
//            //        UnityEngine.GameObject go = UnityEngine.GameObject.Instantiate<UnityEngine.GameObject>(UnityEditor.AssetDatabase.LoadAssetAtPath<UnityEngine.GameObject>("Assets/pathval.prefab"));
//            //        if (go != null)
//            //        {
//            //            pos.y = 0.01f;
//            //            //go.transform.rotation = UnityEngine.Quaternion.Euler(90, -90, 0);
//            //            go.transform.position = pos;
//            //            go.transform.SetParent(UnityEngine.GameObject.Find("path_values").transform);
//            //            tm = go.GetComponent<UnityEngine.TextMesh>();
//            //            _dicExtraCost[idx] = tm;
//            //        }
//            //    }
//            //    if (tm != null)
//            //    {
//            //        tm.text = blockVal.ToString();
//            //        UnityEngine.Debug.DrawLine(pos - UnityEngine.Vector3.forward * 0.25f, pos + UnityEngine.Vector3.forward * 0.25f, UnityEngine.Color.black, 60);

//            //    }
//            //}
//#endif

//        }
        public bool IsWalkable(int x, int z,bool skipDynamic=false)
        {
            if (x < 0 || z < 0 || x >= _size.x || z >= _size.y)
            {
                return false;
            }
            int nodeIdx = z * _size.x + x;
            return IsWalkable(nodeIdx,skipDynamic);
        }
        public bool IsWalkable(int nodeIdx, bool skipDynamic = false)
        {
            if (!skipDynamic && IsDynamicUnwalkableNode(nodeIdx))
            {
                return false;
            }
#if USING_ASTAR_PROJECT && UNITY_EDITOR
            Pathfinding.GridGraph gg = Pathfinding.GridNode.GetGridGraph(0);           
            Pathfinding.GridNode node = gg.nodes[nodeIdx];
            return node.Walkable && (enabledTags >> (int)node.Tag & 0x1) != 0;
#else
            if (_isUnWalkable != null)
            {
#if !BattleInterface
                IInt2 vec = IInt2.zero;
#else
            BattleInterface.Vector2 vec = BattleInterface.Vector2.ZERO;   
#endif
                vec= GetGridPos(nodeIdx);
                //vec.x = x;// nsize
                //vec.y = z;// nsize;
                return !_isUnWalkable(vec);
            }
#endif
            return true;
        }
        public int GetGridNodeId(TSVector position)
        {
            IInt2 pos = IInt2.zero;
            GetGridCoord(position, ref pos);
            return pos.y * _size.x + pos.x;
        }
        public IInt2 GetNearestGridCoordWithoutClamp(TSVector position)
        {
            TSVector vec = GetGridFloatCoord(position);
            int ix = TSMath.Floor(vec.x).AsInt();
            int iz = TSMath.Floor(vec.z).AsInt();
            IInt2 coord;
            coord.x = ix;
            coord.y = iz;
            return coord;
        }
        public int ClampX(int x)
        {
            return CustomMath.Clamp(x, 0, _size.x - 1);
        }
        public int ClampZ(int z)
        {
            return CustomMath.Clamp(z, 0, _size.y - 1);
        }
        public TSVector GetGridFloatCoord(TSVector position)
        {
            TSVector vec = TSVector.zero;
            vec.x =( position.x) / nodeSize;// -_startPos.x;
            vec.z = (position.z) / nodeSize;// -_startPos.z;
            return vec;
        }
        public bool GetGridCoord(TSVector position, ref IInt2 coord)
        {
            TSVector vec = GetGridFloatCoord(position);
            int ix = TSMath.Floor(vec.x).AsInt();
            int iz = TSMath.Floor(vec.z).AsInt();
            int x = CustomMath.Clamp(ix, 0, _size.x - 1);
            int z =CustomMath.Clamp(iz, 0, _size.y - 1);
            //TSVector2 coord;
            coord.x = x;
            coord.y = z;
            return x == ix && z == iz;
        }
        TSVector _tempGridPosition = TSVector.zero;

        public IInt2 Size
        {
            get
            {
                return _size;
            }

            set
            {
                _size = value;
            }
        }
        public FP GetGridPositionDstSqr(TSVector pos)
        {
            TSVector vec = GetGridFloatCoord(pos);
            int ix = TSMath.Floor(vec.x).AsInt();
            int iz = TSMath.Floor(vec.z).AsInt();
            int x = CustomMath.Clamp(ix, 0, _size.x - 1);
            int z = CustomMath.Clamp(iz, 0, _size.y - 1);
            _tempGridPosition.Set((x + CustomMath.FPHalf) * nodeSize, 0, ((z + CustomMath.FPHalf) * nodeSize));
            return (_tempGridPosition-pos).sqrMagnitude;//+_startPos
        }
        public bool TryGetSpeicalPos(out TSVector specPos, TSVector pos, TSVector targetPos)// FP limitDstSqr,
        {
            int idx = GetGridNodeId(pos);
            specPos = GetWorldPosition(idx);
            return true;
           // return (specPos - targetPos).sqrMagnitude <= limitDstSqr;
        }
    
        public bool GetSpeicalPos(List<TSVector> _listPos,out TSVector specPos,FP limitDstSqr,TSVector pos, TSVector targetPos)
        {
            specPos = TSVector.zero;
           
            _listPos.Clear();
            TSVector gFpos= GetGridFloatCoord(pos);
            FP floorX = TSMath.Floor(gFpos.x);
            FP floorZ = TSMath.Floor(gFpos.z);
            FP diffX= gFpos.x- floorX;
            FP diffZ = gFpos.z - floorZ;
            if((diffX==FP.Zero||diffX==CustomMath.FPHalf) &&
                (diffZ == FP.Zero || diffZ== CustomMath.FPHalf))
            {
                specPos = pos;
                return true;
            }
            else if(diffX == CustomMath.FPHalf || diffZ == CustomMath.FPHalf)
            {
                if ( diffX == CustomMath.FPHalf)
                {
                    if(diffZ> CustomMath.FPHalf)
                    {
                        _listPos.Add(new TSVector(gFpos.x,0, floorZ+1));
                        _listPos.Add(new TSVector(gFpos.x, 0, floorZ+ CustomMath.FPHalf));
                        if(targetPos.x>pos.x)
                        {
                            _listPos.Add(new TSVector(floorX+1, 0, floorZ + 1));
                            _listPos.Add(new TSVector(floorX + 1, 0, floorZ + CustomMath.FPHalf));
                        }
                        else
                        {
                            _listPos.Add(new TSVector(floorX , 0, floorZ + 1));
                            _listPos.Add(new TSVector(floorX , 0, floorZ + CustomMath.FPHalf));
                        }
                    }
                    else
                    {
                        _listPos.Add(new TSVector(gFpos.x, 0, floorZ));
                        _listPos.Add(new TSVector(gFpos.x, 0, floorZ + CustomMath.FPHalf));
                        if (targetPos.x > pos.x)
                        {
                            _listPos.Add(new TSVector(floorX + 1, 0, floorZ));
                            _listPos.Add(new TSVector(floorX + 1, 0, floorZ + CustomMath.FPHalf));
                        }
                        else
                        {
                            _listPos.Add(new TSVector(floorX, 0, floorZ ));
                            _listPos.Add(new TSVector(floorX, 0, floorZ + CustomMath.FPHalf));
                        }
                    }
                }
                else
                {
                    if (diffX > CustomMath.FPHalf)
                    {
                        _listPos.Add(new TSVector(floorX+1, 0, gFpos.z));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, gFpos.z));
                        if (targetPos.z > pos.z)
                        {
                            _listPos.Add(new TSVector(floorX + 1, 0, floorZ + 1));
                            _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ + 1));
                        }
                        else
                        {
                            _listPos.Add(new TSVector(floorX + 1, 0, floorZ));
                            _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ));
                        }

                    }
                    else
                    {
                        _listPos.Add(new TSVector(floorX, 0, gFpos.z));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, gFpos.z));
                        if (targetPos.z > pos.z)
                        {
                            _listPos.Add(new TSVector(floorX, 0, floorZ+1));
                            _listPos.Add(new TSVector(floorX + CustomMath.FPHalf , 0, floorZ + 1));
                        }
                        else
                        {
                            _listPos.Add(new TSVector(floorX, 0, floorZ));
                            _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ));
                        }
                    }
                }             
            }
            else
            {
                if (diffX> CustomMath.FPHalf)
                {
                    if(diffZ> CustomMath.FPHalf)
                    {
                       _listPos.Add(new TSVector(floorX + CustomMath.FPHalf,0, floorZ + CustomMath.FPHalf));
                       _listPos.Add(new TSVector(floorX + 1, 0, floorZ + CustomMath.FPHalf));
                       _listPos.Add(new TSVector(floorX + 1, 0, floorZ + 1));
                       _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ + 1));
                    }
                    else
                    {
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ ));
                        _listPos.Add(new TSVector(floorX + 1, 0, floorZ));
                        _listPos.Add(new TSVector(floorX + 1, 0, floorZ + CustomMath.FPHalf));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ + CustomMath.FPHalf));
                    }
                }
                else
                {
                    if (diffZ > CustomMath.FPHalf)
                    {
                        _listPos.Add(new TSVector(floorX , 0, floorZ + CustomMath.FPHalf));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ + CustomMath.FPHalf));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ + 1));
                        _listPos.Add(new TSVector(floorX , 0, floorZ + 1));
                    }
                    else
                    {
                        _listPos.Add(new TSVector(floorX , 0, floorZ));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ));
                        _listPos.Add(new TSVector(floorX + CustomMath.FPHalf, 0, floorZ + CustomMath.FPHalf));
                        _listPos.Add(new TSVector(floorX , 0, floorZ + CustomMath.FPHalf));
                    }
                }                //
              
            }
            //
            FP minDstSqr = FP.MaxValue;
            TSVector toTarget = targetPos - pos;
            int count = _listPos.Count;
            for (int i = 0; i < count; i++)
            {
                TSVector spos = _listPos[i] * nodeSize;// + _startPos

                TSVector dir = targetPos - spos;
                FP dot = TSVector.Dot(dir, toTarget);
                FP dstSqr = dir.magnitude;
                if (dstSqr < minDstSqr && dot >=0)
                {
                    specPos = spos;
                    minDstSqr = dstSqr;
                }
            }
            return minDstSqr<=limitDstSqr;
        }
        public TSVector GetWorldPositionWithOutClamp(int x,int z)
        {
            _tempGridPosition.Set((x + CustomMath.FPHalf) * nodeSize, 0, ((z + CustomMath.FPHalf) * nodeSize));
            return _tempGridPosition;//+ _startPos
        }
        public TSVector GetWorldPosition(int gridIndex)
        {
            int x = CustomMath.Clamp(gridIndex % _size.x, 0, _size.x - 1);
            int z =CustomMath.Clamp((int)(gridIndex / _size.x), 0, _size.y - 1);
            _tempGridPosition.Set((x+CustomMath.FPHalf) * nodeSize, 0, ((z + CustomMath.FPHalf) * nodeSize));
            return _tempGridPosition;//+ _startPos
        }
        public TSVector GetWorldPosition(TSVector vec)
        {
            return GetWorldPosition(GetGridNodeId(vec));
        }
        public TSVector GetWorldRealPosition(TSVector vec)
        {
            return vec + _startPos;
        }
        //
#endregion
        public int GetIdx(int x, int y)
        {
            return y * _size.x + x;
        }
        public IInt2 GetGridPos(int id)
        {
            int idx = id;
            IInt2 pos = IInt2.zero;
            pos.x = idx % _size.x;
            pos.y = idx / _size.x;
            return pos;
        }
        public bool IsSameOrNeighbourNode(TSVector node1, TSVector node2)
        {
            IInt2 pos1=IInt2.zero;
            if(!GetGridCoord(node1, ref pos1))
            {
                return false;
            }
            IInt2 pos2 = IInt2.zero;
            if(!GetGridCoord(node2, ref pos2))
            {
                return false;
            }
            return IsSameOrNeighbourNode(pos1,pos2);
        }
        public bool IsSameOrNeighbourNode(IInt2 node1,IInt2 node2)
        {
            return node1 == node2;//|| (Math.Abs(node1.x-node2.x)<=1 && Math.Abs(node1.y - node2.y) <= 1);
        }
        public EDynamicBlockStatus DynamicBlockedStatus(int posIdx,int startPosIdx,int targetId, int curPosIdx)
        {
            if(posIdx != startPosIdx && posIdx != targetId && posIdx != curPosIdx)
            {
                DynamicBlock db;
                if (_hsDynamicUnwalkableNodes.TryGetValue(posIdx,out db))
                {
                    return db.isStationary? EDynamicBlockStatus.stationaryBlocked: EDynamicBlockStatus.blocked;
                }
            }
            else
            {
                return EDynamicBlockStatus.ignore;
            }
            return EDynamicBlockStatus.none;
        }
      /// <summary>
      /// From id2 to id1
      /// </summary>
      /// <param name="id1"></param>
      /// <param name="id2"></param>
      /// <returns></returns>
        public FP GetAStarNeighbourCost(int id1, int id2,IAStarGoal goal,int curPosIdx)
        {
            int startPosId=goal.StartNode.NodeID;
            int targetId = goal.TargetNode.NodeID;
            bool ignoredDynamicCost = targetId == id1 || targetId==id2;
            FP cost1=FP.Zero;
            // FP cost2 = FP.Zero;
            if(!PathFindingManager.c_useAvoidUnit)
            {
                EDynamicBlockStatus status1= DynamicBlockedStatus(id1, startPosId, targetId, curPosIdx);
                EDynamicBlockStatus status2 = DynamicBlockedStatus(id2, startPosId, targetId, curPosIdx);
                if(status1== EDynamicBlockStatus.stationaryBlocked
                    ||status2== EDynamicBlockStatus.stationaryBlocked)
                {
                    cost1 = C_MaxCost;
                    return cost1;
                }
                else if(status1 == EDynamicBlockStatus.blocked
                    || status2 == EDynamicBlockStatus.blocked)
                {
                    IInt2 pos1 = GetGridPos(id1);
                    IInt2 pos2 = GetGridPos(curPosIdx);
                    IInt2 diff = pos1 - pos2;

                    if (diff.sqrMagnitudeInt < c_dynamicRangeSqr)
                    {
                        cost1 = C_MaxCost;
                        return cost1;
                    }
                }               
            }
            
            if(IsDiagonalBlocked(id1, id2,startPosId,targetId, curPosIdx))
            {
                cost1 = C_MaxCost;
                return cost1;//neighbourCost[0]+
            }
            else
            {
                IInt2 pos1 = GetGridPos(id1);
                IInt2 pos2 = GetGridPos(id2);
                if (Math.Abs(pos1.x-pos2.x)+ Math.Abs(pos1.y - pos2.y)>1)
                {
                    return neighbourCost[0] + cost1;
                }
                else
                {
                    return neighbourCost[1] + cost1;
                }
             
            }
        }
        public FP GetAStarNeighbourCost(int index)
        {
            if(index >=0 && index< neighbourOffset.Length)
            {
                return neighbourCost[index];
            }
#if UNITY_EDITOR && PATHMANAGER_DEBUG
            UnityEngine.Debug.LogError("GetAStarNeighbourCost ,invalid index:"+ index);
#endif
            FP f = FP.One;
            return f;
        }
        
        public void Reset()
        {
#if USING_DYNAMIC_TREE
            if (_dynamicObstaclesTree!=null)
            {
                _dynamicObstaclesTree.Destroy();
            }
#endif
        }
        public bool CompareNodes(AStarNode node1, AStarNode node2)
        {
            return true;
        }
    }
}
