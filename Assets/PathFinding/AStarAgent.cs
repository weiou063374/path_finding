///author:huwei
///date:2018.4.18
//#define USING_ORCA
using System;
using System.Collections.Generic;
using TrueSync;
using AStarMachine;

namespace PathFinding
{
    public enum ERepathType
    {
        none=0,
        hasClosed=1,
        hasOpen=2
    }
    public enum EBlockType
    {
        none,
        agent,
        obstacle,
        staticAgent
    }

    public class AStarAgent : AgentBase
    {
        internal GridMap _map = null;
        public static readonly int S_QuickPathMaxDstOffset = 1;//direct ratio to the geometry size of the map obstacle 
        internal AStarMachine.AStarMachine _asm = null;
		public EPathType _pathType = EPathType.quickPath;
        public List<TSVector> _vecRunningPath = new List<TSVector>();
        public List<TSVector> _vecFullPath = new List<TSVector>();
        int _towardPathIdx = -1;
        public static readonly FP nodeCenterDstSqr =PathFindingManager.StopToNodeCenter?  GridMap.SCALE * GridMap.SCALE / 36:FP.MaxValue;

        FP _runningPathLeftDst = FP.Zero;
       // FP _checkDeltaDst = FP.One;
        bool _velocityChangedByORCA = false;
        public bool _bReachedTarget = false;
        IsWalkable _isValid = null;
        IsPosWalkable _isPosWalkable = null;

      //  Dictionary<int, bool> _preNodeBlocked = new Dictionary<int, bool>();
        TSVector _nextTarget;

        TSVector _preForceAcc = TSVector.zero;
        protected  FP _goalRadiusSq = FP.One * 1 / 100;
        //
        List<CircleObstacleAngleData> _circleObstacles = new List<CircleObstacleAngleData>();
        DynamicTreeQueryCallback _obstacleCallBack = null;

        //int _towardIdxForIgnoreCollision = -1;
        int _nodeIdxForIgnoreCollision = -1;
        List<int> _listNodeIdx = new List<int>();
        int _nodeIdxIgnoreCollisionCount = 0;
      
        int _checkRunningIdxEndTime = 0;
        const int c_max_nodeIdxIgnoreCollisionCount = 3;

        int _calWayPointIdxTime = 0;
#if UNITY_EDITOR
        public bool _showDebug = false;
#endif
        public enum ECheckDirStatus
        {
            ok,
            delay,
            fail
        }
        public void Init(IAgentBehaviour behaviour, FP radius, GridMap map, byte eType)
        {
            _behaviour = behaviour;
            _map = map;
            _isValid = this.IsWalkable;
            _isPosWalkable = this.IsPosWalkable;
            _obstacleCallBack = this.ObstacleRayCastCallback;
        
            _activeBoids = (byte)eType;
            _vecFullPath.Capacity = Math.Max(_vecFullPath.Capacity,100);
            _vecRunningPath.Capacity = Math.Max(_vecRunningPath.Capacity, 100);

            this.Init(behaviour);
        }
        internal AStarAgent(IAgentBehaviour go) : base(go)
        {
            searchFinished = this.PathFindingFinished;
        }
        internal bool NeedDoPriorityPath()
        {
            return _towardPathIdx == 0 && _asm != null && _asm.IsWorking;
        }
        internal void MergePath(EPathType type)
        {
            int iCount = _vecFullPath.Count;
            int idx = 0;
            int rCount = _vecRunningPath.Count;
            TSVector lastVec = TSVector.zero;
            while (true)
            {
                if (rCount>idx  && iCount>idx &&_vecRunningPath[0]== _vecFullPath[iCount-1-idx])
                {
                    _vecFullPath.RemoveAt(iCount - 1 - idx);
                    lastVec = _vecRunningPath[0];
                    _vecRunningPath.RemoveAt(0);
                    idx++;
                }
                else
                {
                    if(idx>0)
                    {
                        _vecFullPath.Add(lastVec);
                    }
                    break;
                }               
            }
            if (type == EPathType.fullPath)
            {
                _towardPathIdx = (_vecFullPath.Count + _towardPathIdx);
            }
            else
            {
                _towardPathIdx = System.Math.Max(_vecFullPath.Count + _vecRunningPath.Count - 1,0);
            }

            if (iCount > 0)
            {
                _vecFullPath.AddRange(_vecRunningPath);
                List<TSVector> tmp = _vecRunningPath;
                _vecRunningPath = _vecFullPath;
#if UNITY_EDITOR && PATHMANAGER_DEBUG && !MULTI_THREAD
                if (PathFindingManager.DEBUG)
                {
                    CheckRuningpathRepeat();
                }
#endif
                tmp.Clear();
                //if(PathFindingManager.c_useSmoothPath)
                //{                  
                //    SmoothPath.SmoothSimple(_vecRunningPath, tmp);
                //    _vecFullPath = _vecRunningPath;
                //    _vecRunningPath = tmp;
                //}
                //else
                {
                    _vecFullPath = tmp;
                }

#if UNITY_EDITOR && !MULTI_THREAD
                if (PathFindingManager.DEBUG)
                {
                    if (_vecRunningPath.Count == 1)
                    {
                       UnityEngine.Debug.LogError("vecRunningPath.Count == 1");
                    }
                }
#endif
            }
            UpdateRunningPathLeftDst(_vecRunningPath);
        }
        void UpdateRunningPathLeftDst(List<TSVector> vecRunningPath)
        {
            if(!c_useProjDst)
            {
                return;
            }
       
            int count = vecRunningPath.Count;

            if (_towardPathIdx > 0 && _towardPathIdx< count)
            {
                _runningPathLeftDst = FP.Zero;
                if(!PathFindingManager.c_useSmoothPath)
                {
                    for (int i = _towardPathIdx; i > 0; i--)
                    {
                        TSVector diff = vecRunningPath[i] - vecRunningPath[i - 1];
                        _runningPathLeftDst += (diff.x != 0 && diff.z != 0) ? GridMap.neighbourCost[0] : GridMap.neighbourCost[1];
                    }
                }
                else
                {
                    IInt2 pos1 = IInt2.zero, pos2 = IInt2.zero;
                    _behaviour.map.GetGridCoord(vecRunningPath[_towardPathIdx], ref pos1);
                    _behaviour.map.GetGridCoord(vecRunningPath[0], ref pos2);
                    _runningPathLeftDst = GridAStarGoal.CalculateHValue(pos1, pos2, Heuristic.DiagonalManhattan);
                }              
            }
        }
        void HandleFullPathResult(AStarMachine.AStarMachine asm, GridMap map,bool canAddRevs)
        {
            //find splice path start position and target position
            //find start pos from unvisited quick path nodes ,use current position 
            TSVector spliceStart = _behaviour.position;

            int iCount = _vecFullPath.Count;
            int scanCount = AStarMachine.AStarMachine.C_MaxRevolutions;
            int eIdx = Math.Max(0, iCount - 1 - scanCount);
            FP quickPathMaxDst = GridMap.SCALE * (scanCount + S_QuickPathMaxDstOffset);
            IInt2 startPos = IInt2.zero;
            map.GetGridCoord(spliceStart, ref startPos);
            GridAStarGoal goal = (GridAStarGoal)(asm.Goal);
            int nearestIdx = iCount - 1;
            FP nerestDiff = int.MaxValue;
            IInt2 curPos = IInt2.zero;
            map.GetGridCoord(_behaviour.position, ref curPos);
            bool skipSplice = false;
            for (int i = iCount - 1; i >= eIdx; i--)
            {
                IInt2 pos = IInt2.zero;
                map.GetGridCoord(_vecFullPath[i], ref pos);
                if(_map.IsSameOrNeighbourNode(pos,curPos))
                {
                    nerestDiff = 0;
                    nearestIdx = i;
                    skipSplice = true;
                }
                else
                {
                    FP dst = GridAStarGoal.CalculateHValue(startPos, pos, goal.Heuristic);
                    FP diff = TSMath.Abs((dst - quickPathMaxDst));
                    if (diff < nerestDiff)
                    {
                        nerestDiff = diff;
                        nearestIdx = i;
                    }
                }                
            }
            if (nearestIdx == iCount - 1 || nearestIdx < 0||skipSplice)//don't need splice path
            {
                //
                if (skipSplice)
                {
                    if (nearestIdx < iCount - 1)
                    {
                        _vecFullPath.RemoveRange(nearestIdx + 1, iCount - nearestIdx - 1);
                    }
                    _vecRunningPath.Clear();
                    _vecRunningPath.AddRange(_vecFullPath);
                    _towardPathIdx = _vecRunningPath.Count - 1;

#if UNITY_EDITOR&& PATHMANAGER_DEBUG && !MULTI_THREAD
                    if (PathFindingManager.DEBUG)
                    {
                        CheckRuningpathRepeat();
                    }
#endif
                }
                else
                {
                    MergePath(EPathType.fullPath);
                }              
            }
            else
            {
                //  
                TSVector spliceTarget = _vecFullPath[nearestIdx];
                bool bFound = _behaviour.pathManager.FindSplicePath(asm, spliceStart, spliceTarget, canAddRevs,
                    _map.GetGridNodeId(_behaviour.position));
                if (bFound)
                {
                    if (nearestIdx < iCount - 1)
                    {
                        _vecFullPath.RemoveRange(nearestIdx + 1, iCount - nearestIdx - 1);
                    }
                    _behaviour.pathManager.ProcessFinishedPath(asm, EPathType.splicePath);
                }
                else
                {
                    MergePath(EPathType.fullPath);
                    //
                    asm.IsWorking = false;
#if UNITY_EDITOR&& PATHMANAGER_DEBUG && !MULTI_THREAD
                    if (PathFindingManager.DEBUG)
                    {
                        UnityEngine.Debug.LogWarning("FindSplicePath fail to target with revs:" + ((PathFindingAgentBehaviour)(_behaviour)).name);
                    }                    
#endif
                }
            }
            //
            if (_vecRunningPath.Count > 0)
            {
                _vecRunningPath[0] = _originTargetPos;
            }
            if (PathFindingManager.c_useSmoothPath)
            {
                _vecFullPath.Clear();
                List<TSVector> tmp = _vecFullPath;
                SmoothPath.SliceSimple(_vecRunningPath, tmp);         
                _vecFullPath = _vecRunningPath;
                _vecRunningPath = tmp;
            }
        }
#if UNITY_EDITOR&& PATHMANAGER_DEBUG
        HashSet<TSVector> hs = null;
        void CheckRuningpathRepeat()
        {
            if(hs==null)
            {
                hs =new HashSet<TSVector>();
            }
            hs.Clear();
            int count = _vecRunningPath.Count;

            for(int i=0;i<count;i++)
            {
                if(hs.Contains(_vecRunningPath[i]))
                {
                    UnityEngine.Debug.LogError("RuningpathRepeat");
                   // break;
                }
                //if (i>0 &&(TSMath.Abs(_vecRunningPath[i].x- _vecRunningPath[i-1].x)>1 || TSMath.Abs(_vecRunningPath[i].z - _vecRunningPath[i - 1].z) > 1))
                //{
                //    UnityEngine.Debug.LogError("Runingpath node missing "+(_behaviour as PathFindingAgentBehaviour).name);
                //    //break;
                //}
                hs.Add(_vecRunningPath[i]);
            }
        }
#endif
        internal void AddDirectNode(TSVector startPos, TSVector endPos)
        {
            if (_asm != null && _asm.IsWorking)
            {
                _asm.IsWorking = false;
                _asm._agent = null;
                _asm = null;
            }
            ClearPath();
            _startPos = startPos;
            _targetPos = endPos;
            _vecRunningPath.Add(endPos);
            _vecRunningPath.Add(startPos);        
        }
        internal void PathFindingFinished(AStarMachine.AStarMachine asm, EPathType pathType, bool canAddRevs)
        {
            if (_map == null)
            {
                return;
            }
			_pathType = pathType;
            AStarMachine.AStarNode node = pathType == EPathType.quickPath ? ((GridAStarGoal)asm.Goal).ClosestNodeToTarget : asm.Goal.CurrentNode;
            if (node == null || asm.Goal.CurrentNode==null||asm.Goal.CurrentNode.G>=GridMap.C_MaxCost)
            {
                if (EPathType.fullPath == pathType)
                {
                    _behaviour.AddUnReachablePos(_map, _map.GetGridNodeId(_targetPos));
                }
                ClearPath();
                _behaviour.pathBlocked = true;
                return;
            }
            _behaviour.pathBlocked = false;
            if (pathType == EPathType.quickPath)
            {
                ClearPath();
            }
            else if (pathType == EPathType.fullPath)
            {
                _vecFullPath.Clear();
            }
            else if (pathType == EPathType.splicePath)
            {
                _vecRunningPath.Clear();
            }
            GridMap map = _map;
            List<TSVector> listPath = pathType == EPathType.fullPath ? _vecFullPath : _vecRunningPath;
            int nodeCount = 0;
            while (node != null)
            {
                listPath.Add(map.GetWorldPosition(node.NodeID));
                node = node.Parent;
                if (nodeCount++ > 1000)
                {
                    listPath.Clear();
                    break;
                }
            }
#if UNITY_EDITOR && !MULTI_THREAD
            if (PathFindingManager.DEBUG)
            {
                if (_vecRunningPath.Count == 1)
                {
                    UnityEngine.Debug.LogError("vecRunningPath.Count == 1,--");
                }

                //Color color = new Color(UnityEngine.Random.Range(0, 1.0f), UnityEngine.Random.Range(0, 1.0f), UnityEngine.Random.Range(0, 1.0f));
                if (pathType != EPathType.fullPath)
                {
                    for (int i = 1; i < _vecRunningPath.Count; i++)
                    {
                        UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(_vecRunningPath[i - 1]+_behaviour.map._startPos)/GridMap.SCALE.AsFloat()
                            , CustomMath.TsVecToVector3(_vecRunningPath[i] + _behaviour.map._startPos) / GridMap.SCALE.AsFloat(), pathType == EPathType.quickPath ? UnityEngine.Color.white : UnityEngine.Color.red, 5);
                    }
                }

                for (int i = 1; i < _vecFullPath.Count; i++)
                {
                    UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(_vecFullPath[i - 1] + _behaviour.map._startPos) / GridMap.SCALE.AsFloat(), CustomMath.TsVecToVector3(_vecFullPath[i] + _behaviour.map._startPos) / GridMap.SCALE.AsFloat(), UnityEngine.Color.blue, 1);
                }
                if(EPathType.fullPath == pathType)
                {
                    if(_vecFullPath.Count>0&& _vecRunningPath.Count>0 && _vecFullPath[_vecFullPath.Count-1]!=_vecRunningPath[0])
                    {
                        UnityEngine.Debug.LogError("_vecFullPath!=_vecRunningPath");
                    }
                }
            }
#endif
            if (EPathType.fullPath == pathType)
            {
                HandleFullPathResult(asm, map,canAddRevs);
            }
            else if (EPathType.splicePath == pathType)
            {
                MergePath(pathType);
            }
            else if (EPathType.quickPath == pathType)
            {
                if (_towardPathIdx < 0)
                {
                    _towardPathIdx = _vecRunningPath.Count - 1;
                }
                UpdateRunningPathLeftDst(_vecRunningPath);
            }
  
        }
        public AStarAgent() : base()
        {
            searchFinished = this.PathFindingFinished;
        }
        public bool IsFindingPath()
        {
            if(PathFindingManager.c_useAvoidUnit)
            {
                if(_vecRunningPath.Count==0 && (_asm == null || !_asm.IsWorking))
                {
                    return false;
                }
                return true;
            }
            if(_asm!=null && _asm.IsWorking)
            {
                return true;
            }
            return _vecRunningPath.Count > 0;// && _towardPathIdx!=0;
        }
        public void ClearPath()
        {
            _vecRunningPath.Clear();
            _vecFullPath.Clear();
            _towardPathIdx = -1;

            _runningPathLeftDst = FP.Zero;

            _checkRunningIdxEndTime = 0;
            _nodeIdxForIgnoreCollision = -1;
            _nodeIdxIgnoreCollisionCount = 0;
            _listNodeIdx.Clear();
            _nodeIdxForIgnoreCollision = -1;
            _calWayPointIdxTime = 0;
           // _preForceAcc = TSVector.zero;
            // velocity = TSVector.zero;
        }
        public override void Reset()
        {
            ClearPath();
            if (_behaviour != null)
            {
               // _checkDeltaDst = TSMath.Min(_behaviour.colliderRadius, GridMap.GetNodeSize());
            }
            base.Reset();
            if (_asm != null && _asm.IsWorking)
            {
                _asm.IsWorking = false;
            }
            _bReachedTarget = false;
            _velocityChangedByORCA = false;
            _map = null;
            // _checkDeltaDst = _checkDeltaDst * _checkDeltaDst;
            _asm = null;
            _nextTarget = TSVector.zero;
           // _preNodeBlocked.Clear();

            _preForceAcc = TSVector.zero;
            _circleObstacles.Clear();
            _currentVelFactor = Max_Vel_Factor;
            _preTestOKVec = TSVector.zero;
            _nodeIdxForIgnoreCollision = -1;
            _checkRunningIdxEndTime = 0;
            _nodeIdxIgnoreCollisionCount = 0;
            _listNodeIdx.Clear();
            _nodeIdxForIgnoreCollision = -1;
        }

        public static FP ClosestPointOnLineFactor(TSVector lineStart, TSVector lineEnd, TSVector point)
        {
            TSVector dir = lineEnd - lineStart;
            FP len = dir.magnitude;

            if (len <= CustomMath.EPSILON) return FP.Zero;

            FP factor = TSMath.Abs((FP)(TSVector.Dot(point - lineEnd, dir) / len));//projection length ratio
            return factor;
        }
        /// <summary>
        /// deprecated
        /// </summary>
        /// <returns></returns>
        public TSVector CalculateWayPoint(List<TSVector> vecRunningPath)
        {
            var p1 = vecRunningPath[_towardPathIdx + 1];
            var p2 = vecRunningPath[_towardPathIdx];

            // Calculate the intersection with the circle
            var t = CustomMath.LineCircleIntersectionFactor(_behaviour.position, p1, p2,GridMap._checkDeltaDst);// _behaviour.colliderRadius
            // Clamp to a point on the segment
            t = TSMath.Clamp(t, FP.Zero, FP.One);
            TSVector waypoint = TSVector.Lerp(p1, p2, t);
            return waypoint;
        }
        public TSVector CalculateProjPos(List<TSVector> vecRunningPath)
        {
            var p1 = vecRunningPath[_towardPathIdx];
            var p2 = vecRunningPath[_towardPathIdx-1];

            TSVector dir = p2 - p1;
            FP len = dir.magnitude;
            TSVector point = _behaviour.position;

            if (len <= CustomMath.EPSILON) return point;

            TSVector dirNorm = dir.normalized;
            TSVector waypoint = TSVector.Dot((point - p1), dirNorm)*dirNorm  +p1;
            return waypoint;
        }
        public bool FindClosestWayPointAvoidObstacle(TSVector point, int startIdx, int endIdx, List<TSVector> vecRunningPath)
        {
            if(_calWayPointIdxTime>_behaviour.baseData.pfm.CurTime)
            {
                return false;
            }
            int count = vecRunningPath.Count;
            int towardIdx = -1;
            for (int i = endIdx; i <= startIdx; i++)
            {
                TSVector blockedPos = TSVector.zero;
                bool hasObstacle = _behaviour.map.IsBlockedByObstacleBetween2Point(_behaviour.position, vecRunningPath[i]
                      , _behaviour.pathManager._queryStack, ref blockedPos);
                if(!hasObstacle)
                {
                    towardIdx = i;
                    break;
                }
            }
            bool bNextBlocked = _towardPathIdx < towardIdx;
            if(towardIdx>=0)
            {
                _towardPathIdx = towardIdx;
            }
            _calWayPointIdxTime = _behaviour.baseData.pfm.CurTime + 200;
            return bNextBlocked;
        }
        public void FindClosestWayPoint(TSVector point, int startIdx, int endIdx, List<TSVector> vecRunningPath)
        {
            int count = vecRunningPath.Count;
            FP minDstSqr = FP.MaxValue;
            int minIdx =int.MaxValue ;//_towardPathIdx;

            for (int i = startIdx; i >= endIdx; i--)
            {
                FP dstSqr = (vecRunningPath[i] - point).sqrMagnitude;
                int diff = (i - minIdx);
                //?????
                if (minDstSqr > dstSqr)//&& (minIdx >= i || (count>2 &&FP.EN1*5 < -dstSqr + minDstSqr))
                {
                    minDstSqr = dstSqr;
                    minIdx = i;
                }
            }
           // if (minIdx != _towardPathIdx)
            {
                bool bSkipMin = false;
                if(_towardPathIdx<minIdx && _towardPathIdx >= 0)
                {
                    TSVector blockedPos = TSVector.zero;
                    int idx1 = _map.GetGridNodeId(_behaviour.position);
                    int idx2 = _map.GetGridNodeId(vecRunningPath[_towardPathIdx]);
                    IInt2 pos1 = _map.GetGridPos(idx1);
                    IInt2 pos2 = _map.GetGridPos(idx2);
                    bool hasObstacle = false;
                    if (Math.Abs(pos1.x - pos2.x) > 1 || Math.Abs(pos1.x - pos2.x) > 1)
                    {
                        hasObstacle = _behaviour.map.IsBlockedByObstacleBetween2Point(_behaviour.position, vecRunningPath[_towardPathIdx]
                     , _behaviour.pathManager._queryStack, ref blockedPos);
                    }
                    if (!hasObstacle)
                    {
                        bSkipMin=true;
                    }
                }
                if(!bSkipMin)
                {
                    _towardPathIdx = minIdx;
                }
               // FP dstSqr = PathFindingManager.StopToNodeCenter ? nodeCenterDstSqr : _checkDeltaDst * _checkDeltaDst;
                if (_towardPathIdx > 0)//&& minDstSqr< dstSqr
                {
                    TSVector pathDir = vecRunningPath[_towardPathIdx - 1] - vecRunningPath[_towardPathIdx];
                    TSVector dir1 = vecRunningPath[_towardPathIdx - 1] - point;
                    TSVector dir2 = vecRunningPath[_towardPathIdx] - point;
                    int sign = TSMath.Sign(TSVector.Dot(pathDir, dir1)) * TSMath.Sign(TSVector.Dot(pathDir, dir2));
                    if (sign <= 0)
                    {
                        _towardPathIdx--;
                    }
                }
                UpdateRunningPathLeftDst(vecRunningPath);
            }
        }
        const int c_searchCount = 3;
#if DYNAMIC_FORCE
        const bool c_useProjDst = false;
#else
        const bool c_useProjDst = true;
#endif
        public TSVector MoveToClosestWayPointByDst(TSVector point, FP fDtSecond, List<TSVector> vecRunningPath,ref FP desiredSpeed)
        {
            int count = vecRunningPath.Count;
            if (PathFindingManager.c_useAvoidUnit && count==2)
            {
                _towardPathIdx = 0;
                return vecRunningPath[0];
            }
            desiredSpeed = _behaviour.maxSpeed;

            if (count == 0)
            {
                return _behaviour.position;
            }
            if (_towardPathIdx < 0 || _towardPathIdx > count - 1)
            {
                _towardPathIdx = count - 1;//,at least has two count:start,target pos  

            }
            int startIdx = Math.Min(_towardPathIdx + c_searchCount, count - 1);
            int endIdx = Math.Max(_towardPathIdx - c_searchCount, 0);
            if (_moveFailedCount >= C_maxMoveFailedCount
              //  || (!_map.IsWalkable(_behaviour.position) && (_towardPathIdx < count - 1 || _towardPathIdx == 0))
                )//blocked, search all path nodes
            {
                //_moveFailedCount = 0;
                ////blocked by obstacles because of orca ,need repath
                //if (_behaviour.group != null)
                //{
                //    AgentGroupManager.instance.RemoveAgent(_behaviour);
                //}
                //ClearPath();
                //PathManager.instance.FindQuickPath(this, _behaviour.position, _targetPos, _map, false);
               // return _behaviour.position;//test
            }
            bool bNextBlocked = false;
            if(PathFindingManager.c_useAvoidUnit)
            {
                bNextBlocked=FindClosestWayPointAvoidObstacle(point, startIdx, endIdx, vecRunningPath);
            }
             
            FP dstSqr = PathFindingManager.StopToNodeCenter ? nodeCenterDstSqr : GridMap._checkDeltaDstSqr;            //
           
            //
            while (count > 0&& _towardPathIdx != 0)//_checkDeltaDst * _checkDeltaDst
            {
                bool bNext = false;
                if (_towardPathIdx+1 <count && c_useProjDst)
                {
                   FP dst = ClosestPointOnLineFactor(vecRunningPath[_towardPathIdx +1], vecRunningPath[_towardPathIdx], _behaviour.position);
                   bNext = dst < GridMap.SCALE * 4/10;
                 
                }
                else
                {
                    if(_towardPathIdx==count-1 && _towardPathIdx>=1)
                    {
                        TSVector pathDir = vecRunningPath[_towardPathIdx - 1] - vecRunningPath[_towardPathIdx];
                        TSVector dir1 = vecRunningPath[_towardPathIdx - 1] - point;
                        TSVector dir2 = vecRunningPath[_towardPathIdx] - point;
                        int sign = TSMath.Sign(TSVector.Dot(pathDir, dir1)) * TSMath.Sign(TSVector.Dot(pathDir, dir2));
                        if (sign <= 0 ||dir2.sqrMagnitude<dstSqr)
                            bNext = true;// (_behaviour.position - vecRunningPath[_towardPathIdx]).sqrMagnitude < dstSqr;
                    }
                    else
                    {
                        bNext = (_behaviour.position - vecRunningPath[_towardPathIdx]).sqrMagnitude < dstSqr;
                    }
                 
                    
                }
                if(bNext)
                {
                    _towardPathIdx--;
                    TSVector diff = vecRunningPath[_towardPathIdx] - vecRunningPath[_towardPathIdx + 1];
                    _runningPathLeftDst -= (diff.x != 0 && diff.z != 0) ? GridMap.neighbourCost[0] : GridMap.neighbourCost[1];
                    break;
                }
                else
                {
                    break;
                }
            }
            if (_towardPathIdx == 0 && _asm != null && _asm.IsWorking)
            {
               // PathManager.instance.DoPriorityPath(this);
                return _behaviour.position;
            }

            if (_towardPathIdx > vecRunningPath.Count -1)
            {
                _towardPathIdx = vecRunningPath.Count -1;//_vecRunningPath ,at least has two count:start,target pos
            }
            //if (_towardPathIdx<=0 || count==0)//|| _vecRunningPath.Count-1< _towardPathIdx + 1
            //{
            //    return _behaviour.position;
            //}
            bool alongPath = false;
            if (PathFindingManager.c_useAvoidUnit)
            {
                if(bNextBlocked && _towardPathIdx>0)
                {
                    alongPath = true;
                }
                else
                {
                    return vecRunningPath[_towardPathIdx];
                }
            }
        
            TSVector rvoTarget =TSVector.zero;
       
            if(c_useProjDst && _towardPathIdx>0 )
            {               
                FP dstLeft =TSMath.Max(_runningPathLeftDst,FP.One);//_runningPathLeftDst + 
               // desiredSpeed = TSMath.Clamp(dstLeft / PathFindingAgentBehaviour.slowdownDistance, 0, FP.One) * _behaviour.maxSpeed;
                if( _towardPathIdx + 1 < count)
                {
                    TSVector pathDir = (vecRunningPath[_towardPathIdx] - vecRunningPath[_towardPathIdx + 1]);
                    if(TSVector.Dot(pathDir, vecRunningPath[_towardPathIdx] - _behaviour.position)<=0)
                    {
                        alongPath = true;
                    }
                    else
                    {
                        rvoTarget = CustomMath.Normalize(pathDir) * dstLeft + _behaviour.position;
                    }                                     
                }
                else
                {
                    rvoTarget = CustomMath.Normalize((vecRunningPath[_towardPathIdx] - _behaviour.position)) * dstLeft + _behaviour.position;
                }             
            }
            else
            {
                alongPath = true;
            }
            if(alongPath )
            {
                TSVector waypoint = TSVector.zero;
               
                if(PathFindingManager.c_useAvoidUnit)
                {
                    if (_towardPathIdx - 1 >= 0 && _towardPathIdx - 1 < vecRunningPath.Count)
                    {
                        waypoint = (_map.IsDynamicUnwalkableNode(_startPosId) || _towardPathIdx + 1 >= count) ? vecRunningPath[_towardPathIdx] : CalculateProjPos(vecRunningPath);//vecRunningPath[_towardPathIdx];//
                        TSVector blockedPos = TSVector.zero;
                        bool hasObstacle = _behaviour.map.IsBlockedByObstacleBetween2Point(waypoint, _vecRunningPath[_towardPathIdx - 1]
                   , _behaviour.pathManager._queryStack, ref blockedPos);
                        if (hasObstacle)
                        {
                            _moveFailedCount = C_maxMoveFailedCount + 1;
                        }
                    }
                  
                }
                else
                {
                    waypoint = vecRunningPath[_towardPathIdx];// _towardPathIdx + 1 >= count ? vecRunningPath[_towardPathIdx] : CalculateProjPos(vecRunningPath);
                }              
                //FP dstLeft = _runningPathLeftDst + (waypoint - _behaviour.position).magnitude;//_runningPathLeftDst + 

                //desiredSpeed = TSMath.Clamp(dstLeft / PathFindingAgentBehaviour.slowdownDistance, 0, FP.One) * _behaviour.maxSpeed;
                rvoTarget = waypoint;// CustomMath.Normalize((waypoint - _behaviour.position)) * dstLeft + _behaviour.position;
            }              
            return rvoTarget;
        }       
        //Dictionary<int, bool> _preDynamicNode = new Dictionary<int, bool>();
        public ERepathType NeedRePath()
        {
            if (PathFindingManager.c_useAvoidUnit)
            {
                //if(_vecRunningPath.Count==2)//direct line path
                //{
                //    if(_map.GetGridNodeId(_vecRunningPath[1])!= _map.GetGridNodeId(_behaviour.position))
                //    {
                //        return ERepathType.hasClosed;
                //    }
                //}
                if (_moveFailedCount >= C_maxMoveFailedCount)
                {
                    return ERepathType.hasClosed;
                }
                return ERepathType.none;//test
            }           
            ERepathType bNeedRepath = 0;
            int nodeID = _map.GetGridNodeId(_behaviour.position);
            int curIdx = nodeID;
            //bool preBlocked = false;
            //if(_preNodeBlocked.TryGetValue(nodeID, out preBlocked))
            //{

            //}
            bool bBlocked = false;
           
           
            int idx = System.Math.Max(_towardPathIdx - 1, 0);
            int count = _vecRunningPath.Count;
            if(count==1)
            {
                bNeedRepath = bNeedRepath | ERepathType.hasClosed;
                return bNeedRepath;
            }
            int sIdx = curIdx;
            int eIdx = -1;
            if (_targetPos!=TSVector.MinValue)
            {
                sIdx = curIdx;// _map.GetGridNodeId(_startPos);
                eIdx = _map.GetGridNodeId(_targetPos) ;
            }
            //check path node,deprecated
            if (idx <= count - 1 && _towardPathIdx >= 0 && count > _towardPathIdx)
            {
                //
                int preNodeId = count - 1 <= _towardPathIdx ? -1 : _map.GetGridNodeId(_vecRunningPath[_towardPathIdx + 1]);
                int tIdx = _map.GetGridNodeId(_targetPos);
                IInt2 curNode = _map.GetGridPos(curIdx);
                for (int i = _towardPathIdx; i >= idx; i--)
                {
                    nodeID = _map.GetGridNodeId(_vecRunningPath[i]);
                    IInt2 nNode= _map.GetGridPos(nodeID);
                    IInt2 diff = curNode - nNode;
                    if (nodeID==curIdx
                       || diff.sqrMagnitudeInt>4
                        )
                    {
                        continue;
                    }
                    //preBlocked = false;
                    //_preNodeBlocked.TryGetValue(nodeID, out preBlocked);
                    if (!PathFindingManager.StopToNodeCenter && nodeID != tIdx)
                    {
                        bBlocked = !_map.IsWalkable(_vecRunningPath[i], nodeID==curIdx ||nodeID==tIdx);
                        if (preNodeId >= 0)//
                        {
                            bool blocked = !_map.IsWalkable(nodeID,nodeID==sIdx||nodeID==eIdx ||nodeID==curIdx)
                                || !_map.IsWalkable(preNodeId, preNodeId == sIdx || preNodeId == eIdx || preNodeId == curIdx)
                                || _map.IsDiagonalBlocked(nodeID, preNodeId,sIdx,eIdx, curIdx);
                            if (blocked)
                            {
                                bNeedRepath = bNeedRepath | ERepathType.hasClosed;
                            }
                        }
                        if (bBlocked)
                        {
                            bNeedRepath = bNeedRepath | ERepathType.hasClosed;
                        }
                    }
                    else
                    {
                        bBlocked = _map.IsDynamicUnwalkableNode(nodeID);
                    }
                    //if (bBlocked != preBlocked)
                    //{
                    //    _preNodeBlocked[nodeID] = bBlocked;
                    //    if (bBlocked)
                    //    {
                    //        bNeedRepath = bNeedRepath | ERepathType.hasClosed;
                    //    }
                    //    else
                    //    {
                    //       // bNeedRepath = bNeedRepath | ERepathType.hasOpen;
                    //    }
                    //}
                    preNodeId = nodeID;
                }
            }

            if (bNeedRepath!=ERepathType.none)
            {
                return bNeedRepath;
            }
            else
            {
                if (_moveFailedCount >= C_maxMoveFailedCount
                  //  || (!_map.IsWalkable(_behaviour.position) && (_towardPathIdx < count - 2 || _towardPathIdx == 0))
                    )//blocked, search all path nodes
                {                   
                    //blocked by obstacles because of orca ,need repath
                    if (_behaviour.group != null)
                    {
                        AgentGroupManager.instance.RemoveAgent(_behaviour);
                    }
                    ClearPath();
                    bNeedRepath = bNeedRepath | ERepathType.hasClosed;
                    return bNeedRepath;
                }
            }
            return ERepathType.none;
        }
        public bool IsNearTargetPos()
        {
            if(_vecRunningPath.Count>0)
            {
                IInt2 node1=IInt2.zero;
                IInt2 node2 = IInt2.zero;
                if(!_map.GetGridCoord(_vecRunningPath[0], ref node1))
                {
                    return false;
                }
                if(!_map.GetGridCoord(_behaviour.position, ref node2))
                {
                    return false;
                }
                return _map.IsSameOrNeighbourNode(node1, node2);
            }
            return true;
        }
        public override void SetDesiredPosOrVelocity(FP deltaTime, IsPosWalkable w = null, bool bSkipBoids=false)
        {           
            List<TSVector> vecRunningPath = _vecRunningPath;
            _velocityChangedByORCA = false;
            if (_behaviour.group != null && _behaviour.group.leader != _behaviour)
            {
                vecRunningPath = (_behaviour.group.leader.agent as AStarAgent)._vecRunningPath;
            }
            
            FP fDtSecond = deltaTime;
            int count = vecRunningPath.Count;
            _nextTarget= _behaviour.position;
            FP desiredSpeed = _behaviour.maxSpeed;
            if (count > 1)
            {
                if(count==2)
                {
                    vecRunningPath[1] = _behaviour.position;
                }

               _bReachedTarget = (vecRunningPath[0] - _behaviour.position).sqrMagnitude<GridMap._checkDeltaDstSqr;//!PathFindingManager.StopToNodeCenter ? (_map.GetGridNodeId(_targetPos) == _map.GetGridNodeId(_behaviour.position)):
                    //(_targetPos - _behaviour.position).sqrMagnitude < nodeCenterDstSqr;// && (vecRunningPath[0] - _behaviour.position).sqrMagnitude < targetCenterDstSqr;// _map.GetGridNodeId(vecRunningPath[0]) == _map.GetGridNodeId(_behaviour.position);
                if (!_bReachedTarget)
                {
                    _nextTarget = MoveToClosestWayPointByDst(_behaviour.position, fDtSecond, vecRunningPath,ref desiredSpeed);
                   // if (target == _behaviour.position)
                    {
                     //   _behaviour.preVelocity = CustomMath.Normalize(_nextTarget - _behaviour.position) * desiredSpeed;//vecRunningPath[_towardPathIdx]
                    }
                }
                else// reach target
                {
                    ClearPath();
                }
            }
            else
            {
                _bReachedTarget = true;
                _behaviour.preVelocity = TSVector.zero;
            }

#if USING_ORCA || USING_RVO
              TSVector desiredVel = CustomMath.Normalize(_nextTarget - _behaviour.position) * _behaviour.maxSpeed;
            if ((!_bReachedTarget ||(_bReachedTarget&&!PathFindingManager.StopToNodeCenter))&&
                _behaviour.ACAgent != null && target != TSVector.MinValue && target != _behaviour.position)
            {
             
                TSVector2 desiredVelocity = CustomMath.TSVecToVec2(desiredVel);
#if UNITY_EDITO
                if (PathFindingManager.DEBUG)
                {
                   UnityEngine.Vector3 start = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.map._startPos) / GridMap.GetNodeSize().AsFloat();
                   UnityEngine.Vector3 end = CustomMath.TsVecToVector3(target+ _behaviour.map._startPos) / GridMap.GetNodeSize().AsFloat();
                   UnityEngine.Debug.DrawLine(start, end, UnityEngine.Color.green, 0.08f);
                }
#endif
                if (_behaviour.ActiveORCA)
                {
#if USING_ORCA
                    FP x = FP.Zero;
                    FP y = FP.Zero;
                    if (_behaviour.baseData.random != null)
                    {
                        x = _behaviour.baseData.random.Next(-1, 1) / 100;
                        y = _behaviour.baseData.random.Next(-1, 1) / 100;
                    }
                    _behaviour.ACAgent.prefVelocity_ = CustomMath.TSVecToVec2(_behaviour.preVelocity) + new TSVector2(x, y);
                    // _behaviour.OrcaAgent.Loop(desiredVelocity, frameTime);
                    // orcaVel = CustomMath.TSVec2ToVec(_behaviour.OrcaAgent.velocity_);     
#elif USING_RVO
                    _behaviour.ACAgent.Position = CustomMath.TSVecToVec2(_behaviour.position);
                    _behaviour.ACAgent.SetTarget(CustomMath.TSVecToVec2(target), desiredSpeed, _behaviour.maxSpeed);
                    _behaviour.ACAgent.BufferSwitch();
#endif

                    // orcaVel = orcaVel.normalized * _behaviour.maxSpeed;             
                }
            }
#endif
            //}  
            base.SetDesiredPosOrVelocity(deltaTime, w,bSkipBoids);
        }
        int c_maxObstacleCount = 20;

        public int GetQuadrant(FP cos,FP sin)
        {
            if(sin>=0 && cos>0)
            {
                return 1;
            }
            else if(sin > 0 && cos <=0)
            {
                return 2;
            }
            else if (sin <= 0 && cos < 0)
            {
                return 3;
            }
            else //if (sin > 0 && cos >= 0)
            {
                return 4;
            }
        }
        public bool IsSmaller(CircleObstacleAngleData a, CircleObstacleAngleData b)
        {
            return IsSmaller(a.quadrant,b.quadrant,a.cos,b.cos);
        }
        public bool  IsSmaller(int quadrant1, int quadrant2,FP cos1,FP cos2)
        {
            if(quadrant1 < quadrant2)
            {
                return true;
            }
            else if(quadrant1== quadrant2)
            {
                switch(quadrant1)
                {
                    case 1:
                    case 2:
                        return cos1 > cos2;
                    case 3:
                    case 4:
                        return cos1 < cos2;
                    default:
                        return true;
                }
            }
            else
            {
                return false;
            }
        }
        public void InsertSort(CircleObstacleAngleData ob, List<CircleObstacleAngleData> obs)
        {
            obs.Add(CircleObstacleAngleData.max);

            //// Insert the agent into the ordered list of neighbours
            int i = obs.Count - 1;
            CircleObstacleAngleData data = obs[i];

            if (IsSmaller(ob,data))
            {
                while (i != 0 && IsSmaller(ob, obs[i - 1]))
                {
                    obs[i] = obs[i - 1];
                    i--;
                }
                obs[i] = ob;
            }
        }
        public bool ObstacleRayCastCallback( int id)//
        {
            if(_circleObstacles.Count>c_maxObstacleCount)
            {
                return false;
            }
            CircleObstacle ob= _map.GetObstaclesByProxyId(id);
           // if((ob._center- CustomMath.TSVecToVec2(_behaviour.position)).LengthSquared()<_behaviour.baseData.neighbourRadiusSqr)
            {
                CircleObstacleAngleData data = new CircleObstacleAngleData();
                data._center = ob._center;
                data._radius = ob._radius;
                TSVector dir = (CustomMath.TSVec2ToVec(ob._center) - _behaviour.position);
                data.cos = _preVelDirection* dir.normalized;//DOT->COS  //_behaviour.preVelocity.normalized
                data.sin = CustomMath.det(_preVelDirection, dir);
                data.quadrant = GetQuadrant(data.cos,data.sin); // _circleObstacles.Add(data);
                data._blockType =EBlockType.obstacle;
                InsertSort(data, _circleObstacles);
            }           
            return true;
        }

        public override void CalculateFinalVelocity(FP deltaTime, bool bSkipBoids = false)
        {
            TSVector desiredV;
            FP dFactor;
            FP maxTime;
            FP obstacleMaxTime = FP.Zero;

#if !DYNAMIC_FORCE
                TSVector dA = DesiredAcc(_nextTarget, out desiredV,out dFactor,out maxTime);//_targetPos
                _behaviour.preVelocity = desiredV;
                return;
#else
            TSVector dA = TSVector.zero;
#endif
            {
                FP fDtSecond = deltaTime;
#if USING_ORCA || USING_RVO
    if (_behaviour.ActiveORCA)
#if USING_ORCA
                TSVector orcaVel =CustomMath.TSVec2ToVec(_behaviour.ACAgent.velocity_);// TSVector.zero;
#elif USING_RVO
                TSVector vec = CustomMath.TSVec2ToVec(_behaviour.ACAgent.CalculateMovementDelta(
                    CustomMath.TSVecToVec2(_behaviour.position), fDtSecond));
                TSVector orcaVel = vec/ fDtSecond;// CustomMath.TSVec2ToVec(_behaviour.OrcaAgent.);// TSVector.zero;
                        }
#endif
#endif

                if (!_bReachedTarget)
                {
                    TSVector pos = _behaviour.position;// ((GridMap)(_map)).GetGridFloatCoord(_behaviour.position);
                    if (_behaviour.group != null)
                    {
                        TSVector velocity = _behaviour.preVelocity;
#if USING_ORCA || USING_RVO
                        velocity=orcaVel;
#endif
                        TSVector a = ApplyForce(deltaTime, TSVector.zero, velocity, pos,  _behaviour.group._agents, bSkipBoids, false);
                        _behaviour.preVelocity = CustomMath.Normalize(_behaviour.preVelocity + a * fDtSecond) * _behaviour.maxSpeed;
                    }
                    else
                    {
                        TSVector realAcc = TSVector.zero;//real acceleration

                        {
#if USING_ORCA || USING_RVO
                            FP speed = orcaVel.magnitude;
                            if(speed>_behaviour.maxSpeed*ORCA_SPEED_FACTOR)
                            {
                                orcaVel = CustomMath.Normalize(orcaVel) * _behaviour.maxSpeed * ORCA_SPEED_FACTOR;
                            }
#endif
                            if(PathFindingManager.c_useAvoidUnit)
                            {
                                int preBoidsType = _activeBoids;
                                _activeBoids =(int)EBoidsActiveType.seperation;// (int)EBoidsActiveType.collisionAvoidance | 
                            

#if USING_ORCA || USING_RVO
                                velocity=orcaVel;
#endif
#if UNITY_EDITOR
                                if (_showDebug)
                                {
                                    UnityEngine.Debug.Log("______new calculate");
                                }
#endif
                              
                                dA = DesiredAcc(_nextTarget, out desiredV,out dFactor,out maxTime);//_targetPos
                                obstacleMaxTime = FP.One;// TSMath.Min(maxTime, FP.One);
                                maxTime = FP.One;// TSMath.Min(maxTime, FP.One);
                                                 // TSVector dDir =_targetPos-_behaviour.position;
                           
                                if (dA!=TSVector.zero || desiredV!=TSVector.zero)
                                {
                                    bool bIgnoreCollision = _nodeIdxIgnoreCollisionCount > c_max_nodeIdxIgnoreCollisionCount;
                                    if (_behaviour.baseData.pfm.CurTime<_checkRunningIdxEndTime)
                                    {
                                        int nodeIdx=_map.GetGridNodeId(_behaviour.position);
                                        int count= _listNodeIdx.Count;
                                        if (count>0 && nodeIdx == _listNodeIdx[count-1])
                                        {

                                        }
                                        else if(_nodeIdxIgnoreCollisionCount<= c_max_nodeIdxIgnoreCollisionCount)
                                        {
                                            _listNodeIdx.Add(nodeIdx);
                                            if (_nodeIdxForIgnoreCollision == nodeIdx)
                                            {
                                                _nodeIdxIgnoreCollisionCount++;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if(_listNodeIdx.Count==1 
                                            || _nodeIdxIgnoreCollisionCount > c_max_nodeIdxIgnoreCollisionCount)
                                        {
                                            _nodeIdxIgnoreCollisionCount = c_max_nodeIdxIgnoreCollisionCount + 1;
                                            bIgnoreCollision = true;
                                        }                   
                                        if(bIgnoreCollision)
                                        {
                                            _checkRunningIdxEndTime = _behaviour.baseData.pfm.CurTime + 1000000;
                                        }
                                        else
                                        {
                                            if (_nodeIdxIgnoreCollisionCount < c_max_nodeIdxIgnoreCollisionCount)
                                            {
                                                _nodeIdxIgnoreCollisionCount = 0;
                                            }
                                            //_nodeIdxForIgnoreCollision = _towardPathIdx;
                                            _listNodeIdx.Clear();
                                            _nodeIdxForIgnoreCollision = _map.GetGridNodeId(_behaviour.position);
                                            _listNodeIdx.Add(_nodeIdxForIgnoreCollision);
                                            _checkRunningIdxEndTime = _behaviour.baseData.pfm.CurTime + 5000;
                                        }
                                    
                                    }
                                    if(bIgnoreCollision)
                                    {
                                        _behaviour.preVelocity = desiredV;
                                        return;
                                    }
                                                                      
                                    //if (/*isNearStaticAgent &&*/ !isCollidering)//8 directions  isTminStaticAgent
                                    
                                        TSVector targetPos = _nextTarget;
                                        bool isNextTargetBlocked = true;
                                        if (_vecRunningPath.Count==2)
                                        {
                                            targetPos = targetPos - desiredV*_behaviour.invMaxSpeed*_behaviour.colliderRadius*3;
                                            isNextTargetBlocked = IsBlockedBetween2Point(_behaviour.position, targetPos, _behaviour.pathManager._queryStack)!=EBlockType.none;
                                        }
                                       
                                        if(!isNextTargetBlocked)
                                        {
                                            _behaviour.preVelocity = desiredV;
#if UNITY_EDITOR

                                            if (_showDebug)
                                            {
                                                UnityEngine.Debug.Log("NextTarget not Blocked turn to desired direction:" + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                            }
#endif
                                            _currentVelFactor = Max_Vel_Factor;
                                            return;
                                        }
                                        FP dstSqr = GridMap.blockDirDst * GridMap.blockDirDst;
                                        _circleObstacles.Clear();
                                        FP dstMin = GridMap.blockDirDst - _behaviour.colliderRadius;
                                        FP targetDstSqr = (_nextTarget - _behaviour.position).sqrMagnitude;
                                        if (targetDstSqr < dstMin)
                                        {
                                            dstMin = TSMath.Sqrt(targetDstSqr);                                           
                                        }
                                        TSVector velNorm = _preVelDirection;// _behaviour.preVelocity.normalized;
                                        _map.CalculateObstacleNeighbours(_behaviour.position, dstMin, _obstacleCallBack, _behaviour.pathManager._queryStack);
                                        int neighbourCount = _behaviour.neighbours.Count;

                                      
                                        TSVector desiredVelNorm = desiredV * _behaviour.invMaxSpeed;
                                        TSVector baseVelNorm = velNorm;// desiredVelNorm;

                                      
                                        for (int k=0;k< neighbourCount;k++)
                                        {
                                            IAgentBehaviour iAgent = _behaviour.neighbours[k];
                                            //future pos
                                            TSVector fPos = iAgent.position;
                                            TSVector dir = (iAgent.position - _behaviour.position);
                                            if (iAgent.agent!=null && dir* velNorm > 0)//<90 degree
                                            {
                                                fPos= iAgent.position+ iAgent.velocity* CustomMath.FPHalf;
                                            }                                          
                                            if(iAgent.agent == null &&(fPos - _behaviour.position).sqrMagnitude< dstSqr)
                                            {
                                                CircleObstacleAngleData data = new CircleObstacleAngleData();
                                                data._center =CustomMath.TSVecToVec2(iAgent.position);
                                                data._radius = iAgent.baseData.colliderRadius;
                                      
                                                data.cos = baseVelNorm * dir.normalized;//DOT->COS
                                                data.sin = CustomMath.det(baseVelNorm, dir);
                                                data.quadrant = GetQuadrant(data.cos,data.sin);
                                                if(iAgent.agent == null)
                                                {
                                                    data._blockType =EBlockType.staticAgent ;
                                                }
                                                else
                                                {
                                                    data._blockType = EBlockType.agent;
                                                }
                                                
                                                InsertSort(data,_circleObstacles);
                                            }                                           
                                        }
                                        int cCount = _circleObstacles.Count;
                                        TSVector resDir = TSVector.zero;
                                        if (cCount > 1)
                                        {
                                            _currentVelFactor = Max_Vel_Factor;
                                            _circleObstacles.Add(_circleObstacles[0]);
                                            //
                              
                                            if (FindClearDir(ref resDir, velNorm,desiredVelNorm,isNextTargetBlocked))
                                            {
                                            //  _behaviour.preVelocity = resDir.normalized* _behaviour.baseData.maxSpeed;
#if UNITY_EDITOR
                                            if (_showDebug)
                                                {                                  
                                                    UnityEngine.Debug.Log("loop find clear dir >1:" + ","
                                                        + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                                }
#endif
                                               // return;
                                            }
                                            else
                                            {
                                               // _behaviour.preVelocity = _behaviour.preVelocity *FP.EN1*3;
                                               // return;
                                            }
                                        }
                                        else if(cCount==1)
                                        {
                                            // _currentVelFactor = Max_Vel_Factor;
                                            _currentVelFactor = TSMath.Max(1, _currentVelFactor * FP.EN1 * 8);
                                            TSVector blockDir =CustomMath.TSVec2ToVec(_circleObstacles[0]._center)-_behaviour.position;
                                            resDir = _behaviour.preVelocity + desiredV;//
                                            if(resDir*blockDir<=0)
                                            {
                                                //_behaviour.preVelocity = resDir.normalized * _behaviour.maxSpeed;
                                            }
                                            else if(_behaviour.preVelocity * blockDir<=0)
                                            {
                                                //  _behaviour.preVelocity = _behaviour.preVelocity;
                                                resDir = _behaviour.preVelocity;
                                            }
                                            else
                                            {
                                                FP cosVelDesired = velNorm * desiredVelNorm;
                                                FP dst90Sqr = _behaviour.baseData.colliderRadius * 3;
                                                dst90Sqr = dst90Sqr * dst90Sqr;

                                                FP blockDstSqr = blockDir.sqrMagnitude;
                                                TSVector blockDirNorm = blockDir.normalized;
                                                TSVector perDir = CustomMath.perpendicular(blockDirNorm);
                                                TSVector curResDir = perDir;

                                                FP cosMax = curResDir * velNorm;
                                                resDir = curResDir;
                                                FP cos = 0;
                                                //
                                               if (_circleObstacles[0]._blockType==EBlockType.agent ||
                                                 (_circleObstacles[0]._blockType == EBlockType.staticAgent &&blockDstSqr > dst90Sqr))
                                                {
                                                    curResDir = 2 * perDir + blockDir;
                                                    curResDir = curResDir.normalized;
                                                    cos = curResDir * velNorm;
                                                    if (cos > cosMax)
                                                    {
                                                        cosMax = cos;
                                                        resDir = curResDir;
                                                    }
                                                }
                                               
                                                //
                                                curResDir = TSVector.zero- perDir;
                                                cos = curResDir * velNorm;
                                                if (cos > cosMax)
                                                {
                                                    cosMax = cos;
                                                    resDir = curResDir;
                                                }
                                            //
                                            if (_circleObstacles[0]._blockType == EBlockType.agent ||
                                                (_circleObstacles[0]._blockType == EBlockType.staticAgent && blockDstSqr > dst90Sqr))
                                            {
                                                    curResDir = 2 * (TSVector.zero - perDir) + blockDir;
                                                    curResDir = curResDir.normalized;
                                                    cos = curResDir * velNorm;
                                                    if (cos > cosMax)
                                                    {
                                                        cosMax = cos;
                                                        resDir = curResDir;
                                                    }
                                                }
                                                
                                                //_behaviour.preVelocity = resDir * _behaviour.maxSpeed;
                                            }                        

#if UNITY_EDITOR
                                            if (_showDebug)
                                            {
                                                CircleObstacleAngleData data = _circleObstacles[0];
                                                UnityEngine.Vector3 start = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                                                UnityEngine.Vector3 end = CustomMath.TsVecToVector3(CustomMath.TSVec2ToVec(data._center) + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                                                UnityEngine.Color color = UnityEngine.Color.white;
                                                UnityEngine.Debug.DrawLine(start, end, color, 0.02f);

                                                start = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                                                end = CustomMath.TsVecToVector3(_behaviour.position + desiredVelNorm * GridMap.SCALE + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                                                color = UnityEngine.Color.blue;

                                                UnityEngine.Debug.DrawLine(start, end, color, 0.02f);
                                            }
#endif
#if UNITY_EDITOR
                                            if (_showDebug)
                                            {
                                                UnityEngine.Debug.LogError("loop find clear dir 1:" + ","
                                                    + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                            }
#endif
                                            //return;
                                        }
                                        else{
                                            //TSVector preVelNorm = _behaviour.preVelocity.normalized;
                                            //FP cos = preVelNorm * desiredV*_behaviour.invMaxSpeed;
                                            //if(cos< CustomMath.FPHalf)//>45
                                            {
                                                _currentVelFactor = TSMath.Max(1, _currentVelFactor * FP.EN1 * 8);
                                                resDir = _behaviour.preVelocity * _currentVelFactor + desiredV;// 
                                                //_behaviour.preVelocity = resDir.normalized * _behaviour.maxSpeed;
                                            }       
#if UNITY_EDITOR
                                            if (_showDebug)
                                            {
                                                UnityEngine.Debug.Log("loop find clear dir desiredV:" + ","
                                                    + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                            }
#endif
                                        }
                                        TSVector resNorm = resDir.normalized;
                                        TSVector toAcc = _behaviour.baseData.maxForce * _behaviour.baseData.invMass * resNorm;
                                        TSVector acc = ApplySeperateForce(toAcc,_behaviour.neighbours, false);
                                    
                                        _behaviour.preVelocity = _behaviour.preVelocity + acc * fDtSecond;
                                        if (_behaviour.preVelocity.sqrMagnitude > _behaviour.maxSpeed * _behaviour.maxSpeed)//|| TSVector.Dot(_behaviour.velocity, a) > 0
                                        {
                                            _behaviour.preVelocity = CustomMath.Normalize(_behaviour.preVelocity) * _behaviour.maxSpeed;
                                        }
                                        if (_preVelDirection * _behaviour.preVelocity <= 0)
                                        {
                                            _preVelDirection = _behaviour.preVelocity.normalized;
                                        }
                                }
                                else if (desiredV == TSVector.zero)
                                {
                                    _behaviour.preVelocity = TSVector.zero;
                                }
                                _activeBoids = preBoidsType;
                              
                            }
                            else
                            {
#if USING_ORCA || USING_RVO
                                _behaviour.preVelocity = orcaVel;
#endif
                            }
                        }
                    }
#if USING_ORCA
                    _velocityChangedByORCA = _behaviour.ACAgent.prefVelocity_ != _behaviour.ACAgent.velocity_;
#endif
                    //
                }
            }         
        }
        static readonly FP s_angle45 = FP.EN1 * 7;
        static readonly FP s_angle30 = FP.EN2 * 85;
        public bool FindClearDir(ref TSVector clearDir, TSVector velNorm,TSVector desiredVelNorm, bool isNextTargetBlocked)
        {
            int count = _circleObstacles.Count;
            FP clearCos = -10;
            FP dst = _behaviour.baseData.colliderRadius * 36 * FP.EN1;
            FP dstSqr = dst* dst;
            FP dst90Sqr = _behaviour.baseData.colliderRadius * 28 * FP.EN1; ;
            dst90Sqr = dst90Sqr * dst90Sqr;
            FP cosVelDesired = velNorm * desiredVelNorm;
            bool needChangeDirForDesired = cosVelDesired < 0;
      
            int resIdx = -1;
            int curPosIdx = _map.GetGridNodeId(_behaviour.position);
            TSVector curWpos = _map.GetWorldPosition(curPosIdx);
            for (int i=1;i<count;i++)
            {
                CircleObstacleAngleData data = _circleObstacles[i];
                CircleObstacleAngleData preData = _circleObstacles[i-1];
                TSVector todir = CustomMath.TSVec2ToVec(data._center - preData._center);
                TSVector preDir= CustomMath.TSVec2ToVec(preData._center)-_behaviour.position;
                TSVector curDir = CustomMath.TSVec2ToVec(data._center) - _behaviour.position;
                if(data._blockType==EBlockType.obstacle)//convert coord
                {
                    curDir = CustomMath.TSVec2ToVec(data._center)- curWpos;
                }
                if (preData._blockType == EBlockType.obstacle)//convert coord
                {
                    preDir = CustomMath.TSVec2ToVec(preData._center) - curWpos;
                }
                TSVector resDir = TSVector.zero;
                bool isClear = true;
                //bool bForceChange
                FP det = CustomMath.det(preDir, curDir);
                //
#if UNITY_EDITOR
                if (_showDebug)
                {
                    UnityEngine.Vector3 start = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    UnityEngine.Vector3 end = CustomMath.TsVecToVector3(CustomMath.TSVec2ToVec(data._center)+ _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    UnityEngine.Color color = UnityEngine.Color.white;
                    if(i== count -1)
                    {
                        color = UnityEngine.Color.yellow;
                    }
                    else if(i==count-2)
                    {
                        color = UnityEngine.Color.red;
                    }
                    UnityEngine.Debug.DrawLine(start, end, color, 0.02f);

                    end = CustomMath.TsVecToVector3(_behaviour.position + desiredVelNorm* GridMap.SCALE * 2 + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    color = UnityEngine.Color.blue;                  
                    UnityEngine.Debug.DrawLine(start, end, color, 0.02f);

                    end = CustomMath.TsVecToVector3(_behaviour.position + velNorm * GridMap.SCALE * 2 + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    color = UnityEngine.Color.gray;
                    UnityEngine.Debug.DrawLine(start, end, color, 0.02f);
                    //start = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.map._startPos) / GridMap.GetNodeSize().AsFloat();
                    //end = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.velocity + _behaviour.map._startPos) / GridMap.GetNodeSize().AsFloat();
                    //color = UnityEngine.Color.black;

                    // UnityEngine.Debug.DrawLine(start, end, color, 0.1f);
                }
#endif
                bool bBetween = i == count - 1;
                FP curDstSqr = curDir.sqrMagnitude;
                FP preDstSqr = preDir.sqrMagnitude;
                curDir = curDir.normalized;
                preDir = preDir.normalized;
                FP cosCurPre = curDir * preDir;
                if (det <= 0 ||(det>0 && todir.sqrMagnitude > dstSqr && cosCurPre < s_angle45))
                {
                    bool desiredDirBetween = false;
                    if(det<=0)
                    {
                        FP cosCur = curDir * desiredVelNorm;
                        FP cosPre = preDir * desiredVelNorm;

                        if((cosCur<CustomMath.FPHalf || curDstSqr> dst90Sqr) && 
                            (cosPre<CustomMath.FPHalf || preDstSqr > dst90Sqr))//>60
                        {
                            TSVector dDir = (desiredVelNorm).normalized;
                            FP cos1 = dDir * preDir.normalized;// *_behaviour.invMaxSpeed;
                            FP sin1 = CustomMath.det(dDir, preDir);
                            int quadrant1 = GetQuadrant(cos1, sin1);

                            FP cos2 = dDir * curDir.normalized;// * _behaviour.invMaxSpeed;
                            FP sin2 = CustomMath.det(dDir, curDir);
                            int quadrant2 = GetQuadrant(cos2, sin2);

                            desiredDirBetween = !IsSmaller(quadrant1, quadrant2, cos1, cos2);
                        }
                    }               

                    if (det<=0 && desiredDirBetween)
                    {
                        clearDir = desiredVelNorm;
                        resIdx = i;
                        break;
                    }
                    else
                    {
                        if(i != count - 1 &&  i>1)
                        {
                            TSVector lPreDir = CustomMath.TSVec2ToVec(data._center - _circleObstacles[i - 2]._center);
                            FP sqrLPre= lPreDir.sqrMagnitude;
                            //FP 
                            if (sqrLPre < dstSqr)//|| cosCurPre > s_angle45
                            {
                                TSVector lDir = CustomMath.TSVec2ToVec(_circleObstacles[i - 2]._center)-_behaviour.position;
                                FP detLPre = CustomMath.det(lDir, curDir);
                                if(detLPre>0)
                                {
                                    continue;
                                }                             
                            }
                        }
                        FP cosCur = curDir * velNorm;
                        FP cosPre = preDir * velNorm;

                        if (cosCur < s_angle45 && cosPre < s_angle45 && bBetween)
                        {
                            resDir = _behaviour.preVelocity;
                        }
                        else if (det > 0 && cosCurPre >= -s_angle45)//<135 ,middle 
                        {
                            resDir = curDir + preDir;
                        }
                        else
                        {
                            TSVector perDir = CustomMath.perpendicular(curDir);
                            bool nearDir = perDir * preDir >= 0;
                            TSVector curResDir = ((nearDir && det > 0) || (!nearDir && det <= 0)) ? perDir : (TSVector.zero - perDir); //TSVector.zero - (curDir.normalized * CustomMath.FPHalf + preDir.normalized);//curDir.normalized*CustomMath.FPHalf - (curDir.normalized+preDir.normalized)

                            FP cos0 = curResDir * velNorm; ;
                            FP maxCos = cos0;
                            resDir = curResDir;

                            if (data._blockType == EBlockType.agent ||
                                (data._blockType == EBlockType.staticAgent && curDstSqr > dst90Sqr))
                            {
                                curResDir = 2*curResDir + curDir;
                                curResDir = curResDir.normalized;
                                cos0 = curResDir * velNorm; ;
                                if (cos0 > maxCos)
                                {
                                    resDir = curResDir;
                                    maxCos = cos0;
                                }
                            }
                           

                            //previous direction   
                            perDir = CustomMath.perpendicular(preDir.normalized);
                            nearDir = perDir * curDir >= 0;
                            TSVector preResDir = ((nearDir && det > 0) || (!nearDir && det <= 0)) ? perDir : (TSVector.zero - perDir);
                            //
                            cos0 = preResDir * velNorm; ;
                            if (cos0 > maxCos)
                            {
                                resDir = preResDir;
                                maxCos = cos0;
                            }
                            if (preData._blockType == EBlockType.agent ||
                                 (preData._blockType == EBlockType.staticAgent && preDstSqr > dst90Sqr))
                            {
                                preResDir = 2 * preResDir + preDir;
                                preResDir = preResDir.normalized;

                                cos0 = preResDir * velNorm; ;
                                if (cos0 > maxCos)
                                {
                                    resDir = preResDir;
                                    maxCos = cos0;
                                }
                            }
                           
                            //middle
                            TSVector middleDir = (curDir + preDir).normalized;
                            if (det <= 0)
                            {
                                middleDir = TSVector.zero - middleDir;
                            }
                            cos0 = middleDir * velNorm; ;
                            if (cos0 > maxCos)
                            {
                                resDir = middleDir;
                                maxCos = cos0;
                            }
                            // resDir = GetResDir(curResDir,preResDir,middleDir,velNorm,desiredVelNorm,cosVelDesired);
                        }
                    }
                   
                }                  
                else 
                {
                    isClear = false;
                }
                if(!isClear)
                {
                    continue;
                }
                FP cos = velNorm * resDir.normalized;
                if(cos> clearCos && isClear)
                {
                    clearCos = cos;
                    clearDir = resDir;
                    resIdx = i;
                }
            }
            if (resIdx > 0)
            {
#if UNITY_EDITOR
                if (_showDebug)
                {
                    UnityEngine.Vector3 start = CustomMath.TsVecToVector3(_behaviour.position + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    UnityEngine.Vector3 end = CustomMath.TsVecToVector3(_behaviour.position + clearDir*2* GridMap.SCALE + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    UnityEngine.Color color = UnityEngine.Color.green;

                    UnityEngine.Debug.DrawLine(start, end, color, 0.02f);
                 

                    color = UnityEngine.Color.magenta;
                    end = CustomMath.TsVecToVector3(CustomMath.TSVec2ToVec(_circleObstacles[resIdx]._center) + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    UnityEngine.Debug.DrawLine(start, end, color, 0.02f);
                    
                    end = CustomMath.TsVecToVector3(CustomMath.TSVec2ToVec(_circleObstacles[resIdx-1]._center) + _behaviour.map._startPos) / GridMap.SCALE.AsFloat();
                    UnityEngine.Debug.DrawLine(start, end, color, 0.02f);
                    UnityEngine.Debug.LogError("count:" + count + ",resIdx:" + resIdx);
                }
#endif
                return true;
            }
            else
            {
#if UNITY_EDITOR
                if (_showDebug)
                {
                    UnityEngine.Debug.LogError("count:" + count + ",resIdx:" + resIdx);
                }
#endif
            }
            return false;
        }
        public EBlockType IsBlockedBetween2Point(TSVector pos1,TSVector pos2,Stack<int> stack)
        {

            stack.Clear();
            TSVector blockedPos = TSVector.zero;
            bool hasObstacle = _behaviour.map.IsBlockedByObstacleBetween2Point(_behaviour.position, pos2
                , stack,ref blockedPos);
            if(hasObstacle)
            {
                return EBlockType.obstacle;
            }
            stack.Clear();
            bool hashNeighbour = _behaviour.baseData.pfm.HasNeighboursBetween2Point(_behaviour.proxyId, pos1
                , pos2, stack);
            if(hashNeighbour)
            {
                return EBlockType.agent;
            }
            return EBlockType.none;
        }

        /// Relaxation time for the driving force
       // protected static readonly FP _ksi = FP.One * 54 / 100;

        public int TowardPathIdx
        {
            get
            {
                return _towardPathIdx;
            }

            set
            {
                _towardPathIdx = value;
            }
        }

  
        public override void Loop(FP frameTime, IsPosWalkable isWalkable = null, bool bSkipBoids = false)
        {
            base.Loop(frameTime, _isPosWalkable, bSkipBoids);
        }
        public bool IsPosWalkable(TSVector pos)
        {
            IInt2 vec=IInt2.zero;
            if(!_map.GetGridCoord(pos, ref vec))
            {
                return false;
            }
            return IsWalkable(vec.x, vec.y, false);
        }
        public bool IsWalkable(int x,int y,bool bSkip)
        {
            //return true;
            int nodeID = _map.GetIdx(x, y);
            if (_map.GetGridNodeId(_startPos) == nodeID)
            {
                return true;
            }
            if (_velocityChangedByORCA)
            {
                if(!PathFindingManager.StopToNodeCenter)
                {
                    int pIdx = _map.GetGridNodeId(_behaviour.position);
                    TSVector pos = _map.GetWorldPosition(nodeID);
                    if((pos-_behaviour.position).sqrMagnitude<_behaviour.colliderRadius* _behaviour.colliderRadius)//
                    {
                        return true;
                    }
                    int sIdx = pIdx;
                    int eIdx = -1;
                    if (_targetPos != TSVector.MinValue)
                    {
                        sIdx = pIdx;
                        eIdx = _map.GetGridNodeId(_targetPos);
                    }
                    bool blocked = !_map.IsWalkable(pIdx, pIdx == sIdx || pIdx == eIdx)
                               || !_map.IsWalkable(nodeID, nodeID == sIdx || nodeID == eIdx)
                               || _map.IsDiagonalBlocked(pIdx, nodeID, sIdx, eIdx, pIdx);
                    if (blocked)
                    {
                        return false;
                    }
                }                
            }
            return _map.IsWalkable(x,y,bSkip);
        }
    }
}
