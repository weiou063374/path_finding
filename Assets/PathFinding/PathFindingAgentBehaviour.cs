///author:huwei
///date:2018.4.17
//#define SEARCH_ENEMY
#define USING_DYNAMIC_TREE
//#define NONE_FLOW_FIELD
//#define USING_ORCA
#define FORCE_BASED
#if UNITY_EDITOR
using UnityEngine;
#endif
using System.Collections;
using System.Collections.Generic;
using TrueSync;

namespace PathFinding
{
    public enum EAgentType
    {
        none,
        flowFiled,
        astar,
        ingoreObstacle
        //   moveToSpec
    }
    public enum EMoveToSpecStatus
    {
        normal,
        // normal,
        // needMoveToSpecPos,
        movingToSpecPos,
        onSpecPos
    }
    //public enum EAStarPathType
    //{
    //    none,
    //    quickPath,
    //    fullPath,
    //    splicePath,
    //    priorityPath
    //}
    public  delegate void RereachTargetFail(int idx);
    public struct AgentBaseData
    {
        public int id;
        public int mapId;
        public System.Int64 playerId;
        public EAgentType eAgentType;
        public TSVector position;
        public TSVector defaultTargetPos;
        public LoopCallback loopCallback;
        public byte boidsType;//= (byte)EBoidsActiveType.all;
        public int collidesWithLayer;// = 1
        public FP maxSpeed;
        public FP viewRadius;
        public FP neighbourRadius;
        public FP colliderRadius;
        public PathFindingManager pfm;
        public TSRandom random;
        public FP maxForce;
        public FP mass;

        //
        public FP neighbourRadiusSqr;
        public FP maxForceSqr;
        public FP invMass;
        public byte groupId;
        public RereachTargetFail targetFailCallback;
        public void Reset()
        {
            id = -1;
            mapId = -1;
            playerId = -1;
            eAgentType = EAgentType.none;
            position = TSVector.zero;
            defaultTargetPos = TSVector.MaxValue;
            loopCallback = null;
            boidsType = 0;
            collidesWithLayer = 1;
            maxSpeed = FP.One;
            neighbourRadius = FP.Zero;
            colliderRadius = FP.Zero;
            maxForce = FP.Zero;
            mass = FP.One;
            pfm = null;
            maxForceSqr = FP.Zero;
            invMass = FP.One;
            groupId = 0;
            targetFailCallback = null;
        }
    }
    public delegate void LoopCallback(FP frameTime, TSVector position, TSVector velocity);
    public class PathFindingAgentBehaviour : IAgentBehaviour, IResetable
    {
        public static readonly FP slowdownDistance = CustomMath.FPHalf * GridMap.SCALE;//none
        public AgentBaseData _baseData;
        public AgentBaseData baseData { get { return _baseData; } }
        public TSVector _defaultTargetPos { get { return _baseData.defaultTargetPos; } }
        public bool _hasNeighbourTargetedDefaultPos = false;// need change to a start due to flow field poorly avoidance unit

        public const int C_MAXFORCE = 20;
        public static int C_DESIRED_DELTATIME = 100;
        public bool _IsTargetPosSetFromInternal = false;

        public void ChangeMaxSpeed(FP speed)
        {
            _baseData.maxSpeed = speed;
            _baseData.maxForce = _baseData.maxSpeed / C_DESIRED_DELTATIME / 3 * _baseData.mass * 1000;
            _baseData.maxForceSqr = _baseData.maxForce * _baseData.maxForce;
        }

        #region data pool
        public static SingleObjectPool<AStarAgent> s_AstarAgent = new SingleObjectPool<AStarAgent>(10);
        public static SingleObjectPool<IgnoreObstacleAgent> s_ignoreObstacleAgent = new SingleObjectPool<IgnoreObstacleAgent>(10);
        public static SingleObjectPool<FlowFieldAgent> s_ffAgent = new SingleObjectPool<FlowFieldAgent>(10);
#if USING_ORCA
        public static SingleObjectPool<ORCAAgent> s_ORCAAgent = new SingleObjectPool<ORCAAgent>(10);
#elif USING_RVO
        public static SingleObjectPool<RVOAgent> s_ORCAAgent = new SingleObjectPool<RVOAgent>(10);
#endif
#if FORCE_BASED
        List<LineObstacle> _neighbourObstacles = new List<LineObstacle>();
        public List<LineObstacle> neighbourObstacles { get { return _neighbourObstacles; } }
#endif
        #endregion

#if !USING_DYNAMIC_TREE
        #region QTDATA
        protected PathFindingAgentBehaviour _next = null;
        public QTData next { set { _next = (PathFindingAgentBehaviour)value; } get { return _next; } }
        #endregion
#else
        AABB _aabb;
        public AABB aabb
        {
            get
            {
                return _aabb;
            }
            set
            {
                _aabb = value;
            }
        }
        int _proxyId = -1;
        DynamicTreeQueryCallback _dynamicQueryCallBack;

        public int proxyId
        {
            get
            {
                return _proxyId;
            }
            set
            {
                _proxyId = value;
            }
        }
#endif
        public TSVector position { set { _baseData.position = value; _baseData.pfm.MoveProxy(this, TSVector.zero); } get { return _baseData.position; } }
        FP _invMaxSpeed = FP.One;
        HashSet<int> _dicUnReachableAgents = new HashSet<int>();
       
#if UNITY_EDITOR&& PATHMANAGER_DEBUG
        public SpriteRenderer _sr = null;
        public string name = "";
#endif
        private PathManager _pathManager = null;
        public PathManager pathManager
        {
            get
            {
                return _pathManager;
            }

            set
            {
                _pathManager = value;
            }
        }

        TSVector _velocity;
        public TSVector velocity
        {
            get
            {
                return _velocity;
            }

            set
            {
                _velocity = value;
            }
        }
        TSVector _preVelocity = TSVector.zero;
        public TSVector preVelocity
        {
            get
            {
                return _preVelocity;
            }

            set
            {
                _preVelocity = value;
            }
        }

        bool _enabled;
        public bool enabled
        {
            get
            {
                return _enabled;
            }

            set
            {
                _enabled = value;
            }
        }
        public FP invMaxSpeed
        {
            get
            {
                return _invMaxSpeed;
            }

            set
            {
                _invMaxSpeed = value;
            }
        }

        public FP maxSpeed
        {
            get
            {
                return _baseData.maxSpeed;

            }

            set
            {
                _baseData.maxSpeed = value;
            }
        }


        public const int maxAllyNeighbours = 30;
        public const int maxEnemyNeighbours = 16;
        bool _hasUpdateNeighbours;
        FP _radiusSqr;
        List<IAgentBehaviour> _allyNeighbours = new List<IAgentBehaviour>();
        public List<FP> _allyNeighboursDstSqr = new List<FP>();
        public FP neighbourRadius { get { return _baseData.neighbourRadius; } set { _baseData.neighbourRadius = value; } }
#if !USING_DYNAMIC_TREE
        public FP InsertNeighbour(QTData data, FP rangeSqr)
        {
            PathFindingAgentBehaviour agent = (PathFindingAgentBehaviour)data;
            // Check  agents collides 
            if (this == agent ||  (agent._baseData.collidesWithLayer & _baseData.collidesWithLayer) == 0) return rangeSqr;
          
            List<IAgentBehaviour> _neighbours = agent.campId ==  _baseData.camp ? _allyNeighbours : null;
            List<FP> neighboursDestSqr = agent.campId == _baseData.camp ? _allyNeighboursDestSqr : null;

            if (_neighbours==null)
            {
                return rangeSqr;
            }
            // 2D distance
            FP distSqr = (agent.position - position).sqrMagnitude;
            int maxNeighbours = agent.campId == _baseData.camp ? maxAllyNeighbours : maxEnemyNeighbours;
            if (distSqr < rangeSqr)
            {
                if (_neighbours.Count < maxNeighbours)
                {
                    //neighbours.Add(agent);
                    //neighboursDestSqr.Add(distSqr);
                    _neighbours.Add(null);
                    neighboursDestSqr.Add(FP.MaxValue);
                }

                //// Insert the agent into the ordered list of neighbours
                int i = _neighbours.Count - 1;
                if (distSqr < neighboursDestSqr[i])
                {
                    while (i != 0 && distSqr < neighboursDestSqr[i - 1])
                    {
                        _neighbours[i] = _neighbours[i - 1];
                        neighboursDestSqr[i] = neighboursDestSqr[i - 1];
                        i--;
                    }
                    _neighbours[i] = agent;
                    neighboursDestSqr[i] = distSqr;
                }

                if (_neighbours.Count == maxNeighbours)
                {
                    rangeSqr = neighboursDestSqr[neighboursDestSqr.Count - 1];
                }
            }
            return rangeSqr;
        }
#endif
        //
        //  static FP agentTimeHorizon = 0;// 1/ new FP(10);
#if !USING_DYNAMIC_TREE
        public void CalculateNeighbours(int camp,Quadtree<QTData> qdTree)
        {
            List<IAgentBehaviour> _neighbours = camp == _baseData.camp ? _allyNeighbours : null;
            List<FP> neighboursDestSqr = camp == _baseData.camp ? _allyNeighboursDestSqr : null;

            int maxNeighbours = camp == _baseData.camp ? maxAllyNeighbours : maxEnemyNeighbours;
            _neighbours.Clear();
            neighboursDestSqr.Clear();
            if (maxNeighbours > 0)
            {
                qdTree.Query(position, maxSpeed, agentTimeHorizon, neighbourRadius, this);
            }
            _hasUpdateNeighbours = true;
    }
#else
        public void CalculateNeighbours()
        {
            if (_baseData.pfm == null)
            {
                return;
            }
            if (_nextCheckActionTime > _baseData.pfm.CurTime && _nextCalNeighboursTime > _baseData.pfm.CurTime)//agent!=null &&
            {
                return;
            }
            if (!_hasUpdateNeighbours)
            {
                _neighbourObstacles.Clear();
                _allyNeighbours.Clear();
                _allyNeighboursDstSqr.Clear();
                _baseData.pfm.CalculateAgentNeighbours(this, _dynamicQueryCallBack, _baseData.neighbourRadius);
                _hasUpdateNeighbours = true;
                _nextCalNeighboursTime = _baseData.pfm.CurTime + 50;
            }
        }
        bool DynamicTreeQueryCallback(int id)
        {
            PathFindingAgentBehaviour agent = _baseData.pfm.GetAgentByProxyId(id);
            bool bOver = true;
            if (agent != null)
            {
                if (id == proxyId)// || agent._baseData.groupId != _baseData.groupId
                {
                    return true;
                }
                FP dstSqr = (agent.position - _baseData.position).sqrMagnitude;

                if (_allyNeighbours.Count < maxAllyNeighbours)//agent.campId == campId && 
                {

                    //  FP dst = _baseData.colliderRadius * 5;
                    if (dstSqr < _baseData.neighbourRadiusSqr)//  dst * dst
                    {
                        InsertSort(agent, dstSqr, _allyNeighbours, _allyNeighboursDstSqr, maxAllyNeighbours);
                    }
                }
            }
            if (_allyNeighbours.Count >= maxAllyNeighbours && bOver)//overflow
            {
                //#if UNITY_EDITOR
                //                // Debug.Log("_allyNeighbours count:"+ _allyNeighbours.Count);
                //#endif
                return false;
            }
            return true;
        }
#endif
        public void InsertSort(PathFindingAgentBehaviour agent, FP dstSqr, List<IAgentBehaviour> nbs, List<FP> neighboursDestSqr, int maxCount)
        {
            //sort
            if (nbs.Count < maxCount)
            {
                //neighbours.Add(agent);
                //neighboursDestSqr.Add(distSqr);
                nbs.Add(null);
                neighboursDestSqr.Add(FP.MaxValue);
            }
            else
            {
                return;
            }

            //// Insert the agent into the ordered list of neighbours
            int i = nbs.Count - 1;
            if (dstSqr < neighboursDestSqr[i])
            {
                while (i != 0 && dstSqr < neighboursDestSqr[i - 1])
                {
                    nbs[i] = nbs[i - 1];
                    neighboursDestSqr[i] = neighboursDestSqr[i - 1];
                    i--;
                }
                nbs[i] = agent;
#if UNITY_EDITOR
                //if (PathFindingManager.DEBUG)
                //{
                //    if (i > 0)
                //    {
                //        if (nbs[i] == nbs[i - 1])
                //        {
                //            UnityEngine.Debug.LogError("DynamicTree error!");
                //        }
                //    }
                //}
#endif
                neighboursDestSqr[i] = dstSqr;
            }
        }
        //
        internal void FindAllyAgents(int camp, List<IAgentBehaviour> list)
        {
            int count = _allyNeighbours.Count;
            for (int i = 0; i < count; i++)
            {
                if (_allyNeighbours[i].playerId == camp)
                {
                    list.Add(_allyNeighbours[i]);
                }
            }
        }
        #region IAgentBehaviour

        private bool _pathBlocked = false;
        public bool pathBlocked
        {
            get
            {
                return _pathBlocked;
            }

            set
            {
                _pathBlocked = value;
            }
        }
        public GridMap map
        {
            get
            {
                return _map;
            }

            set
            {
                _map = value;
            }
        }
        public GridMap _map;
#if UNITY_EDITOR && PATHMANAGER_DEBUG
        public AgentBase _agent = null;
        public AStarAgent _aAgent = null;
#else
        AgentBase _agent = null;
#endif
#if USING_ORCA
        ORCAAgent _orcaAgent = null;
#elif USING_RVO
        RVOAgent _orcaAgent = null;
#endif
        bool _activeORCA = true;
        public AgentBase agent
        {
            get
            {
                return _agent;
            }

            set
            {
                _agent = value;
#if UNITY_EDITOR && PATHMANAGER_DEBUG
                if (_agent is AStarAgent)
                {
                    _aAgent = _agent as AStarAgent;
                }
#endif
            }
        }
        public List<IAgentBehaviour> neighbours
        {
            get
            {
                if (!_hasUpdateNeighbours)
                {
#if !USING_DYNAMIC_TREE
                    _baseData.pfm.CalculateAgentNeighbours(this);
#else
                    CalculateNeighbours();
#endif
                }
                return _allyNeighbours;
            }

            set
            {
                _allyNeighbours = value;
            }
        }
        public System.Int64 playerId
        {
            get
            {
                return _baseData.playerId;
            }

            set
            {
                _baseData.playerId = value;
                if (_baseData.pfm != null)
                {
                    _baseData.pfm.AddCamp(_baseData.playerId);
                }
            }
        }
#if USING_ORCA || USING_RVO
#if USING_ORCA
            public ORCAAgent ACAgent
#elif USING_RVO
            public RVOAgent OrcaAgent
#endif
            {
                get
                {
                    return _orcaAgent;
                }

                set
                {
                    _orcaAgent = value;
                }
            }
#endif
        public FP colliderRadius
        {
            get
            {
                return _baseData.colliderRadius;
            }

            set
            {
                _baseData.colliderRadius = value;
                _radiusSqr = _baseData.colliderRadius * _baseData.colliderRadius;
            }
        }
        AgentGroup _group;
        public AgentGroup group
        {
            get
            {
                return _group;
            }
            set
            {
                _group = value;
            }
        }
        #endregion
        public int _nextCheckActionTime = 0;
        public int _nextCalNeighboursTime = 0;

        // public FP _attackRange = FP.Zero;
        public bool stopMoving = false;
        // public IAgentBehaviour _target = null;
        TSVector _targetPos = TSVector.MaxValue;
        public TSVector _originTargetPos = TSVector.MaxValue;
        public TSVector targetPos
        {
            get { return _targetPos; }
        }

        private TSVector _newTargetPos = TSVector.MinValue;
        FP _targetDistSqr = FP.MaxValue;
        public EMoveToSpecStatus _moveStatus = EMoveToSpecStatus.normal;
        TSVector _moveToSpecPos = TSVector.zero;

        public EAgentType _agentType=EAgentType.none;
        public EAgentType AgentType
        {
            get { return _agentType; }
            set
            {
                //_agentType = value;
                ChangeAgentType(value);
            }
        }

        public bool ActiveORCA
        {
            get
            {
                return _activeORCA;
            }

            set
            {
                _activeORCA = value;
            }
        }

        public int Id
        {
            get
            {
                return _baseData.id;
            }

            set
            {
                _baseData.id = value;
            }
        }

        public TSVector NewTargetPos
        {
            get { return _newTargetPos; }
        }
        public bool CanSetTargetPos(TSVector pos)
        {
            int idx = _map.GetGridNodeId(pos);
            TSVector mpos = _map.GetWorldPosition(idx);
            if (_dicUnReachableAgents.Contains(idx) || !map.IsWalkable(mpos,true))
            {
                return false;
            }
            return true;
        }
        public bool SetNewTargetPos(TSVector pos,bool bSetInternal=false)
        {
            int idx = _map.GetGridNodeId(pos);
            TSVector mpos = _map.GetWorldPosition(idx);
            if (_dicUnReachableAgents.Contains(idx) ||!map.IsWalkable(mpos,true))
            {
                return false;
            }
            _originTargetPos = pos;
             _IsTargetPosSetFromInternal = bSetInternal;
            pos = mpos;
            _newTargetPos = pos;
          
            if (_newTargetPos != _defaultTargetPos)
            {
                _hasNeighbourTargetedDefaultPos = false;
            }
            return true;
        }
        #region for thread buffer
        ERepathType _needRepath = ERepathType.none;
        EPathType _aStarPathType = EPathType.none;
        bool _bTryToFindPath = false;
        int _revsAvailable = 0;
        int _NextReqRevsFrameTime = 0;
        #endregion

        public void Init(AgentBaseData data)
        {
            data.mass = 1;
            data.invMass = 1 / data.mass;
            //data.maxForce = data.maxSpeed / C_DESIRED_DELTATIME / 3 * data.mass * 1000;
            //data.maxForceSqr = data.maxForce * data.maxForce;
            data.neighbourRadiusSqr = data.neighbourRadius * data.neighbourRadius;
            _baseData = data;
            ChangeMaxSpeed(_baseData.maxSpeed);
            TSVector2 w = new TSVector2(_baseData.colliderRadius, _baseData.colliderRadius);
            _radiusSqr = _baseData.colliderRadius * _baseData.colliderRadius;
            _aabb.lowerBound = CustomMath.TSVecToVec2(_baseData.position) - w;
            _aabb.upperBound = CustomMath.TSVecToVec2(_baseData.position) + w;

            _dynamicQueryCallBack = this.DynamicTreeQueryCallback;
            _map = GridMapManager.instance.GetMap(_baseData.mapId);
            invMaxSpeed = 1 / maxSpeed;
            bool bEnable = enabled;
            // OnEnable();
            ChangeAgentType(_baseData.eAgentType);

            if (_agentType == EAgentType.flowFiled && bEnable)
            {
                _agent.TargetPos = _defaultTargetPos;
                _agent._originTargetPos= _defaultTargetPos;
            }
            if (_baseData.pfm != null)
            {
                _baseData.pfm.AddAgent(this);
            }
#if UNITY_EDITOR&& PATHMANAGER_DEBUG
            name = _baseData.id.ToString();
#endif
        }
        public void OnEnable()
        {
            _enabled = true;

            _allyNeighbours.Clear();
            _allyNeighboursDstSqr.Clear();
#if USING_ORCA || USING_RVO
                if(_activeORCA && _orcaAgent==null)
                {
#if USING_ORCA
                    _orcaAgent = s_ORCAAgent.New();
                    _orcaAgent._behaviour = this;
#elif USING_RVO
                    _orcaAgent.Radius =_baseData.colliderRadius;
                    _orcaAgent.AgentTimeHorizon = FP.One/4;
                    _orcaAgent.ObstacleTimeHorizon = FP.One / 4;
                    _orcaAgent.Locked = false;
                    _orcaAgent.MaxNeighbours = maxAllyNeighbours;
                    _orcaAgent.DebugDraw = false;
                    _orcaAgent.Layer = Pathfinding.RVO.RVOLayer.DefaultAgent;
                    _orcaAgent.CollidesWith = Pathfinding.RVO.RVOLayer.DefaultAgent;
                    _orcaAgent.Priority = 1;

                    FP elevation=0;
                    _orcaAgent.Position =CustomMath.TSVecToVec2( _baseData.position);

                    _orcaAgent.Height = 1;
                    _orcaAgent.ElevationCoordinate = elevation + 1 - CustomMath.FPHalf;
                    _orcaAgent.BufferSwitch();
#endif
                }
#endif       
            ChangeAgentType(EAgentType.none);
            // _baseData.pfm.AddAgent(this);
        }

        public void ChangeAgentType(EAgentType newType)
        {
#if NONE_FLOW_FIELD
            if (newType==EAgentType.flowFiled)
            {
                return;
            }
#endif
            if (_agentType != newType)
            {
                if (_agentType == EAgentType.flowFiled)
                {
                    FlowFieldAgent agent = _agent as FlowFieldAgent;
                    _baseData.pfm._flowManager.RemoveAgent(agent._behaviour, agent.TargetPos, _map);
                    s_ffAgent.Store(agent);
                    //if (_baseData.pfm != null)
                    //{
                    //    _baseData.pfm.RemoveAgent(this);
                    //}
                    this._agent = null;
                }
                else if (_agentType == EAgentType.astar)
                {
                    AStarAgent agent = _agent as AStarAgent;
                    //if (_baseData.pfm != null)
                    //{
                    //    _baseData.pfm.RemoveAgent(this);
                    //}
                    s_AstarAgent.Store(agent);
#if USING_ORCA || USING_RVO
                    if(_orcaAgent!=null)
                    {
                        _orcaAgent.ClearData();
                    }
#endif
                    this._agent = null;
                }
                else if(_agentType == EAgentType.ingoreObstacle)
                {
                    IgnoreObstacleAgent agent = _agent as IgnoreObstacleAgent;
                    s_ignoreObstacleAgent.Store(agent);
                    _agent = null;
                }

                GridMap map = _map;
                int nodeId = map.GetGridNodeId(_baseData.position);
                if (newType == EAgentType.flowFiled)
                {
                    ResetMoveToSpec();

                    FlowFieldAgent agent = s_ffAgent.New();
                    agent.Init(_baseData.boidsType, this);
                    this.agent = agent;

                    _activeORCA = false;
                    //FlowFieldManager.instance.AddAgent(agent);
                    //  _baseData.pfm.AddAgent(this);
                    if (_agentType == EAgentType.none)
                    {

                        {
                            map.UpdateDynamicUnwalkableNodes(_baseData.position, false,false);
                        }
                    }
                    // NotifyNeighboursRepath();
                }
                else if (newType == EAgentType.astar)
                {
                  //  ResetMoveToSpec();
                    AStarAgent agent = s_AstarAgent.New();
                    this.agent = agent;
                    agent.Init(this, colliderRadius, _map, _baseData.boidsType);
                    _activeORCA = true;
#if USING_ORCA || USING_RVO
                    if(_orcaAgent==null)
                    {
                        _orcaAgent = s_ORCAAgent.New();
                        _orcaAgent._behaviour = this;
                    }
#endif
                    // _baseData.pfm.AddAgent(this);
                    if (_agentType == EAgentType.none)
                    {
                        // if(_enabled)
                        {
                            map.UpdateDynamicUnwalkableNodes(_baseData.position, false,false);
                        }

                    }
                    // NotifyNeighboursRepath();
                }
                else if(newType==EAgentType.ingoreObstacle)
                {
                    IgnoreObstacleAgent agent = s_ignoreObstacleAgent.New();
                    this.agent = agent;
                    agent.Init(_baseData.boidsType,this);
                }
                else if (newType == EAgentType.none)
                {
                    //if (PathFindingManager.StopToSpecialPos && !PathFindingManager.c_useAvoidUnit)
                    {
                        if(_enabled)
                        {
                            map.UpdateDynamicUnwalkableNodes(_baseData.position, false, false);
                        }
                        map.UpdateDynamicUnwalkableNodes(_baseData.position, _enabled,true);
                    }
                    _preVelocity = TSVector.zero;
                    _targetDistSqr = FP.MaxValue;
                    // _targetPos = TSVector.MaxValue;
                    _newTargetPos = TSVector.MinValue;
                }
            }
            _agentType = newType;
#if UNITY_EDITOR && !MULTI_THREAD
            if (PathFindingManager.DEBUG && _agentType == EAgentType.none)
            {
                if (_agent != null)
                {
                    UnityEngine.Debug.LogError("agent null erro:" + _baseData.id);
                }
            }
#endif
        }
        public void OnDisable()
        {
            _enabled = false;
            // _activeAStar = false;
            //_activeFlowField = false;

            _NextReqRevsFrameTime = 0;
            _nextCheckActionTime = 0;
            _nextCalNeighboursTime = 0;
            _invMaxSpeed = 1;
            _hasNeighbourTargetedDefaultPos = false;
            ChangeAgentType(EAgentType.none);

            if (_map != null)
            {
                _map.UpdateDynamicUnwalkableNodes(position, false,false);
            }
#if USING_ORCA || USING_RVO
           s_ORCAAgent.Store(_orcaAgent);
           _orcaAgent = null;
#endif
            if (_group != null && AgentGroupManager.instance != null)
            {
                AgentGroupManager.instance.RemoveAgent(this);
            }
            _targetPos = TSVector.MaxValue;

            if (_baseData.pfm != null)
            {
                _baseData.pfm.RemoveAgent(this);
            }
            _agent = null;
            //  _needRepath = false;

            _baseData.Reset();
            _velocity = TSVector.zero;
            _preVelocity = TSVector.zero;

            _moveStatus = EMoveToSpecStatus.normal;
            _moveToSpecPos = TSVector.zero;
            ResetMoveToSpec();
            _dicUnReachableAgents.Clear();
            _pathBlocked = false;

            _pathManager = null;
            _hasUpdateNeighbours = false;
            _dynamicQueryCallBack = null;
            _allyNeighbours.Clear();
            _allyNeighboursDstSqr.Clear();
            _targetDistSqr = FP.Zero;

            stopMoving = false;
            _newTargetPos = TSVector.MinValue;

            _map = null;
        }
        public static void ClearAgentPool()
        {
            s_AstarAgent.Clear();
            s_ffAgent.Clear();
            s_ignoreObstacleAgent.Clear();
        }
        public void AddUnReachablePos(GridMap map, int idx)
        {
            if (_targetPos != TSVector.MaxValue && map.GetGridNodeId(_targetPos) == idx)
            {
                _dicUnReachableAgents.Add(idx);
            }
            else
            {
                _dicUnReachableAgents.Remove(idx);
            }
            if(_baseData.targetFailCallback!=null)
            {
                _baseData.targetFailCallback(idx);
            }
        }
        public int _frameOffset = 0;
        //main thread
        /// <summary>
        /// check agent type,if having targets in sight then do a star path finding,
        /// otherwise using flow field path finding to default target position
        /// </summary>
        /// <param name="frameTime"></param>
        /// <param name="newTargetPos"></param>
        public void CheckAction(FP frameTime, TSVector newTargetPos)
        {
            if (_nextCheckActionTime <= _baseData.pfm.CurTime)//agent!=null &&
            {
                bool bValidTargetPos = (newTargetPos != TSVector.MinValue && newTargetPos != TSVector.MaxValue);
                if (_baseData.eAgentType==EAgentType.ingoreObstacle )
                {
                    if (bValidTargetPos)
                    {
                        _targetPos = newTargetPos;
 
                    }
                    if (stopMoving)
                    {                                             
                        ChangeAgentType(EAgentType.none);                      
                    }
                    else
                    {
                        ChangeAgentType(EAgentType.ingoreObstacle);
                        _agent.TargetPos = _targetPos;
                        _agent._originTargetPos = _originTargetPos;
                    }
                    return;
                }
                int eCount = 0;
                FP dstSqr = 0;
           
                if ((bValidTargetPos && (newTargetPos != _defaultTargetPos || _hasNeighbourTargetedDefaultPos)) || stopMoving)
                {
                    eCount = 1;
                }
                // bool bInAttackableRange = false;
                bool doPathFinding = false;
                if (eCount > 0)
                {
                    if (bValidTargetPos)
                    {
                        dstSqr = (newTargetPos - _baseData.position).sqrMagnitude;
                    }
           
                    if (stopMoving)
                    {
                        if (_moveStatus == EMoveToSpecStatus.normal && _radiusSqr < dstSqr)//need check more frequently
                        {
                            _frameOffset = 30;
                        }
                        if (PathFindingManager.StopToSpecialPos)
                        {
                            //target change when move to spec position
                            if (_moveStatus != EMoveToSpecStatus.normal && _targetPos != TSVector.MaxValue
                                && _targetPos != newTargetPos && _targetDistSqr != dstSqr)
                            {
                                ResetMoveToSpec();//stop
                            }
                            // else if(_agentType != EAgentType.moveToSpec)//flow filed,or a star path finding
                            {
                                bool bOk = TryMoveToSpecPos(newTargetPos, stopMoving);//stopMoving
                                if (bOk)
                                {
                                    _targetPos = newTargetPos;
                                    _targetDistSqr = dstSqr;
                                }
                            }
                        }
                        else
                        {
                            if (bValidTargetPos)
                            {
                                _targetPos = newTargetPos;
                                _targetDistSqr = dstSqr;
                            }
                            ChangeAgentType(EAgentType.none);
                            // _newTargetPos = TSVector.MinValue;
                            return;
                        }
                        if (_moveStatus == EMoveToSpecStatus.normal && EAgentType.none != _agentType
                            && _map.IsSameOrNeighbourNode(newTargetPos, _baseData.position))
                        {
                            //_frameOffset = 0;
                            _targetPos = newTargetPos;
                            _targetDistSqr = (newTargetPos - position).sqrMagnitude; // curTargetDstSqr;
                            ChangeAgentType(EAgentType.none);
                            _frameOffset = 300;
                            return;
                        }
                    }
                    else
                    {
                        // prepare to change to move to spec state
                        if (_moveStatus == EMoveToSpecStatus.normal)//
                        {
                            FP dst = (maxSpeed * frameTime * 3 + _baseData.neighbourRadius);//todo
                            if (dstSqr < dst * dst && _radiusSqr < dstSqr)//need update more frequently
                            {
                                _frameOffset = 30;
                            }
                        }
                        //new target is valid and different from previous target,do a star path finding
                        int idx1 = map.GetGridNodeId(newTargetPos);
                        int idx2 = map.GetGridNodeId(_targetPos);
                        if (idx1 != idx2 && newTargetPos != TSVector.MinValue)
                        {
                            _targetPos = newTargetPos;
                            doPathFinding = true;
                            _pathBlocked = false;//need reset path blocked because of a new path should be found
                        }
                        // current closest enemy can't be attacked,try to move to attack last enenmy,this should be done outside
                        //if (_targetPos !=TSVector.MaxValue&& _targetPos != newTargetPos )
                        //{
                        //    if( _agentType != EAgentType.none)
                        //    {
                        //        TryMoveToSpecPos(_targetPos, _attackRange + colliderRadius);
                        //    }                      
                        //}
                        //else 
                        if (_moveStatus != EMoveToSpecStatus.normal)//target can't be attacked ,stop move to spec
                        {
                            ResetMoveToSpec();
                        }
                    }

                    // _activeORCA = !bInAttackableRange;//
#if UNITY_EDITOR && PATHMANAGER_DEBUG && !MULTI_THREAD
                    if (PathFindingManager.DEBUG)
                    {
                        if (_sr != null)
                        {
                            _sr.material.color = stopMoving ? Color.red : _sr.material.color;
                        }
                    }
#endif
#if USING_FLOW_FIELD
                    if (_pathBlocked)
                    {
                        ChangeAgentType(EAgentType.flowFiled);
                        if (_agent != null)
                        {
                            _agent.StartPos = _baseData.position;
                            _agent.TargetPos = _defaultTargetPos;
                            _frameOffset = 500;
                            _pathBlocked = false;
                            _targetPos = _defaultTargetPos;
                            _newTargetPos = _defaultTargetPos;
                        }
                        return;
                    }
#endif
                    if (_agentType == EAgentType.flowFiled || doPathFinding
                        || (_agentType == EAgentType.none && !stopMoving))//remove flow filed //|| _moveStatus == EMoveToSpecStatus.normal
                    {
                        _frameOffset = 30;//slow down
                        ChangeAgentType(EAgentType.astar);
                    }

                    //-------------------------------------------
                    //find target                              
                }
                else
                {
                    _frameOffset = 100;
                    TSVector targetPos = TSVector.zero;
                
                    {
                        if (PathFindingManager.StopToSpecialPos && _moveStatus != EMoveToSpecStatus.normal)
                        {
                            ResetMoveToSpec();
                        }
#if USING_FLOW_FIELD

                        if (newTargetPos == _defaultTargetPos && _targetPos != newTargetPos)
                        {
                            ChangeAgentType(EAgentType.flowFiled);
                            _targetPos = newTargetPos;
                            if (_agent != null)
                            {
                                _agent.StartPos = _baseData.position;
                                _agent.TargetPos = _defaultTargetPos;
                                _targetPos = _defaultTargetPos;
                            }
                        }
#endif
                    }
                    #if USING_FLOW_FIELD
                    if ((_agentType != EAgentType.astar ||
                        ((AStarAgent)_agent).IsNearTargetPos()) && _moveStatus == EMoveToSpecStatus.normal && _agentType != EAgentType.flowFiled)
                    {
                        ChangeAgentType(EAgentType.flowFiled);
                        if (_agent != null)
                        {
                            _agent.StartPos = _baseData.position;
                            _agent.TargetPos = _defaultTargetPos;
                            _targetPos = _defaultTargetPos;
                        }
                    }
                    if(_agentType == EAgentType.flowFiled)
                    {
                        if (bValidTargetPos && newTargetPos == _defaultTargetPos)
                        {
                            dstSqr = (newTargetPos - _baseData.position).sqrMagnitude;
                            if(dstSqr<_baseData.viewRadius*_baseData.viewRadius)//full fill agent will cause issue
                            {
                                ChangeAgentType(EAgentType.astar);
                            }
                        }
                    }
#endif
                }
                _frameOffset = _frameOffset > 0 ? _frameOffset : 100;
            }

        }
        public void ResetMoveToSpec()
        {
            if (_moveToSpecPos != TSVector.zero)
            {
                int idx = _map.GetGridNodeId(_moveToSpecPos);
                _baseData.pfm.RemoveSpecPos(idx);
            }
            _moveToSpecPos = TSVector.zero;
            _moveStatus = EMoveToSpecStatus.normal;
#if USING_RVO
            if (_activeORCA && _orcaAgent!=null)
            {
                _orcaAgent.Locked = false;
            }
#endif
        }
        public void StopMoveToSpec()
        {
            ResetMoveToSpec();
            ChangeAgentType(EAgentType.astar);
        }
        //
        public bool TryMoveToSpecPos(TSVector targetPos, bool stopMoving)//,FP limitDst
        {
            int idx = _map.GetGridNodeId(_baseData.position);
            if (PathFindingManager.StopToSpecialPos && _moveStatus == EMoveToSpecStatus.normal)
            {
                bool isWalkable = !_map.IsDynamicUnwalkableNode(idx);
                if (isWalkable && !_baseData.pfm.ContainSpecPos(idx))
                {
                    FP dstSqr = (_baseData.position - targetPos).sqrMagnitude;
                    //FP limitDstSqr = limitDst * limitDst;
                    //if (dstSqr <= limitDstSqr)//in attack range
                    if (stopMoving)
                    {
                        if (_map.TryGetSpeicalPos(out _moveToSpecPos, _baseData.position, targetPos))//, limitDstSqr
                        {
                            _moveStatus = EMoveToSpecStatus.movingToSpecPos;
                            _baseData.pfm.AddSpecPos(idx);
                            ChangeAgentType(EAgentType.none);
#if USING_RVO
                            if(_activeORCA && _orcaAgent!=null)
                            {
                                _orcaAgent.Locked = true;
                            }
#endif
                            return true;
                        }
                    }
                    else// if (_moveStatus != EMoveToSpecStatus.normal)
                    {
                        ResetMoveToSpec();
                    }
                }
                else
                {
                    if (_map.GetWorldPosition(idx) == _baseData.position)
                    {
                        _moveStatus = EMoveToSpecStatus.onSpecPos;
                        ChangeAgentType(EAgentType.none);
                    }
                }
            }

            return false;
        }
        //multi threads
        public void ResetBufferData()
        {
            _frameOffset = 10;
            if (_nextCheckActionTime > _baseData.pfm.CurTime)
            {
                return;
            }
            _aStarPathType = EPathType.none;
            _needRepath = ERepathType.none;
            _bTryToFindPath = false;
            // _revsAvailable = 0;

            _hasUpdateNeighbours = false;
        }

        //main thread
        public void CheckAvailableRevs()
        {
            if (_NextReqRevsFrameTime > _baseData.pfm.CurTime)
            {
                return;
            }

            int preRevsAvailable = 0;// _revsAvailable;
            _revsAvailable = 0;
            if (_agentType == EAgentType.astar && _agent != null)
            {
                AStarAgent aStarAgent = _agent as AStarAgent;
                _needRepath = aStarAgent.NeedRePath();

                bool isTargetNull = _targetPos == _defaultTargetPos;


                TSVector targetPos = _targetPos;
                bool bSameTarget = this.agent.TargetPos == targetPos;
                bool bNeedChangeTarget = !bSameTarget;//&& curTargetDstSqr < _targetDistSqr - CustomMath.FPHalf;
                this.agent.StartPos = _baseData.position;
                this.agent.TargetPos = targetPos;
                this.agent._originTargetPos = _originTargetPos;

                bool bDefautlToAgent = !isTargetNull && this.agent.TargetPos == _defaultTargetPos;
                bool bToDefault = isTargetNull;
                bool isFindingPath = aStarAgent.IsFindingPath();
                bool bNeedReFinding = _needRepath != ERepathType.none || !isFindingPath;
                bool bNeedFindPath = ((bNeedChangeTarget && !bToDefault) || bNeedReFinding);
                if (bNeedFindPath)//|| aStarAgent._asm==null 
                {
                    _aStarPathType = EPathType.quickPath;
                }
                if (_aStarPathType == EPathType.quickPath)
                {

                    _bTryToFindPath = (!bSameTarget || !aStarAgent.IsNearTargetPos())
                        && _needRepath == ERepathType.none && (!bDefautlToAgent) && !bToDefault;

                    if (_pathManager.RequestRevs(this.agent.StartPos, this.agent.TargetPos
                        , AStarMachine.AStarMachine.C_MaxRevolutions, preRevsAvailable))
                    {
                        _revsAvailable = AStarMachine.AStarMachine.C_MaxRevolutions;
                        aStarAgent._moveFailedCount = 0;
                        _NextReqRevsFrameTime = _baseData.pfm.CurTime + 80;
                    }
                    else
                    {
                        _preVelocity = TSVector.zero;
                    }
                }
                else
                {
                    if (aStarAgent.NeedDoPriorityPath())
                    {
                        if (_pathManager.RequestRevs(this.agent.StartPos, this.agent.TargetPos
                                                , AStarMachine.AStarMachine.MAX_REVS, preRevsAvailable))
                        {
                            _aStarPathType = EPathType.priorityPath;
                            _revsAvailable = AStarMachine.AStarMachine.MAX_REVS;
                            _NextReqRevsFrameTime = _baseData.pfm.CurTime + 80;
                        }
                        // _pathManager.DoPriorityPath(aStarAgent);

                    }
                }

            }
        }
        //multi threads
        public void FindPath()
        {
            if (_nextCheckActionTime > _baseData.pfm.CurTime || _agentType != EAgentType.astar)
            {
                return;
            }
           
            if (_aStarPathType == EPathType.quickPath )
            {
                if (_targetPos != TSVector.MinValue && _targetPos != TSVector.MaxValue)
                {
                    TSVector blockedPos = TSVector.zero;
                    pathManager._queryStack.Clear();
                    AStarAgent aa = _agent as AStarAgent;
                    bool bBlocked = false;
                    //if(aa!=null)
                    //{
                    //    if(aa.IsFindingPath())
                    //    {
                    //        return;
                    //    }
                    //}
                    if (PathFindingManager.c_useAvoidUnit)
                    {
                        bBlocked = _map.IsBlockedByObstacleBetween2Point(_baseData.position, _targetPos
                            , pathManager._queryStack, ref blockedPos);
                        if(!bBlocked)
                        {
                            aa.AddDirectNode(_baseData.position, _originTargetPos);//
                        }
                    }
                    else
                    {
                        if (aa.IsBlockedBetween2Point(_baseData.position, _targetPos
                               , pathManager._queryStack) == EBlockType.none)
                        {
                            aa.AddDirectNode(_baseData.position, _originTargetPos);//
                        }
                        else
                        {
                            bBlocked = true;
                        }
                    }
                    if (PathFindingManager.c_useAvoidUnit)
                    {
                        if (bBlocked)
                        {
                            //if ((blockedPos - _baseData.position).sqrMagnitude < GridMap.CheckFullPathDstSqr)
                            {
                                //need do priority full  path directly
                                _aStarPathType = EPathType.priorityPath;
                                if (aa._asm == null)
                                {
                                    aa.ClearPath();
                                    _pathManager.CreateAstarPath(aa, _baseData.position, _targetPos,
                                        _map, _map.GetGridNodeId(_baseData.position));
                                }
                                else
                                {

                                }
                            }
                        }
                    }
                      
               
                    if (!bBlocked)
                    {
                        return;
                    }
                }
            }
            if (_revsAvailable == 0)
            {
                return;
            }
            TSVector targetPos = _targetPos;
            int idx = _map.GetGridNodeId(targetPos);
            if (_aStarPathType == EPathType.quickPath)
            {
                EPathReturnStatus status = _pathManager.FindQuickPath(this.agent, _revsAvailable, this.agent.StartPos, this.agent.TargetPos,
                  this.map, false);//_bTryToFindPath
                if (status == EPathReturnStatus.ok)
                {
                    if (_dicUnReachableAgents.Contains(idx))
                    {
                        _dicUnReachableAgents.Remove(idx);
                    }
                }
                else if (status == EPathReturnStatus.fail && !_bTryToFindPath)
                {
                    // _dicUnReachableAgents[idx] = _target;
                    AddUnReachablePos(_map, idx);
                    this.agent.TargetPos = _targetPos;
                    this.agent._originTargetPos = _originTargetPos;
                }
            }
            else if (_aStarPathType == EPathType.priorityPath)
            {
                _pathManager.DoPriorityPath(this.agent);
            }

            // wait ,do nothing
        }
        // main thread
        public void PostFindPath()
        {
            if (_nextCheckActionTime > _baseData.pfm.CurTime)
            {
                return;
            }
            if (_agentType == EAgentType.astar && _agent != null)
            {
                AStarAgent aa = _agent as AStarAgent;
                if (aa._asm != null && aa._asm.IsWorking && !aa._asm._inQueue)
                {                   
                    _pathManager.Enqueue(aa._asm);
                }
            }
#if USING_FLOW_FIELD
            bool isTargetNull = _targetPos == _defaultTargetPos;
            if (_aStarPathType == EPathType.none && isTargetNull 
                && !_hasNeighbourTargetedDefaultPos && _agentType == EAgentType.astar)
            {
                FP dstSqr = (_defaultTargetPos - _baseData.position).sqrMagnitude;
                if (dstSqr > _baseData.viewRadius * _baseData.viewRadius)
                {
                    ChangeAgentType(EAgentType.flowFiled);
                    if (_agent != null)
                    {
                        _agent.StartPos = _baseData.position;
                        _agent.TargetPos = _defaultTargetPos;
                        _newTargetPos = _targetPos = _defaultTargetPos;
                        _frameOffset = 100;
                    }
                }                

            }
#endif
        }
        //multi thread
        public void SetDesiredPosOrVelocity(FP deltaTime, bool bSkipBoids)
        {
            //if (_nextCheckActionTime > _baseData.pfm.CurTime)
            //{
            //    return;
            //}
            if (_moveStatus == EMoveToSpecStatus.movingToSpecPos)
            {
                TSVector dir = (_moveToSpecPos - _baseData.position);
                FP desiredSpeed = TSMath.Clamp(dir.magnitude / slowdownDistance, 0, FP.One) * _baseData.maxSpeed;
                FP moveDst = desiredSpeed * deltaTime;
                if (dir.sqrMagnitude > moveDst * moveDst)
                {
                    _preVelocity = CustomMath.Normalize(dir) * desiredSpeed;// * moveDst;
                }
                else
                {
                    _preVelocity = TSVector.zero;
                }
#if USING_ORCA || USING_RVO
                if (ActiveORCA && _orcaAgent!=null)
                {
#if USING_ORCA

                    FP x = FP.Zero; 
                    FP y = FP.Zero; 
                    if(_baseData.random!=null)
                    {
                        x= _baseData.random.Next(-1, 1) / 100;
                        y= _baseData.random.Next(-1, 1) / 100;
                    }
                    _orcaAgent.prefVelocity_ = CustomMath.TSVecToVec2(_preVelocity) + new TSVector2(x,y);
                    //  _orcaAgent.Loop(CustomMath.TSVecToVec2(_preVelocity), frameTime);
                    // _preVelocity = CustomMath.TSVec2ToVec(_orcaAgent.velocity_);       
#elif USING_RVO
                    //if(_preVelocity==TSVector.zero)
                    //{
                    //    _orcaAgent.Locked = true;
                    //}
                    _orcaAgent.Position = CustomMath.TSVecToVec2(_baseData.position);
                    _orcaAgent.SetTarget(CustomMath.TSVecToVec2(_moveToSpecPos), desiredSpeed, _baseData.maxSpeed);
                    _orcaAgent.BufferSwitch();
#endif
                }
#endif
            }
            else if (_moveStatus == EMoveToSpecStatus.normal)
            {
                if (_agentType != EAgentType.none && _agent != null)
                {
                    _agent.SetDesiredPosOrVelocity(deltaTime, null, bSkipBoids);
                }
                if (_agentType == EAgentType.none)
                {
                    _preVelocity = TSVector.zero;
#if USING_RVO
                   // if (_preVelocity == TSVector.zero)
                    {
                        _orcaAgent.Locked = true;
                        //_orcaAgent.BufferSwitch();
                    }
#endif
                }
                else
                {
#if USING_RVO
                    _orcaAgent.Locked = false;
#endif
                }
#if USING_RVO
                _orcaAgent.Position = CustomMath.TSVecToVec2(_baseData.position);
                if(_orcaAgent.Locked!=_orcaAgent.locked)
                {
                    _orcaAgent.BufferSwitch();
                }
#endif
            }
        }

        //main thread
        public void FinalMoveHandle(FP frameTime, bool bSkipBoids)
        {

            TSVector prePos = _baseData.position;
            int prePosIdx = _map.GetGridNodeId(prePos);
            bool bPosChange = false;
            if (_moveStatus == EMoveToSpecStatus.movingToSpecPos)
            {

                if (_preVelocity != TSVector.zero)
                {
                    _baseData.position += _preVelocity * frameTime;
#if UNITY_EDITOR && !MULTI_THREAD
                    if (PathFindingManager.DEBUG && (_preVelocity * frameTime).magnitude > 0.08f)
                    {
                        Debug.Log("velocity speed:" + (_preVelocity * frameTime).magnitude.AsFloat());
                    }
#endif
                }
                else
                {
#if UNITY_EDITOR && !MULTI_THREAD
                    if (PathFindingManager.DEBUG && (_moveToSpecPos - _baseData.position).magnitude > 0.08f)
                    {
                        Debug.Log("End velocity speed:" + (_moveToSpecPos - _baseData.position).magnitude.AsFloat());
                    }
#endif
                    _baseData.position = _moveToSpecPos;

                    _moveStatus = EMoveToSpecStatus.onSpecPos;
                    ChangeAgentType(EAgentType.none);
                    bool isTargetNull = _targetPos == TSVector.MaxValue;
                    if (!isTargetNull)
                    {
                        _targetDistSqr = (_targetPos - _baseData.position).sqrMagnitude;
                    }
                }
                TSVector diff = _baseData.position - prePos;
                bPosChange = diff != TSVector.zero;
                if (bPosChange)
                {
                    _baseData.pfm.MoveProxy(this, diff);
                }
                //if (_baseData.callback != null)
                //{
                //    _baseData.callback(frameTime, _baseData.position, _velocity);
                //}
            }
            else if (_moveStatus == EMoveToSpecStatus.normal)
            {
                if (_agentType != EAgentType.none && _agent != null)
                {
                    agent.Loop(frameTime, null, bSkipBoids);
                    TSVector diff = _baseData.position - prePos;
                    bPosChange = diff != TSVector.zero;
                    if (bPosChange)
                    {
                        _baseData.pfm.MoveProxy(this, diff);
                    }
                }
                else
                {
                    TSVector diff = _baseData.position - prePos;
                    bPosChange = diff != TSVector.zero;
                }
                //if (_agent != null && _agentType != EAgentType.none)
                //{
                //    if (_baseData.callback != null)
                //    {
                //        _baseData.callback(frameTime, _baseData.position, _velocity);
                //    }
                //}
            }
           // if (bPosChange || _velocity != _preVelocity || PathFindingManager.DEBUG)
            {
                _velocity = _preVelocity;
                if (_baseData.loopCallback != null)
                {
                    _baseData.loopCallback(frameTime, _baseData.position, _velocity);
                }
            }
            if(!PathFindingManager.c_useAvoidUnit)
            {
                UpdateDynamicBlock(prePosIdx,_map.GetGridNodeId(_baseData.position) );
            }
           
            if (_nextCheckActionTime <= _baseData.pfm.CurTime)//agent!=null &&
            {
                _nextCheckActionTime = _baseData.pfm.CurTime + (_frameOffset > 0 ? _frameOffset : 100);
            }
        }
        public void UpdateDynamicBlock(int preIdx,int curIdx)
        {
            if (curIdx!= preIdx)
            {
                map.UpdateDynamicUnwalkableNodes(preIdx, false, false);
                map.UpdateDynamicUnwalkableNodes(curIdx, true, _agentType == EAgentType.none);
            }
        }
        public void Loop(int frameTime, bool bSkipBoids)
        {
            if (!_enabled || _pathManager == null)
            {
                return;
            }

            ResetBufferData();

            CheckAction(frameTime, _newTargetPos);//main thread
            CheckAvailableRevs();
            FindPath();
            PostFindPath();
            SetDesiredPosOrVelocity(frameTime, false);
            CalculateFinalVelocity(frameTime);
            FinalMoveHandle(frameTime, bSkipBoids);
          
        }
        public void CalculateFinalVelocity(FP frameTime)
        {
            //if (_nextCheckActionTime > _baseData.pfm.CurTime)
            //{
            //    return;
            //}
#if USING_ORCA || USING_RVO
            if (_activeORCA && _orcaAgent != null)
            {
#if USING_ORCA
                _orcaAgent.Loop(frameTime);

#elif USING_RVO
                _orcaAgent.CalculateVelocity(_pathManager.context, frameTime);//
                _orcaAgent.PostCalculation();
#endif
            }
            
#endif
            if((_agentType==EAgentType.astar ||_agentType==EAgentType.ingoreObstacle) && !stopMoving)
            {
              //  AStarAgent agent = _agent as AStarAgent;
                if (_agent != null)
                {
                    agent.CalculateFinalVelocity(frameTime, false);// 
                }
            }
          
        }
        public void MultiThreadLoop(FP deltaTime, bool bSkipBoids, EAgentLoopStep eStep)
        {
#if UNITY_EDITOR
            if (_baseData.id < 0)
            {
                return;
            }
#endif
            if (!_enabled || _pathManager == null )
            {
                return;
            }
            //if ((eStep > EAgentLoopStep.checkAction && stopMoving))
            //{
            //    return;
            //}
            switch (eStep)
            {
                //case EAgentLoopStep.resetBuffData:
                //    ResetBufferData();//multi threads
                //    break;
                case EAgentLoopStep.checkAction:
                    ResetBufferData();
                    CheckAction(deltaTime, _newTargetPos);//main thread
                    CheckAvailableRevs();//main thread
                    FindPath();//multi threads,
                    break;
                //case EAgentLoopStep.checkAvailabelRevs:
                //    CheckAvailableRevs();//main thread
                //    break;
                case EAgentLoopStep.findPath:
                    //......................todo
                    PostFindPath();//main threads ???
                    SetDesiredPosOrVelocity(deltaTime, false);//multi threads
                    break;
                //case EAgentLoopStep.postFindPath:
                //    PostFindPath();//main threads
                //    break;
                //case EAgentLoopStep.setDesiredPos:
                //    SetDesiredPosOrVelocity(deltaTime, false);//multi threads
                //    break;
                case EAgentLoopStep.CalculateVelocity:
                    CalculateFinalVelocity(deltaTime);//multi threads
                    break;
                case EAgentLoopStep.FinalMoveHandle:
                    FinalMoveHandle(deltaTime, bSkipBoids);//main thread
                    break;
            }
        }
  		public TSVector GetRealPosition()
        {
            if(_map==null)
            {
                return TSVector.zero;
            }
            return ( position + _map._startPos);//CustomMath.TsVecToVector3
        }
        public void Reset()
        {
            OnDisable();
        }
      
    }
}
