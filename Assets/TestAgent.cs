using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PathFinding;
using TrueSync;
public class TestAgent : MonoBehaviour {
   // [SerializeField]
    public PathFindingAgentBehaviour unit;
    public FP attackRange;

    // Use this for initialization
    DynamicTreeQueryCallback _dynamicQueryCallBack;
    internal List<IAgentBehaviour> _listEnemyAgents = new List<IAgentBehaviour>();
    internal List<FP> _listEnemyAgentsDstSqr = new List<FP>();
    bool _hasUpdateNeighbours = false;

    public float _attackRange;
    public EAgentType _agentType;
    public Vector3 _velocity;
    public Vector3 _preVelocity;
    public int _offsetTime;

    public bool debugDrawPath = false;
    public bool Static = false;
    public int _pathNodeIdx = -1;
    public int _totalPathNodeCount = 0;
    public int _id;
    public Vector3 targetPos;

    public int _targetId = -1;
    public TestPathFinding _testPathFinding = null;

    public void CalculateEnemies()
    {
       // if (!_hasUpdateNeighbours)
        {
            //
             _listEnemyAgents.Clear();
             _listEnemyAgentsDstSqr.Clear();
        
            unit._baseData.pfm.CalculateAgentNeighbours(this.unit, _dynamicQueryCallBack,unit._baseData.viewRadius);
            _hasUpdateNeighbours = true;
        }
    }
   public void FailFindPathCallback(int idx)
    {
        _targetId = 0;
    }
    bool DynamicTreeQueryCallback(int id)
    {
        PathFindingAgentBehaviour agent = unit._baseData.pfm.GetAgentByProxyId(id);
        bool bOver = true;
        if (agent != null)
        {
            if (id == unit.proxyId)
            {
                return true;
            }

            if(_listEnemyAgents.Count< PathFindingAgentBehaviour.maxEnemyNeighbours)
            {
                bOver = false;
            }
            if(agent.playerId != unit.playerId)
            {
               // _listEnemyAgents.Add(agent);
                FP dstSqr = (agent.position - unit._baseData.position).sqrMagnitude;
                FP rDst = TSMath.Max(unit._baseData.colliderRadius + unit._baseData.viewRadius,attackRange+1);
                if (dstSqr> rDst*rDst)
                {
                    return true;
                }
                // _listEnemyAgentsDstSqr.Add(dstSqr);
                List<FP> neighboursDestSqr = _listEnemyAgentsDstSqr;
                List<IAgentBehaviour> nbs = _listEnemyAgents;
                //sort
                if (_listEnemyAgents.Count < PathFindingAgentBehaviour.maxEnemyNeighbours)
                    {
                        //neighbours.Add(agent);
                        //neighboursDestSqr.Add(distSqr);
                        nbs.Add(null);
                        neighboursDestSqr.Add(FP.MaxValue);
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
#if UNITY_EDITOR && !MULTI_THREAD
                        if(PathFindingManager.DEBUG)
                        {
                            if (i > 0)
                            {
                                if (nbs[i] == nbs[i - 1])
                                {
                                    UnityEngine.Debug.LogError("DynamicTree error!");
                                }
                            }
                        }                     
#endif
                    neighboursDestSqr[i] = dstSqr;
                    }
                //
            }
        }
        if (bOver)//overflow
        {
            return false;
        }
        return true;
    }
    void Start () {
        _dynamicQueryCallBack = this.DynamicTreeQueryCallback;

    }
    //multi threads
#if SEARCH_ENEMY
        public void FindTarget()
        {
            if (_nextCheckActionTime > _baseData.pfm.CurTime)
            {
                return;
            }
            int eCount = _listEnemyAgents.Count;
            //enemy in sight,either move to spec or AStar path finding
            if (eCount > 0 && _agentType == EAgentType.astar && !_pathBlocked && _agent != null)
            {
                AStarAgent aStarAgent = _agent as AStarAgent;
                _needRepath = aStarAgent.NeedRePath();
                bool isFindingPath = aStarAgent.IsFindingPath();

                for (int i = 0; i < eCount; i++)
                {
                    IAgentBehaviour enemyAgent = _listEnemyAgents[i];//closest enemy
                    int idx = _map.GetGridNodeId(enemyAgent.position);
                    FP curTargetDstSqr = (enemyAgent.position - position).sqrMagnitude;
                    TSVector targetPos = enemyAgent.position;
                    _targetPos = enemyAgent.position;
                    if (_dicUnReachableAgents.Contains(idx) && _needRepath!=ERepathType.hasOpen)//
                    {
                        if (eCount - 1 == i)//can't reach enemy agent,then walk to default target pos
                        {
                            targetPos = _defaultTargetPos;
                            enemyAgent = null;
                            _targetPos = _defaultTargetPos;
                            idx = _map.GetGridNodeId(_defaultTargetPos);
                            if (_dicUnReachableAgents.Contains(idx))
                            {
                                _targetDistSqr = (_defaultTargetPos - _baseData.position).sqrMagnitude;
                                //_nextCheckActionTime = _baseData.pfm.CurTime + 1000;
                                // _dicUnReachableAgents.Clear();//
                                return;
                            }
                            curTargetDstSqr = (_defaultTargetPos - _baseData.position).sqrMagnitude;
                        }
                        else
                        {
                            continue;
                        }
                    }

                    bool bSameTarget = this.agent.TargetPos == targetPos;
                    bool bNeedChangeTarget = !bSameTarget && curTargetDstSqr < _targetDistSqr - CustomMath.FPHalf;

                    bool bToDefault = _targetPos == _defaultTargetPos;

                    bool bNeedReFinding = _needRepath!=ERepathType.none || !isFindingPath;
                    bool canFind = _pathManager.CanFindPathNow();
                    if (!canFind)
                    {
                        _frameOffset = 30;
                    }
                    bool bNeedFindPath = ((bNeedChangeTarget && !bToDefault) || bNeedReFinding)
                        && canFind;
                    if (bNeedFindPath)//|| aStarAgent._asm==null 
                    {
                        _aStarPathType = EAStarPathType.quickPath;
                        _targetDistSqr = curTargetDstSqr;

#if UNITY_EDITOR
                        if (PathFindingManager.DEBUG)
                        {
                            if (_needRepath!=ERepathType.none)
                            {
                                UnityEngine.Debug.Log("needrepath findpath");
                            }
                            if (!isFindingPath)
                            {
                                UnityEngine.Debug.Log("not FindingPath findpath");
                            }
                            if (bNeedChangeTarget && !bToDefault)
                            {
                                UnityEngine.Debug.Log("not bSameTarget findpath");
                            }
                        }
#endif
                        break;
                    }
                    else if (bSameTarget)// && aStarAgent._asm != null same target and still finding the path,should't change target
                    {
                        break;
                    }

                }
                _frameOffset = _frameOffset > 0 ? _frameOffset : 300;
                return;
            }
            if(eCount==0 )
            {
                if (_agentType == EAgentType.astar && _agent != null)
                {
                    AStarAgent aStarAgent = _agent as AStarAgent;
                    if (aStarAgent.IsNearTargetPos())
                    {
                        _needRepath = aStarAgent.NeedRePath();
                        bool isFindingPath = aStarAgent.IsFindingPath();
                        bool bNeedReFinding = _needRepath!=ERepathType.none || !isFindingPath;
                        if (bNeedReFinding)//|
                        {
                            //_needRepath = true;
                            _aStarPathType = EAStarPathType.quickPath;
                      
                            this.agent.StartPos = _baseData.position;
                            this.agent.TargetPos = _defaultTargetPos;

                            //TSVector pathEndPos = _pathManager.FindQuickPath(this.agent, this.agent.StartPos, this.agent.TargetPos, _map, false);//
                            _targetPos = _defaultTargetPos;
                        }
                    }                   
                }
            }
        }
#endif
    private void OnEnable()
    {
        if(unit==null && Static)
        {
            AgentBaseData data = new AgentBaseData();
            data.id = -1;
            data.mapId = 0;
            data.playerId = 0;
            data.eAgentType = EAgentType.none;
            data.defaultTargetPos = CustomMath.Vector3ToTsvec(this.transform.position);
            data.loopCallback = this.Loop;
            data.boidsType = (byte)EBoidsActiveType.all;
            data.maxSpeed = FP.One * 3;
            data.position = CustomMath.Vector3ToTsvec(this.transform.position);
            data.collidesWithLayer = 1;
            data.viewRadius = FP.One * 6;
            data.neighbourRadius = FP.One * data.maxSpeed;
            data.neighbourRadiusSqr = data.neighbourRadius * data.neighbourRadius;
            data.colliderRadius = 0.5f;//test
         
            data.pfm = TestPathFinding._pfm;
            unit = new PathFindingAgentBehaviour();
            unit.enabled = true;
            unit.Init(data);
        }

        if(unit!=null)
        {
            unit.OnEnable();
        }
    }
    private void OnDisable()
    {
        if (unit != null)
        {
            unit.OnDisable();
        }
    }

    public void Loop()
    {

        if (!enabled|| !gameObject.activeSelf)
        {
            return;
        }
        if (!unit.enabled)
        {
            unit.OnEnable();
            unit.ChangeAgentType( unit._baseData.eAgentType==EAgentType.ingoreObstacle? EAgentType.ingoreObstacle : EAgentType.astar);
            unit.agent.TargetPos = unit._defaultTargetPos;
        }
        CalculateEnemies();
        TestAgent agent = null;
        FP tagetDstSqr = FP.MaxValue;
        if (_targetId>=0)
        {
            agent = _testPathFinding.GetAgent(_targetId);
            if(agent!=null)
            {
                tagetDstSqr = (agent.unit.position - unit.position).sqrMagnitude;
            }
        }
       
        for (int i=0;i<_listEnemyAgents.Count;i++)
        {
            FP trueRange = attackRange + _listEnemyAgents[i].colliderRadius;
            FP dstSqr= (_listEnemyAgents[i].position - unit.position).sqrMagnitude;
        
            if (dstSqr < trueRange * trueRange )//in attackage range
            {             
                if(unit.AgentType != EAgentType.none)
                {
                    unit.SetNewTargetPos(_listEnemyAgents[i].position);//////////////////////////
                    unit.stopMoving = true;
                    unit._nextCheckActionTime = 0;
                    unit._frameOffset = 20;
                    _targetId = _listEnemyAgents[i].baseData.id;
                }                         
                break;
            }
            if (_targetId >= 0 && dstSqr >= tagetDstSqr - GridMap.GetNodeSize() * GridMap.GetNodeSize()
                && _targetId!=unit.Id)
            {
                if(agent!=null &&unit.CanSetTargetPos(agent.unit.position))
                {
                    break;
                }           
            }
            if (unit.SetNewTargetPos(_listEnemyAgents[i].position)  )
            {
                int idx1 = unit.map.GetGridNodeId(unit.NewTargetPos);
                int idx2= unit.map.GetGridNodeId(unit.targetPos);
                if (idx1 != idx2)
                {
                    unit.stopMoving = false;
                }
                _targetId = _listEnemyAgents[i].baseData.id;
                break;
            }            
        }
        if(_listEnemyAgents.Count==0||_targetId==0)
        {          
           
            TSVector targetPos = CustomMath.Vector3ToTsvec(_testPathFinding.destObj[unit.playerId].transform.position);
            // unit.ChangeAgentType(EAgentType.flowFiled);
            if((targetPos - unit.position).sqrMagnitude > GridMap._checkDeltaDstSqr)
            {
                unit.stopMoving = false;
            }
            if (unit.NewTargetPos == TSVector.MinValue || unit.NewTargetPos == TSVector.MaxValue || _targetId == 0)
                // || (unit.NewTargetPos - unit.position).sqrMagnitude<unit.neighbourRadius*unit.neighbourRadius*225/100)
            {
                unit.SetNewTargetPos(targetPos);
            }
           
          //unit.agent.TargetPos = unit._defaultTargetPos;
        }
        if(unit.AgentType==EAgentType.astar && unit.agent!=null)
        {
            AStarAgent ag = unit.agent as AStarAgent;
            if(debugDrawPath)
            {
                for (int i = 1; i < ag._vecRunningPath.Count; i++)
                {
                    UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(ag._vecRunningPath[i - 1]), CustomMath.TsVecToVector3(ag._vecRunningPath[i]), UnityEngine.Color.green, 1);
                }              
            }
#if UNITY_EDITOR
            ag._showDebug = debugDrawPath;
#endif
            _pathNodeIdx = ag.TowardPathIdx;
            _totalPathNodeCount = ag._vecRunningPath.Count;
        }
    }
    float fTime = 0;
    public void Loop(FP frameTime, TSVector position, TSVector velocity)
    {
        if (!enabled || !gameObject.activeSelf)
        {
            return;
        }
        if (unit!=null)
        {
           // attacRange = unit._attackRange.AsFloat();
            _agentType = unit._agentType;
            _velocity = CustomMath.TsVecToVector3(unit.velocity);
            _preVelocity= CustomMath.TsVecToVector3(unit.preVelocity);
            _offsetTime = unit._frameOffset;
            _id = unit.Id;
            targetPos = CustomMath.TsVecToVector3(unit.targetPos);
        }
      //  Debug.Log(unit.Id+ ",speed:"+velocity.magnitude);
        FP trueAtkRange = (attackRange + unit.colliderRadius);//warning,_listEnemyAgents[0].colliderRadius
        FP atkRangeSqr = trueAtkRange * trueAtkRange;
        if(unit.targetPos!=TSVector.MaxValue)
        {
            FP dstSqr = (unit.targetPos - unit.position).sqrMagnitude;
            if(unit.AgentType!=EAgentType.none)
            {
                unit.stopMoving = dstSqr <= atkRangeSqr;
            }          
        }       

        gameObject.transform.position = CustomMath.TsVecToVector3(position);
        if(Time.realtimeSinceStartup- fTime>Time.deltaTime*2 && fTime>0)
        {
           // Debug.LogError("diff time:"+ (Time.realtimeSinceStartup - fTime) );           
        }
        fTime = Time.realtimeSinceStartup;
        if ( velocity.magnitude / frameTime > 0.01f)//todo
        {
            //transform.forward = CustomMath.TsVecToVector3(velocity);//todo
            var rot = transform.rotation;
            var targetRot = Quaternion.LookRotation(CustomMath.TsVecToVector3(velocity), transform.up);
            const float RotationSpeed = 5;
  
            transform.rotation = Quaternion.Slerp(rot, targetRot, frameTime.AsFloat() * RotationSpeed);
        }
    }
}
