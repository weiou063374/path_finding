///author:huwei
///date:2018.4.17
#define USING_DYNAMIC_TREE
//#define USING_ORCA
#define FORCE_BASED
using System;

using System.Collections.Generic;

using TrueSync;
namespace PathFinding
{
    public enum EBoidsActiveType
    {
        none = 0,
        seperation = 1,
        alignment = 1 << 1,
        cohesion = 1 << 2,
        terrainSeperation = 1 << 3,
        avoidUnit = 1 << 4,
        collisionAvoidance = 1 << 5,
        all = (1 << 6) - 1
    }
    public enum ENewPosStatus
    {
        ok,
        adjust,
        failed,
    }
    public enum ECheckDirStatus
    {
        ok,
        delay,
        fail
    }
#if USING_DYNAMIC_TREE
    public interface IAgentBehaviour : DynamicTreeData
#else
    public interface IAgentBehaviour : QTData
#endif
    {
        void Loop(int frameTime, bool bSkipBoids);
        System.Int64 playerId { get; set; }
        AgentBase agent { get; set; }
#if USING_ORCA || USING_RVO
#if USING_ORCA
        ORCAAgent ACAgent { get; set; }
#elif USING_RVO
        RVOAgent  ACAgent { get; set; }
#endif
        bool ActiveORCA { get; set; }
#endif
        List<IAgentBehaviour> neighbours { get; set; }
        FP maxSpeed { get; set; }
        FP invMaxSpeed { get; set; }
        FP colliderRadius { get; set; }
        AgentGroup group { get; set; }
        GridMap map { get; set; }
#if FORCE_BASED
        List<LineObstacle> neighbourObstacles { get; }
#endif
        void AddUnReachablePos(GridMap map, int idx);
        bool enabled { get; set; }
        TSVector velocity { get; set; }
        TSVector preVelocity { get; set; }
        bool pathBlocked { get; set; }
        PathManager pathManager { get; set; }
        AgentBaseData baseData { get; }
    }


    public class AgentBase : IResetable, AStarMachine.IAgent
    {
        public static readonly FP S_terrainSepFactor = FP.One / 3;
        public static readonly FP S_seperationFactor = FP.One;
        public static readonly FP S_alignmentFactor = FP.One / 2;
        public static readonly FP S_cohesionFactor = FP.One / 20;
        public static readonly FP S_BoidsFactor = FP.One;
        internal IAgentBehaviour _behaviour = null;
        public int _activeBoids = (int)EBoidsActiveType.none;

        public const int Max_Vel_Factor = 5;
        protected FP _currentVelFactor = 5;
        protected TSVector _preTestOKVec = TSVector.zero;

        protected TSVector _preVelDirection = TSVector.zero;
       // public int current
        public bool IsBoisTypeActive(EBoidsActiveType type)
        {
            if (_activeBoids == (int)EBoidsActiveType.all)
            {
                return true;
            }
            return (((int)type) & _activeBoids) > 0;
        }
        public int GetBoisTypeExcept(EBoidsActiveType type)
        {
            return EBoidsActiveType.all - type;
        }
        protected TSVector _startPos;
        protected TSVector _targetPos;
        public TSVector _originTargetPos;
        protected int _startPosId;
        //  protected FP _speed;
        internal int _moveFailedCount = 0;
        internal SearchFinished searchFinished = null;
        protected TSVector _preBoidsForce = TSVector.zero;
        //public FP maxSpeed = FP.One*3;

        public TSVector StartPos { set { _startPos = value; _startPosId = _behaviour.map.GetGridNodeId(_startPos); } get { return _startPos; } }
        public virtual TSVector TargetPos { set { _targetPos = value; } get { return _targetPos; } }
        //  public FP speed { set { _speed = value; } get { return _speed; } }
        
        public AgentBase()
        {

        }
        public void Init(IAgentBehaviour behaviour)
        {

        }
        internal AgentBase(IAgentBehaviour go)
        {

        }
        public virtual void Reset()
        {
            _moveFailedCount = 0;
            _startPos = TSVector.zero;
            _targetPos = TSVector.MinValue;
            _behaviour = null;
            _startPosId = -1;
            _preBoidsForce = TSVector.zero;
        }

        public virtual void OnEnable()
        {
            Reset();
        }
        public virtual void OnDisable()
        {
            //if(PathFindingManager.Instance)
            //{
            //    PathFindingManager.instance.RemoveAgent(this);
            //}
        }
        internal const int C_maxMoveFailedCount = 10;
        public virtual void SetDesiredPosOrVelocity(FP frameTime, IsPosWalkable w = null, bool bSkipBoids = false)
        {

        }
        public virtual void CalculateFinalVelocity(FP deltaTime, bool bSkipBoids = false)
        {

        }
        public virtual void Loop(FP frameTime, IsPosWalkable isWalkable = null, bool bSkipBoids = false)
        {
            if (_behaviour.agent == null)
            {
                return;
            }
            _behaviour.velocity = _behaviour.preVelocity;
#if UNITY_EDITOR
            if (PathFindingManager.DEBUG)
            {
                if ((_behaviour as PathFindingAgentBehaviour)._moveStatus != EMoveToSpecStatus.normal)
                {
                    UnityEngine.Debug.LogError(" moveStatus loop error:" + (_behaviour as PathFindingAgentBehaviour).Id);
                }
            }
#endif
            FP fDtSecond = frameTime;// PathFindingManager.instance.DeltaTime * CustomMath.MsToSecond;
            TSVector newPos = _behaviour.position + _behaviour.velocity * fDtSecond;
            ENewPosStatus status = isWalkable==null?ENewPosStatus.ok:CheckNewPos(_behaviour.position, ref newPos, isWalkable);
            if (ENewPosStatus.ok != status)
            {
                if (_moveFailedCount < C_maxMoveFailedCount)
                {
                    _moveFailedCount++;
                }
            }
            if (ENewPosStatus.failed == status)
            {
                return;
            }
            _behaviour.position = newPos;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="forceToApply">basic force</param>
        /// <param name="basicVelocity">basic Velocity</param>
        /// <returns></returns>
        protected TSVector ApplyForce(FP deltaTime, TSVector desiredDirection, TSVector basicVelocity, TSVector pos, List<IAgentBehaviour> agents, bool bUsePreForce, bool bSkipStatic)//
        {
            // isTminStaticAgent = false;
            int count = agents.Count;
            TSVector boidsVelocity = TSVector.zero;
            //TSVector forceToApply = TSVector.zero;
            TSVector forceToApply = TSVector.zero;
            TSVector avoidObstacle = TSVector.zero;
            FP mAvoidObstacle = FP.Zero;
            if (IsBoisTypeActive(EBoidsActiveType.terrainSeperation) && false)
            {
                avoidObstacle = Boids.BoidsBehaviourAvoidObstacle(pos, _behaviour.map, basicVelocity);
                //desiredDirection = (desiredDirection + dir * S_terrainSepFactor);
                //if(desiredDirection!=TSVector.zero)
                //{
                //    desiredDirection = desiredDirection.normalized;
                //}   
                avoidObstacle = CustomMath.Normalize(avoidObstacle, out mAvoidObstacle);
            }
            if (desiredDirection != TSVector.zero)
            {
                desiredDirection = CustomMath.Normalize(desiredDirection);
                forceToApply += Boids.SteerTowards(_behaviour, desiredDirection, basicVelocity);
            }
            if (mAvoidObstacle > FP.Zero)
            {
                forceToApply += Boids.SteerTowards(_behaviour, avoidObstacle, basicVelocity);
            }
            if (PathFindingManager.c_useAvoidUnit)
            {
                if (IsBoisTypeActive(EBoidsActiveType.avoidUnit))
                {
                    TSVector avoidVector = SteerForUnitAvoidance.GetDesiredSteering(_behaviour);
                    TSVector force = TSVector.ClampMagnitude(avoidVector / deltaTime * _behaviour.baseData.invMass, _behaviour.baseData.maxForce);
                    forceToApply += force;
                }
                //else if (IsBoisTypeActive(EBoidsActiveType.collisionAvoidance))
                //{
                // //   bool isTminStaticAgent = false;
                //    forceToApply += CustomMath.TSVec2ToVec(ForceBasedAgent.computeForces(_behaviour, basicVelocity,out isTminStaticAgent));
                //}

            }

            if (_activeBoids > 0 && !bUsePreForce &&
                (IsBoisTypeActive(EBoidsActiveType.seperation) || IsBoisTypeActive(EBoidsActiveType.cohesion)
                || IsBoisTypeActive(EBoidsActiveType.alignment)))
            {
                // int count =(i%3== _updateStart)? this.neighbours.Count:0;//_group._thiss
                TSVector totalForce = TSVector.zero;
                TSVector averageHeading = TSVector.zero;
                TSVector centerOfMass = TSVector.zero;
                int neighboursCountSep = 0;
                int neighboursCountAlig = 0;
                int neighboursCountCoh = 0;
                FP radius = _behaviour.colliderRadius * 4;
                FP sepSqr = radius * radius;// 
                FP nrSqr = _behaviour.baseData.neighbourRadiusSqr * FP.EN2 * 25;
                for (int j = 0; j < count; j++)
                {
                    IAgentBehaviour a = agents[j];// _behaviour.neighbours
                    if (IsBoisTypeActive(EBoidsActiveType.seperation))
                    {
                        Boids.BoidsBehaviourSeparation(_behaviour, a, sepSqr, ref totalForce, ref neighboursCountSep, bSkipStatic);
                    }
                    if (IsBoisTypeActive(EBoidsActiveType.alignment))
                    {
                        Boids.BoidsBehaviourAlignment(_behaviour, a, nrSqr, ref averageHeading, ref neighboursCountAlig);
                    }
                    if (IsBoisTypeActive(EBoidsActiveType.cohesion))
                    {
                        Boids.BoidsBehaviourCohesion(_behaviour, a, nrSqr, ref centerOfMass, ref neighboursCountCoh);
                    }
                    if (a.enabled && a.agent == null)//the ally neighbour has a target,
                                                     //cause this agent change type to atar type targeted to the same target
                    {
                        PathFindingAgentBehaviour agent = _behaviour as PathFindingAgentBehaviour;
                        if (agent.AgentType == EAgentType.flowFiled)
                        {
                            TSVector target = (a as PathFindingAgentBehaviour).targetPos;
                            bool isDefaultTaget = target == a.baseData.defaultTargetPos;
                            if (target != TSVector.MaxValue && target != TSVector.MinValue && !isDefaultTaget)
                            {
                                agent.stopMoving = false;
                                agent._hasNeighbourTargetedDefaultPos = true;
                                target.y =0;
                                agent.SetNewTargetPos(target, true);
                            }
                            if(!isDefaultTaget)
                            {
                                agent.preVelocity = TSVector.zero;
                                forceToApply = TSVector.zero;
                                // agent.velocity = TSVector.zero;
                                // agent.ChangeAgentType(EAgentType.astar);
                                return forceToApply;
                            }                        
                        }
                    }
                }

                if (count > 0)
                {
                    TSVector sep = TSVector.zero;
                    if (totalForce != TSVector.zero)
                    {
                        //totalForce.Multiply(agent.maxForce / neighboursCount)
                        sep = totalForce * (_behaviour.baseData.maxForce) / neighboursCountSep;
                        //sep=Boids.SteerTowards(_behaviour, sep, basicVelocity);
                        //
                        FP lenSqr = sep.sqrMagnitude;
                        if (lenSqr > _behaviour.baseData.maxForceSqr)
                        {
                            FP fval = _behaviour.baseData.maxForce / TSMath.Sqrt(lenSqr);
                            sep = sep * fval;
                        }
                    }
                    TSVector avh = TSVector.zero;
                    if (averageHeading != TSVector.zero) //average heading
                    {
                        averageHeading = averageHeading / (neighboursCountAlig);
                        avh = Boids.SteerTowards(_behaviour, CustomMath.Normalize(averageHeading), basicVelocity);
                    }
                    //average position 
                    TSVector coh = TSVector.zero;
                    if (centerOfMass != TSVector.zero)// seek that position
                    {
                        centerOfMass = centerOfMass + _behaviour.position;
                        neighboursCountCoh++;
                        centerOfMass = centerOfMass / neighboursCountCoh;
                        coh = Boids.BoidsBehaviourSeek(_behaviour, basicVelocity, centerOfMass);
                    }
                    _preBoidsForce = sep * S_seperationFactor + avh * S_alignmentFactor + coh * S_cohesionFactor;
                    forceToApply += _preBoidsForce;
                    if (PathFindingManager.DEBUG)
                    {
#if UNITY_5_5_OR_NEWER && !MULTI_THREAD
                        if (FP.Abs(forceToApply.x) > GridMap.SCALE * 1000 || FP.Abs(forceToApply.z) > GridMap.SCALE * 1000)
                        {
                            UnityEngine.Debug.LogError("forceToApply error!");
                        }
#endif
                    }
                }
            }
            else if (bUsePreForce)
            {
                forceToApply += _preBoidsForce;
            }

            if (forceToApply != TSVector.zero)
            {
                if(TSMath.Abs(forceToApply.x)> _behaviour.baseData.maxForce
                    || TSMath.Abs(forceToApply.z) > _behaviour.baseData.maxForce)
                //FP lengthSquared = forceToApply.sqrMagnitude;
                //if (lengthSquared > _behaviour.baseData.maxForceSqr)//&& _activeBoids!=(byte)EBoidsActiveType.seperation)
                {
                    forceToApply = forceToApply * FP.EN3;
                    forceToApply = _behaviour.baseData.maxForce * forceToApply.normalized;
                } 
                return forceToApply * _behaviour.baseData.invMass;//
            }
            return forceToApply;
        }
        /// <summary>
        /// 
        /// </summary>
        /// <param name="forceToApply">basic force</param>
        /// <param name="basicVelocity">basic Velocity</param>
        /// <returns></returns>
        protected TSVector ApplySeperateForce(TSVector toAcc, List<IAgentBehaviour> agents,bool bSkipStatic)//,out bool isTminStaticAgent
        {
            int count = agents.Count;
            TSVector boidsVelocity = TSVector.zero;
            TSVector forceToApply = TSVector.zero;           
            {                
                TSVector totalForce = TSVector.zero;               
                int neighboursCountSep = 0;             
                FP radius = _behaviour.colliderRadius * 4;
                FP sepSqr = radius * radius;// 
                FP nrSqr = _behaviour.baseData.neighbourRadiusSqr * FP.EN2 * 25;
                for (int j = 0; j < count; j++)
                {
                    IAgentBehaviour a = agents[j];// _behaviour.neighbours
                    Boids.BoidsBehaviourSeparation(_behaviour, a, sepSqr, ref totalForce, ref neighboursCountSep, bSkipStatic);

                }

                if (count > 0)
                {
                    TSVector sep = totalForce * (_behaviour.baseData.maxForce) / count;
                   
                    FP lenSqr = sep.sqrMagnitude;
                    if (lenSqr > _behaviour.baseData.maxForceSqr)
                    {
                        FP fval = _behaviour.baseData.maxForce / TSMath.Sqrt(lenSqr);
                        sep = sep * fval;
                    }
                    forceToApply =sep;
                    if (PathFindingManager.DEBUG)
                    {
#if UNITY_5_5_OR_NEWER && !MULTI_THREAD
                        if (FP.Abs(forceToApply.x) > GridMap.SCALE * 1000 || FP.Abs(forceToApply.z) > GridMap.SCALE * 1000)
                        {
                            UnityEngine.Debug.LogError("forceToApply error!");
                        }
#endif
                    }
                }
            }          

            if (forceToApply != TSVector.zero)
            {
                FP max = TSMath.Max(TSMath.Abs(forceToApply.x), TSMath.Abs(forceToApply.z));
                if (max > _behaviour.baseData.maxForce*FP.EN1*7)
                {
                    forceToApply = forceToApply / max;
                    forceToApply = _behaviour.baseData.maxForce * forceToApply.normalized * FP.EN1 * 6;
                }
                return (forceToApply+toAcc) * _behaviour.baseData.invMass;//
            }
            return forceToApply+toAcc;
        }
        internal ENewPosStatus CheckNewPos(TSVector position, ref TSVector newPos, IsPosWalkable isWalkable)
        {
            //TSVector newPos = position + velocity * fDtSecond;
            ENewPosStatus status = ENewPosStatus.ok;
            if (PathFindingManager.DEBUG)
            {
#if UNITY_5_5_OR_NEWER && !MULTI_THREAD
                if (FP.Abs(_behaviour.velocity.x) >_behaviour.maxSpeed*2 || FP.Abs(_behaviour.velocity.z) > _behaviour.maxSpeed * 2)
                {
                    UnityEngine.Debug.LogError("velocity error!");
                }
#endif
            }
            if (!isWalkable(newPos))
            {
                TSVector newPosX = newPos;
                newPosX.x = position.x;
                TSVector newPosZ = newPos;
                newPosZ.z = position.z;
                status = ENewPosStatus.adjust;
                if (isWalkable(newPosX))
                {
                    newPos = newPosX;
                    _behaviour.velocity = CustomMath.Normalize(newPos - _behaviour.position) * _behaviour.preVelocity.magnitude;// _behaviour.maxSpeed;
                }
                else if (isWalkable(newPosZ))
                {
                    newPos = newPosZ;
                    _behaviour.velocity = CustomMath.Normalize(newPos - _behaviour.position) * _behaviour.preVelocity.magnitude;// * _behaviour.maxSpeed;
                }
                else
                {
                    _behaviour.velocity = TSVector.zero;
                    return ENewPosStatus.failed;
                }
            }

            return status;
        }
        protected TSVector DesiredAcc(TSVector goal, out TSVector prefV, out FP factor, out FP time)
        {
            time = FP.Zero;
            prefV = TSVector.zero;
            TSVector vPref = goal - _behaviour.position;
            FP mag;
            factor = FP.One;
            vPref = CustomMath.Normalize(vPref, out mag) * _behaviour.maxSpeed;
            if (mag * mag < GridMap._checkDeltaDstSqr)
            {
                return TSVector.zero;
            }
            time = mag / _behaviour.maxSpeed;
            //driving force
            prefV = vPref;

            if (_behaviour.velocity == TSVector.zero)
            {
                _behaviour.preVelocity = prefV;
            }
            else
            {
                _behaviour.preVelocity = _behaviour.velocity;
            }
            if (_preVelDirection == TSVector.zero)
            {
                _preVelDirection = prefV * _behaviour.invMaxSpeed;
            }
            factor = _behaviour.baseData.maxForce * _behaviour.invMaxSpeed;
            TSVector desiredF = (vPref - _behaviour.preVelocity) * factor;///_ksi
            return desiredF * _behaviour.baseData.invMass;
        }
        public ECheckDirStatus CheckDir(int dirIdx, TSVector dNormlDir, TSVector perpendicular1,
           TSVector perpendicular2, TSVector test45Dir1, ref FP maxDot, ref TSVector minAngleTestDir,bool bIgnoreObstacle)
        {
            FP dirDst = GridMap.blockDirDst;
            TSVector testDir = TSVector.zero;
            bool bExtraCheck = true;
            ECheckDirStatus status = ECheckDirStatus.fail;
            switch (dirIdx)
            {
                case 0://45 from disired direction
                    testDir = test45Dir1;    //45 to disired direction                 
                    break;
                case 1://45 to disired direction
                    TSVector test45Dir2 = CustomMath.perpendicular(test45Dir1);
                    testDir = test45Dir2 * dNormlDir > 0 ? test45Dir2 : (TSVector.zero - test45Dir2);
                    break;
                case 2: //perpendicular direction 1
                    testDir = perpendicular1;
                    //---
                    //   |      handle this situation  ,>5 degree from previous test ok direction
                    //---
                    bExtraCheck = _preTestOKVec * perpendicular2 < 996 * FP.EN3;
                    break;
                case 3:  //perpendicular direction 2
                    testDir = perpendicular2;
                    bExtraCheck = _preTestOKVec * perpendicular1 < 996 * FP.EN3;
                    break;
                case 4://135 from disired direction 1
                    testDir = TSVector.zero - test45Dir1;
                    break;
                case 5://135 to disired direction
                    testDir = CustomMath.perpendicular(test45Dir1);
                    testDir = testDir * dNormlDir < 0 ? testDir : (TSVector.zero - testDir);
                    break;
                case 6: //opposite direction
                    testDir = TSVector.zero - dNormlDir;
                    break;
            }
            TSVector pos2 = _behaviour.position + testDir * dirDst;

            if (bExtraCheck && !IsBlockedBetween2Point(_behaviour.position, pos2
                    , _behaviour.pathManager._queryStack,bIgnoreObstacle))
            {
                bool bOk = dirIdx < 2;
                if (dirIdx >= 2)
                {
                    FP dot = _behaviour.velocity * testDir;
                    if (dot < 0)
                    {
                        if (dot > maxDot)
                        {
                            maxDot = dot;
                            minAngleTestDir = testDir;
                        }
                        status = ECheckDirStatus.delay;
                    }
                    else
                    {
                        status = ECheckDirStatus.ok;
                    }
                }
                else
                {
                    status = ECheckDirStatus.ok;
                }
                if (status == ECheckDirStatus.ok)
                {
                    _behaviour.preVelocity = testDir * _behaviour.maxSpeed;
                    return status;
                }
            }
            return status;
        }
        protected bool IsBlockedBetween2Point(TSVector pos1, TSVector pos2, Stack<int> stack,bool bIgnoreObstacle)
        {
            if(!bIgnoreObstacle)
            {
                stack.Clear();
                TSVector blockedPos = TSVector.zero;
                bool hasObstacle = _behaviour.map.IsBlockedByObstacleBetween2Point(_behaviour.position, pos2
                    , stack, ref blockedPos);
                if (hasObstacle)
                {
                    return true;
                }
            }
           
            stack.Clear();
            bool hashNeighbour = _behaviour.baseData.pfm.HasNeighboursBetween2Point(_behaviour.proxyId, _behaviour.position
                , pos2, stack);
            return hashNeighbour;
        }
    }

}
