///author:huwei
///date:2018.4.5
using System;
using System.Collections.Generic;

using AStarMachine;
using TrueSync;
namespace PathFinding
{
    public class IgnoreObstacleAgent : AgentBase
    {
        IsPosWalkable _isPosWalkable = null;
        internal IgnoreObstacleAgent(IAgentBehaviour go) : base(go)
        {

        }
        public IgnoreObstacleAgent() : base()
        {

        }
        public void Init(int eType, IAgentBehaviour agent)
        {
            _activeBoids = (int)eType - (int)EBoidsActiveType.collisionAvoidance - (int)EBoidsActiveType.avoidUnit;
            this._behaviour = agent;
            this.Init(agent);
            if (_isPosWalkable == null)
            {
                _isPosWalkable = IsWalkable;
            }

        }
        public override void Reset()
        {
            base.Reset();
        }
        public bool IsWalkable(TSVector pos)
        {
            IInt2 iPos = _behaviour.map.GetNearestGridCoordWithoutClamp(pos);
            return _behaviour.map.IsInGridArea(iPos.x, iPos.y);
        }

        public override void SetDesiredPosOrVelocity(FP deltaTime, IsPosWalkable w = null, bool bSkipBoids = false)
        {
            bool bValidTargetPos = (_targetPos != TSVector.MinValue && _targetPos != TSVector.MaxValue);
            if (!bValidTargetPos)
            {
                return;
            }
            TSVector vec = _behaviour.position;// 
            FP fDtSecond = deltaTime;
            //List<IAgentBehaviour> agents = _behaviour.neighbours;
            int preBoidsType = _activeBoids;
            bool bUsePreBoidsForce = bSkipBoids && _preBoidsForce != TSVector.zero;
            _activeBoids = !bUsePreBoidsForce ? _activeBoids : (byte)EBoidsActiveType.none;
            // bool bStatic = false;
            //TSVector a = ApplyForce(deltaTime,_targetPos-_behaviour.position , _behaviour.velocity, vec, null
            //    , agents, bUsePreBoidsForce, false); //TSVector.zero;// 
            //TSVector deltaV = a * fDtSecond;
            //FP maxSpeedSqr = _behaviour.maxSpeed * _behaviour.maxSpeed;

            //_behaviour.preVelocity = (_behaviour.velocity + deltaV);//.normalized*_behaviour.maxSpeed;
            //if (_behaviour.preVelocity.sqrMagnitude > _behaviour.maxSpeed * _behaviour.maxSpeed)//|| TSVector.Dot(_behaviour.velocity, a) > 0
            //{
            //    _behaviour.preVelocity = CustomMath.Normalize(_behaviour.preVelocity) * _behaviour.maxSpeed;
            //}
            _behaviour.preVelocity = (_targetPos - _behaviour.position).normalized * _behaviour.maxSpeed;
#if UNITY_EDITOR && !MULTI_THREAD
            if (PathFindingManager.DEBUG)
            {
                if (FP.Abs(_behaviour.preVelocity.x) > _behaviour.maxSpeed * 2 || FP.Abs(_behaviour.preVelocity.z) > _behaviour.maxSpeed * 2)
                {
                    UnityEngine.Debug.LogError("velocity error!");
                }
            }
#endif
            _activeBoids = preBoidsType;
        }
        public override void CalculateFinalVelocity(FP deltaTime, bool bSkipBoids = false)
        {            
                FP fDtSecond = deltaTime;
                bool bReachedTarget = false;//(_targetPos - _behaviour.position).sqrMagnitude < GridMap.GetNodeSize();
                if (!bReachedTarget)
                {
                    TSVector pos = _behaviour.position;// ((GridMap)(_map)).GetGridFloatCoord(_behaviour.position);                  
                    
                    TSVector realAcc = TSVector.zero;//real acceleration

                    if (PathFindingManager.c_useAvoidUnit)
                    {
                        int preBoidsType = _activeBoids;
                        _activeBoids = (int)EBoidsActiveType.seperation;// (int)EBoidsActiveType.collisionAvoidance | 

                        TSVector velocity = _behaviour.velocity;

                        TSVector desiredV;
                        FP dFactor;
                        FP maxTime;
                        FP obstacleMaxTime = FP.Zero;
                        TSVector dA = DesiredAcc(_targetPos, out desiredV, out dFactor, out maxTime);//_targetPos
                        obstacleMaxTime = FP.One;// TSMath.Min(maxTime, FP.One);
                        maxTime = FP.One;// TSMath.Min(maxTime, FP.One);

                        if (dA != TSVector.zero || desiredV != TSVector.zero)
                        {
                            //check dA dir is clear or not
                            TSVector tryV = desiredV;//.normalized * _behaviour.maxSpeed;
                            bool bStatic = false;
                            FP ttcTry = -1;
                            bool isCollidering = false;
                            TSVector try_a = TSVector.zero;

                            {
                                try_a = CustomMath.TSVec2ToVec(ForceBasedAgent.computeForces
                                        (_behaviour, null, tryV, false, maxTime, obstacleMaxTime
                                        , out bStatic, out ttcTry, ref isCollidering, true))
                                        * _behaviour.baseData.invMass;
                            }

                            bool dDirClear = ttcTry < 0;// desired direction is clear or not
                            bool isRealAccZero = true;
                            bool isTminStaticAgent = false;
                            bool isNearStaticAgent = false;
                            TSVector sepA = TSVector.zero;

                            if (!dDirClear)//
                            {
                                isTminStaticAgent = false;
                                FP ttc = -1;
                                bool newMoveing = velocity == TSVector.zero;
                                if (newMoveing)
                                {
                                    int count = _behaviour.neighbours.Count;
                                    realAcc = CustomMath.perpendicular(desiredV);
                                }
                                else
                                {
                                    realAcc = CustomMath.TSVec2ToVec(ForceBasedAgent.computeForces(_behaviour, null, velocity, false,
                                        1, 1, out isTminStaticAgent, out ttc, ref isCollidering, true))
                                   * _behaviour.baseData.invMass;

                                    if (ttc > 0 && (realAcc * _behaviour.baseData.mass).sqrMagnitude > _behaviour.baseData.maxForceSqr)
                                    {
                                        realAcc = _behaviour.baseData.maxForce * _behaviour.baseData.invMass * realAcc.normalized;
                                    }
                                }

                                TSVector preA = realAcc;
                                isRealAccZero = realAcc == TSVector.zero;
                                if (!isRealAccZero && isTminStaticAgent)//current dir is not clear,blocked by static object
                                {
                                    //current direction is not clear,
                                    //then turn to perpendicular direction of avoid direction
                                    if (ttc > 0)
                                    {
                                        realAcc = CustomMath.perpendicular(realAcc);
                                    }
                                    bool changeA = false;
                                    if (realAcc * _behaviour.velocity < 0)// * desiredV
                                    {
                                        TSVector pos2 = _behaviour.position + realAcc.normalized * GridMap.blockDirDst;
                                        bool hashNeighbour = _behaviour.baseData.pfm.HasNeighboursBetween2Point(_behaviour.proxyId, _behaviour.position
                                                           , pos2, _behaviour.pathManager._queryStack);
                                        if (!hashNeighbour)
                                        {
                                            changeA = true;
                                        }
                                    }
                                    if (changeA)
                                    {
#if UNITY_EDITOR && !MULTI_THREAD
                                        if (PathFindingManager.DEBUG)
                                        {
                                            UnityEngine.Debug.Log("change real acc dir");
                                        }
#endif
                                    realAcc = TSVector.zero - realAcc;
                                    }
                                }
                                isNearStaticAgent = ttc < FP.EN1 * 2 && isTminStaticAgent;
                                // otherwise,take boids force into consideration 
                                if (isCollidering)
                                {
                                    sepA = ApplyForce(deltaTime, TSVector.zero, velocity, pos,
                                 _behaviour.neighbours, bSkipBoids, false);// 
                                    realAcc = realAcc + sepA;
                                }
                                isRealAccZero = realAcc == TSVector.zero;
                            }

                            if (isRealAccZero)//current diretion is clear
                            {
                                //                                      
                                if (!dDirClear)//disired diretion is not clear
                                {
                                    //check perpendicular of velocity is clear or not,1,5
                                    if (_behaviour.velocity == TSVector.zero)
                                    {
                                        _behaviour.preVelocity = CustomMath.perpendicular(desiredV);
                                        velocity = _behaviour.velocity;
                                    }
                                    tryV = (desiredV * _behaviour.invMaxSpeed + _behaviour.preVelocity.normalized * _currentVelFactor).normalized * _behaviour.maxSpeed; //CustomMath.perpendicular(velocity);
                                                                                                                                                                         //  tryV = (TSVector.Dot(tryV, desiredV) >= 0 ? tryV : (TSVector.zero - tryV)).normalized * _behaviour.maxSpeed;// velocity.magnitude
                                    bStatic = false;
                                    ttcTry = -1;
                                    try_a = CustomMath.TSVec2ToVec(ForceBasedAgent.computeForces
                                            (_behaviour, null, tryV, false, 1, 1, out bStatic, out ttcTry, ref isCollidering, true)) * _behaviour.baseData.invMass;
                                    if (ttcTry > 0)//not clear,use pre velocity  ttcTry < FP.EN1*2 &&
                                    {
                                        _currentVelFactor = Max_Vel_Factor;
                                        _behaviour.preVelocity = velocity.normalized * _behaviour.maxSpeed;
#if UNITY_EDITOR && !MULTI_THREAD
                                        if (PathFindingManager.DEBUG)
                                        {
                                            UnityEngine.Debug.Log("use pre velocity :" + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                            TSVector dirPos = _behaviour.position + _behaviour.preVelocity.normalized * 2;
                                            UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(_behaviour.position),
                                               CustomMath.TsVecToVector3(dirPos), UnityEngine.Color.magenta, 0.1f);
                                        }
#endif
                                    //   _nextCalDesiredVelTime = _behaviour.baseData.pfm.CurTime
                                    //+ PathFindingAgentBehaviour.C_DESIRED_DELTATIME * 2;
                                    return;
                                    }
                                    else// test dir is clear then turn to test dir
                                    {
                                        _currentVelFactor *= 7 * FP.EN1;
                                        _behaviour.preVelocity = tryV;//
#if UNITY_EDITOR && !MULTI_THREAD
                                        if (PathFindingManager.DEBUG)
                                        {
                                            UnityEngine.Debug.Log("turn to test dir:" + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                            TSVector dirPos = _behaviour.position + _behaviour.preVelocity.normalized * 2;
                                            UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(_behaviour.position),
                                               CustomMath.TsVecToVector3(dirPos), UnityEngine.Color.cyan, 0.1f);
                                        }
#endif
                                    //dA = tryV;
                                    return;
                                    }
                                }
                                else//turn to desired direction
                                {
                                    _behaviour.preVelocity = CustomMath.Normalize(desiredV) * _behaviour.maxSpeed;
#if UNITY_EDITOR && !MULTI_THREAD
                                    if (PathFindingManager.DEBUG)
                                    {
                                        UnityEngine.Debug.Log("turn to desired direction:" + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                                    }
#endif
                                return;
                                }

                            }//current diretion is not clear,go on
                             //has static block nearby,try to find clear dir
                            if (isNearStaticAgent && !isCollidering)//8 directions  isTminStaticAgent
                            {
                                TSVector dNormlDir = desiredV * _behaviour.invMaxSpeed;
                                TSVector perpendicular1 = CustomMath.perpendicular(dNormlDir);
                                TSVector perpendicular2 = TSVector.zero - perpendicular1;

                                //45 from disired direction
                                TSVector test45Dir1 = (perpendicular1 + dNormlDir).normalized;

                                TSVector minAngleTestDir = perpendicular1;
                                FP maxDot = FP.MinValue;
                                bool bOk = false;
                                for (int i = 0; i < 7; i++)
                                {
                                    ECheckDirStatus status = CheckDir(i, dNormlDir, perpendicular1, perpendicular2, test45Dir1
                                        , ref maxDot, ref minAngleTestDir, true);
                                    if (status == ECheckDirStatus.ok)
                                    {
                                        bOk = true;
                                        break;
                                    }
                                    else if (status == ECheckDirStatus.delay)
                                    {
                                        bOk = true;
                                    }
                                }
                                if (maxDot != FP.MinValue)//status = ECheckDirStatus.delay
                                {
                                    _behaviour.preVelocity = minAngleTestDir * _behaviour.maxSpeed;
                                    if (minAngleTestDir == perpendicular1)//??????????
                                    {
                                        _preTestOKVec = perpendicular1;
                                    }
                                    else if (minAngleTestDir == perpendicular2)
                                    {
                                        _preTestOKVec = perpendicular2;
                                    }
                                }
                                if (bOk)
                                {
                                    return;
                                }
                            }

                            TSVector validA = dA + realAcc;
                            // FP accFactor = 1;
                            FP validTime = deltaTime;
                            bool ignoreDA = (!isRealAccZero)
                                && (isTminStaticAgent || sepA != TSVector.zero);
                            //not calculating desired velocity and (near static block or boids force influences )
                            if (ignoreDA)// && _nextCalDesiredVelTime > _behaviour.baseData.pfm.CurTime
                            {
                                validA = realAcc;
                            // accFactor = 1;
#if UNITY_EDITOR && !MULTI_THREAD
                                if (PathFindingManager.DEBUG)
                                {
                                    UnityEngine.Debug.Log("validA:a:" + CustomMath.TsVecToVector3(realAcc)
                                        + ",isTminStaticAgent:" + isTminStaticAgent + ",sepA:" + CustomMath.TsVecToVector3(sepA)
                                        + ",isNearStaticAgent" + isNearStaticAgent);
                                }
#endif
                        }
                        else
                            {
#if UNITY_EDITOR && !MULTI_THREAD
                                if (PathFindingManager.DEBUG)
                                {
                                    UnityEngine.Debug.Log("calculating desired velocity or not static blocked");
                                }
#endif
                        }
                        if (((validA) * _behaviour.baseData.mass).sqrMagnitude > _behaviour.baseData.maxForceSqr)
                            {
                                validA = validA.normalized * _behaviour.baseData.maxForce * _behaviour.baseData.invMass;
                            }
                            _behaviour.preVelocity = velocity + (validA) * validTime;//

#if UNITY_EDITOR && !MULTI_THREAD
                            if (PathFindingManager.DEBUG)
                            {
                                TSVector forcePos = _behaviour.position + validA.normalized * 2;
                                UnityEngine.Debug.DrawLine(CustomMath.TsVecToVector3(_behaviour.position),
                                    CustomMath.TsVecToVector3(forcePos), UnityEngine.Color.yellow, 0.1f);
                            }
#endif

                        if (_behaviour.preVelocity.sqrMagnitude > _behaviour.maxSpeed * _behaviour.maxSpeed)//|| TSVector.Dot(_behaviour.velocity, a) > 0
                            {
                                _behaviour.preVelocity = CustomMath.Normalize(_behaviour.preVelocity) * _behaviour.maxSpeed;
                            }
#if UNITY_EDITOR && !MULTI_THREAD
                            if (PathFindingManager.DEBUG)
                            {
                                UnityEngine.Debug.Log(" _behaviour.preVelocity:" + CustomMath.TsVecToVector3(_behaviour.preVelocity));
                            }
#endif
                        // _preForceAcc = a;
                    }
                    else if (desiredV == TSVector.zero)
                        {
                            _behaviour.preVelocity = TSVector.zero;
                        }
                        _activeBoids = preBoidsType;

                    }
                }
        }
        public override void Loop(FP deltaTime, IsPosWalkable w = null, bool bSkipBoids = false)
        {
            base.Loop(deltaTime, _isPosWalkable, bSkipBoids);
        }
    }
}
