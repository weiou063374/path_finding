///author:huwei
///date:2018.4.5
using System;
using System.Collections.Generic;

using AStarMachine;
using TrueSync;
namespace PathFinding
{
    public class FlowFieldAgent : AgentBase
    {
        IsWalkable _isvalid = null;//delegate gen gc
        IsPosWalkable _isPosWalkable = null;
        public override TSVector TargetPos { get { return _targetPos; }
            set { if (_targetPos != value) { flowField = _behaviour.baseData.pfm._flowManager.UpdateAgent(this._behaviour, _targetPos, value,_behaviour.map);
                    _isvalid = flowField.IsValid; _isPosWalkable = IsWalkableSkipDynamic; }; _targetPos = value; } }

        public FlowFieldGroup _group = null;
        internal FlowFieldAgent(IAgentBehaviour go) : base(go)
        {

        }
        public FlowFieldAgent() : base()
        {

        }
        public void Init(int eType,IAgentBehaviour agent)
        {
            _activeBoids =(int)eType-(int)EBoidsActiveType.collisionAvoidance-(int)EBoidsActiveType.avoidUnit;
            this._behaviour = agent;
            this.Init(agent);
        }
        public override void Reset()
        {
            base.Reset();
            flowField = null;
            _group = null;
        }
      
        public FlowField flowField = null;

        public bool IsWalkableSkipDynamic(TSVector pos)
        {
            int idx = flowField._gridMap.GetGridNodeId(pos);
            if (flowField._gridMap.IsDynamicUnwalkableNode(idx))
            {
                TSVector centerPos = flowField._gridMap.GetWorldPosition(idx);
                FP dot = TSVector.Dot(pos - _behaviour.position, centerPos - _behaviour.position);
                return dot <= 0;// CustomMath.FPHalf;//>=60 degree
            }
            IInt2 iPos = flowField._gridMap.GetNearestGridCoordWithoutClamp(pos);
            return flowField._gridMap.IsWalkable(iPos.x, iPos.y, true);
        }
        public override void SetDesiredPosOrVelocity(FP deltaTime, IsPosWalkable w = null, bool bSkipBoids = false)
        {          
            if(flowField!=null)
            {
                TSVector desiredDirection = flowField.steeringBehaviourFlowField(_behaviour);
                TSVector vec = _behaviour.position;// flowField._gridMap.GetGridFloatCoord(_behaviour.position);
                FP fDtSecond = deltaTime;
                List<IAgentBehaviour> agents = _behaviour.neighbours;
                int preBoidsType = _activeBoids;
                bool bUsePreBoidsForce= bSkipBoids && _preBoidsForce!=TSVector.zero;
                _activeBoids = !bUsePreBoidsForce ? _activeBoids : (byte)EBoidsActiveType.none;
               // bool bStatic = false;
                TSVector a =  ApplyForce(deltaTime,desiredDirection, _behaviour.velocity, vec,
                    agents, bUsePreBoidsForce,false); //TSVector.zero;// 
                TSVector deltaV = a * fDtSecond;
                FP maxSpeedSqr = _behaviour.maxSpeed * _behaviour.maxSpeed;
                //if (deltaV.sqrMagnitude> maxSpeedSqr)
                //{
                //    deltaV = deltaV.normalized * _behaviour.maxSpeed;
                //}
                _behaviour.preVelocity = (_behaviour.velocity + deltaV);//.normalized*_behaviour.maxSpeed;
                if (_behaviour.preVelocity.sqrMagnitude > _behaviour.maxSpeed * _behaviour.maxSpeed)//|| TSVector.Dot(_behaviour.velocity, a) > 0
                {
                    _behaviour.preVelocity = CustomMath.Normalize(_behaviour.preVelocity) * _behaviour.maxSpeed;
                }
#if UNITY_EDITOR && !MULTI_THREAD
                if (PathFindingManager.DEBUG)
                {
                    if (FP.Abs(_behaviour.preVelocity.x) > _behaviour.maxSpeed * 2 || FP.Abs(_behaviour.preVelocity.z) > _behaviour.maxSpeed * 2)
                    {
                        UnityEngine.Debug.LogError("velocity error!");
                    }
                }
#endif
                if (agents.Count>0)
                {
                    //IAgentBehaviour nearestNeighbour = agents[0];
                    //FP r = nearestNeighbour.colliderRadius + _behaviour.colliderRadius;
                    //TSVector dir = nearestNeighbour.position - _behaviour.position;
                    //if (nearestNeighbour.agent == null)//the ally neighbour has a target,
                    //                                   //cause this agent change type to atar type targeted to the same target
                    //{
                    //    PathFindingAgentBehaviour agent = _behaviour as PathFindingAgentBehaviour;
                    //    TSVector target = (nearestNeighbour as PathFindingAgentBehaviour)._targetPos;
                    //    if(target!=TSVector.MaxValue)
                    //    {
                    //        agent.SetNewTargetPos(target);
                    //    }                      
                    //   // agent.ChangeAgentType(EAgentType.astar);
                    //    return;
                    //}
                    //if (dir.sqrMagnitude<r*r )
                    //{                        
                    //    FP dot = TSVector.Dot(dir, _behaviour.preVelocity);
                    //    if(dot>0)//60 degree,CustomMath.FPHalf
                    //    {
                           
                    //        {
                    //            TSVector v = _behaviour.preVelocity;
                    //            v.x = -dir.z;
                    //            v.z = dir.x;
                    //            _behaviour.preVelocity = v.normalized * _behaviour.maxSpeed;//10;// _behaviour.preVelocity.magnitude;
                    //        }
                        
                    //        //TSVector v = _behaviour.preVelocity;
                    //        //v.x = 0;
                    //        //dot = dir.z *v.z;
                    //        //if(dot<= CustomMath.FPHalf)
                    //        //{
                    //        //    _behaviour.preVelocity = v.normalized*_behaviour.maxSpeed;
                    //        //}
                    //        //else
                    //        //{
                    //        //    v = _behaviour.preVelocity;
                    //        //    v.z = 0;
                    //        //    dot = dir.x * v.x;
                    //        //    if(dot<= CustomMath.FPHalf)
                    //        //    {
                    //        //        _behaviour.preVelocity = v.normalized * _behaviour.maxSpeed;
                    //        //    }
                    //        //    else
                    //        //    {
                    //        //        _behaviour.preVelocity = TSVector.zero;
                    //        //    }

                    //        //}
                    //    }
                    //}
                }
                _activeBoids = preBoidsType;
            }
        }
        public override void Loop(FP deltaTime,IsPosWalkable w=null, bool bSkipBoids = false)
        {
           // Navigate(bSkipBoids);
            if(flowField!=null)
            {
                base.Loop(deltaTime, _isPosWalkable, bSkipBoids);
            }
        }
    }
}
