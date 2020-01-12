//author:huwei
//date:2018.6.10
using TrueSync;
using System.Collections.Generic;
//#define USE_OBSTACLE
namespace PathFinding
{

    public class ForceBasedAgent
    {
        protected static readonly FP _goalRadiusSq = FP.One * 1 / 100;
        protected static readonly FP _k = FP.One * 15 / 10;

        protected const int _t0 = 3;

        protected const int _m = 2;
        const int C_MaxColliderTime = 1;
        //AgentBehaviour
        // public IAgentBehaviour _behaviour;
        //obstacle neighbours
        // List<LineObstacle> _obstacleNeighbours = new List<LineObstacle>();
        //
        public ForceBasedAgent()
        {
            //  _enabled = false;
        }

        public void Reset()
        {
            // _behaviour = null;
            ClearData();
        }
        public void ClearData()
        {
            //  _vPref = _F = _velocity = TSVector2.zero;
        }

        public static void init()
        {
            // _goalRadiusSq = GridMap.nodeSize.AsFloat() * 0.25f;
            // initialize the ForceBasedAgent based on the initial conditions
            //_id = simEngine.getNumForceBasedAgents();
            //      _position = initialConditions.position;
            //_velocity = initialConditions.velocity;
            //_goal = initialConditions.goal;
            //_radius = initialConditions.radius;
            //_prefSpeed = initialConditions.prefSpeed;
            //_maxAccel = initialConditions.maxAccel;
            //_neighborDist = initialConditions.neighborDist;
            //_k = initialConditions.k;
            //_ksi = initialConditions.ksi;
            //_m = initialConditions.m;
            //_t0 = initialConditions.t0;
            //_goalRadiusSq = initialConditions.goalRadius* initialConditions.goalRadius;


        }


        public void doStep()
        {
            //_vPref = _goal - CustomMath.TSVecToVec2( _behaviour.position);
            //FP distSqToGoal = _vPref.LengthSquared();

            //if (distSqToGoal < _goalRadiusSq)
            //{
            //    _enabled = false;
            //    return;
            //}

            //// compute preferred velocity
            //_vPref *= _prefSpeed / TSMath.Sqrt(distSqToGoal);

            //// compute the new velocity of the ForceBasedAgent
            //computeForces();
        }
        static readonly FP static_radiusFactor = FP.One * 8 / 10;
        const int c_maxForceFactor= 1;
        static TSVector2 ComputeForce(IAgentBehaviour behaviour, TSVector pos, TSVector otherPosition,
          FP radiusSum, bool isStatic, FP maxTime, TSVector basicVelocity, TSVector otherVelocity
            ,ref bool isTminStaticAgent, ref FP tmin, ref FP time,ref bool isCollidering)
        {
            TSVector2 F = TSVector2.zero;
            TSVector dir = otherPosition - pos;
       
            if(otherVelocity==TSVector.zero && TSVector.Dot(dir, basicVelocity)<=0)
            {
                return F;
            }

            FP distanceSq = dir.sqrMagnitude;
            FP radiusSq = radiusSum * radiusSum;
            //  bool newMin = false;           
            // if (distanceSq != radiusSq)
            {
                // if ForceBasedAgents are actually colliding use their separation distance 
                TSVector2 v = CustomMath.TSVecToVec2(basicVelocity - otherVelocity);//warning
                if (distanceSq < radiusSq)
                {
                    isCollidering = true;
                    FP r = radiusSum - TSMath.Sqrt(distanceSq);
                    radiusSq = r * r;           
                    //F = -CustomMath.TSVecToVec2(dir).normalized * behaviour.baseData.maxForce * 3;
                    //return F;
                }
                TSVector2 w = CustomMath.TSVecToVec2(dir);
               
                FP a = v * v;
                FP b = w * v;
                FP c = w * w - radiusSq;
                FP discr = b * b - a * c;//(a*t^2-2*b*t+w*w-rSqr=0)
                if (discr > 0 && (a < -CustomMath.EPSILON || a > CustomMath.EPSILON))
                {
                    discr = TSMath.Sqrt(discr);
                    FP t = (b - discr) / a;
                    bool bMax = false;
                    TSVector2 val = TSVector2.zero;

                    if (t > 0 && t < maxTime)
                    {
                        if (t < tmin)
                        {
                            tmin = t;
                            time = t;
                            isTminStaticAgent = isStatic;
                            //  newMin = true;
                        }
                        TSVector2 vecDir = (v - (b * v - a * w));
                        //other.agent!=null? (v - (b * v - a * w)) 
                        //: CustomMath.perpendicular(v - (b * v - a * w));
                        //int rIdx=behaviour.pathManager.RadomIdx()%2;
                        //   vecDir = rIdx == 0 ? vecDir : vecDir * -1;
                        if (t < FP.EN1 * 2)
                        {
                            bMax = true;
                            //  F +=(-(v - (b * v - a * w) / discr)).normalized*behaviour.baseData.maxForce;
                        }
                        else
                        {
                            //-_k * System.Math.Exp((-t / _t0).AsFloat()) * (v - (b * v - a * w) / discr)
                            // (a * System.Math.Pow(t.AsFloat(), _m)) * (_m / t + FP.One / _t0);
                            //
                            FP fValX = -t / _t0;
                            val = -_k * CustomMath.ApproximateExp2(fValX) * vecDir
                            / (discr * a * t * t) * (_m / t + FP.One / _t0);//System.Math.Exp(().AsFloat())//System.Math.Pow(t.AsFloat(), _m)
                        }
                        if (bMax || TSMath.Abs(val.x) > behaviour.baseData.maxForce * c_maxForceFactor
                        || TSMath.Abs(val.y) > behaviour.baseData.maxForce * c_maxForceFactor)
                        {
                            FP maxXY = TSMath.Max(TSMath.Abs(vecDir.x), TSMath.Abs(vecDir.y));
                            if (maxXY>1)
                            {
                                vecDir = vecDir / maxXY;
                            }
                            val = -(vecDir).normalized * behaviour.baseData.maxForce * c_maxForceFactor;
                        }
                        //if(newMin)
                        //{
                        //    F = val;//only the most threatening agent
                        //}
                        F = val;
                    }

                }
            }
          
            return F;
        }
        internal static TSVector2 computeForces(IAgentBehaviour behaviour,List<CircleObstacleAngleData> circleObstacles, TSVector basicVelocity,bool bUseForwardPos,
           FP maxTime,FP obstacleMaxTime, out bool isTminStaticAgent,out FP time,ref bool isCollidering,bool bIgnoreObstacle=false)//,
        {          
            time = -1;
            TSVector2 F = TSVector2.zero;
            List <IAgentBehaviour> neighbours = behaviour.neighbours;
            int icount = neighbours.Count;
            isTminStaticAgent = false;
            FP tmin = FP.MaxValue;
            TSVector forwardPos =TSVector.zero;
            if(bUseForwardPos)
            {
                forwardPos= behaviour.position + basicVelocity.normalized * behaviour.colliderRadius;
            }
            //return TSVector2.zero;
            TSVector pos = (bUseForwardPos ? forwardPos : behaviour.position);
         
            // compute the anticipatory force from each neighbor
            for (int i = 0; i < icount; ++i)
            {
                IAgentBehaviour other = neighbours[i];
                FP radiusSum = other.colliderRadius;// + behaviour.colliderRadius;
                if (!bUseForwardPos)
                {
                    radiusSum = other.colliderRadius + behaviour.colliderRadius;
                }
                if (behaviour != other )
                {
                    maxTime = other.agent == null? obstacleMaxTime:maxTime;
                    F += ComputeForce(behaviour, pos, other.position, radiusSum, other.agent == null, maxTime,
                        basicVelocity, other.velocity, ref isTminStaticAgent, ref tmin, ref time,ref isCollidering);
#if UNITY_EDITOR && !MULTI_THREAD

                    if (PathFindingManager.DEBUG && behaviour.agent != null)
                    {
                        if (TSMath.Abs(F.x) > behaviour.baseData.maxForce * 1000
                            || TSMath.Abs(F.y) > behaviour.baseData.maxForce * 1000)
                        {
                            UnityEngine.Debug.Log("F over flow!");
                        }
                    }
#endif
                }
            }
            //
            if(!bIgnoreObstacle)
            {
                FP dirDst = GridMap.blockDirDst;
                TSVector testDir = basicVelocity.normalized;
                TSVector pos2 = behaviour.position + testDir * dirDst;

                behaviour.pathManager._queryStack.Clear();
                TSVector blockedPos = TSVector.zero;
                bool hasObstacle = behaviour.map.IsBlockedByObstacleBetween2Point(behaviour.position, pos2
                    , behaviour.pathManager._queryStack, ref blockedPos);
                if (hasObstacle)
                {
                    F = F - CustomMath.TSVecToVec2(testDir) * behaviour.baseData.maxForce;
                    time = FP.EN1;//near static obstacle
                    isTminStaticAgent = true;
                }
            }
           // behaviour.pathManager._queryStack.Clear();
            //
            //icount = circleObstacles.Count;
            //bool temp=false;
            //for (int i = 0; i < icount; ++i)
            //{
            //    CircleObstacle other = circleObstacles[i];
            //    FP radiusSum = other._radius;// + behaviour.colliderRadius;
            //    if (!bUseForwardPos)
            //    {
            //        radiusSum = other._radius + behaviour.colliderRadius;
            //    }
            //    maxTime = obstacleMaxTime;
            //    F += ComputeForce(behaviour, pos, CustomMath.TSVec2ToVec(other._center), radiusSum, true, maxTime,
            //             basicVelocity, TSVector.zero, ref isTminStaticAgent, ref tmin, ref time,ref temp);
            //}
#if USE_OBSTACLE
            //forces from static obstacles
            List<LineObstacle> obstacleNeighbours = behaviour.neighbourObstacles;
            icount = obstacleNeighbours.Count;
            TSVector2 position = CustomMath.TSVecToVec2(behaviour.position);
            TSVector2 velocity= CustomMath.TSVecToVec2(basicVelocity);
            FP radiusSqr = behaviour.colliderRadius * behaviour.colliderRadius;
            FP neighbourDstSqr = behaviour.neighbourRadius * behaviour.neighbourRadius;
            for (int i = 0; i < icount; ++i)
            {
                LineObstacle obstacle = obstacleNeighbours[i];
                TSVector2 n_w = CustomMath.ClosestPointLineSegment(obstacle._p1, obstacle._p2, position)- position;
                FP d_w = n_w.LengthSquared();

                if (velocity * n_w < 0 || d_w == radiusSqr || d_w > neighbourDstSqr) // Agent is moving away from obstacle, already colliding or obstacle too far away
                    continue;

                FP radius = d_w < radiusSqr ? TSMath.Sqrt(d_w) : behaviour.colliderRadius; // correct the radius, if the Agent is already colliding	
              
                FP a = velocity * velocity;
                bool discCollision = false, segmentCollision = false;
                FP t_min = FP.MaxValue;

                FP c = FP.Zero, b = FP.Zero, discr=FP.Zero;
                FP b_temp = FP.Zero, discr_temp = FP.Zero, c_temp = FP.Zero, D_temp = FP.Zero;
                TSVector2 w_temp = TSVector2.zero, w = TSVector2.zero, o1_temp = TSVector2.zero
                    , o2_temp = TSVector2.zero, o_temp = TSVector2.zero, o = TSVector2.zero, w_o = TSVector2.zero;

                // time-to-collision with disc_1 of the capped rectangle (capsule)
                w_temp = obstacle._p1 - position;
                b_temp = w_temp * velocity;
                c_temp = w_temp * w_temp - (radius * radius);
                discr_temp = b_temp * b_temp - a * c_temp;
                if (discr_temp > 0 && (a < -CustomMath.EPSILON || a > CustomMath.EPSILON))
                {
                    discr_temp = TSMath.Sqrt(discr_temp);
                    FP t = (b_temp - discr_temp) / a;
                    if (t > 0 && t<C_MaxColliderTime) {
                        t_min = t;
                        b = b_temp;
                        discr = discr_temp;
                        w = w_temp;
                        c = c_temp;
                        discCollision = true;
                    }
                }

                // time-to-collision with disc_2 of the capsule
                w_temp = obstacle._p2 - position;
                b_temp = w_temp * velocity;
                c_temp = w_temp * w_temp - (radius * radius);
                discr_temp = b_temp * b_temp - a * c_temp;
                if (discr_temp > 0 && (a < -CustomMath.EPSILON || a > CustomMath.EPSILON))
                {
                    discr_temp = TSMath.Sqrt(discr_temp);
                    FP t = (b_temp - discr_temp) / a;
                    if (t > 0 && t < t_min) {
                        t_min = t;
                        b = b_temp;
                        discr = discr_temp;
                        w = w_temp;
                        c = c_temp;
                        discCollision = true;
                    }
                }

                // time-to-collision with segment_1 of the capsule
                o1_temp = obstacle._p1 + radius * obstacle._normal;
                o2_temp = obstacle._p2 + radius * obstacle._normal;
                o_temp = o2_temp - o1_temp;

                D_temp =CustomMath.det(velocity, o_temp);
                if (D_temp != 0)
                {
                    FP inverseDet = 1 / D_temp;
                    FP t = CustomMath.det(o_temp, position - o1_temp) * inverseDet;
                    FP s = CustomMath.det(velocity, position - o1_temp) * inverseDet;
                    if (t > 0 && s >= 0 && s <= 1 && t < t_min)
                    {
                        t_min = t;
                        o = o_temp;
                        w_o = position - o1_temp;
                        discCollision = false;
                        segmentCollision = true;
                    }
                }

                // time-to-collision with segment_2 of the capsule
                o1_temp = obstacle._p1 - radius * obstacle._normal;
                o2_temp = obstacle._p2 - radius * obstacle._normal;
                o_temp = o2_temp - o1_temp;

                D_temp = CustomMath.det(velocity, o_temp);
                if (D_temp != 0)
                {
                    FP inverseDet = 1 / D_temp;
                    FP t = CustomMath.det(o_temp, position - o1_temp) * inverseDet;
                    FP s = CustomMath.det(velocity, position - o1_temp) * inverseDet;
                    if (t > 0 && s >= 0 && s <= 1 && t < t_min)
                    {
                        t_min = t;
                        o = o_temp;
                        w_o = position - o1_temp;
                        discCollision = false;
                        segmentCollision = true;
                    }
                }
                bool bMax = false;
                TSVector2 val = TSVector2.zero;
                if (discCollision)
                {                
                    if (t_min < FP.EN1 * 2)
                    {
                        bMax=true;
                       // F += (-(velocity - (b * velocity - a * w) / discr)).normalized * behaviour.baseData.maxForce;
                    }
                    else
                    {
                        val = -_k * System.Math.Exp((-t_min / _t0).AsFloat()) * (velocity - (b * velocity - a * w) / discr)
                       / (a * System.Math.Pow(t_min.AsFloat(), _m)) * (_m / t_min + FP.One / _t0);
                    }
                    if (bMax || TSMath.Abs(val.x)> behaviour.baseData.maxForce
                        || TSMath.Abs(val.y) > behaviour.baseData.maxForce)
                    {
                        val = (-(velocity - (b * velocity - a * w) / discr)).normalized * behaviour.baseData.maxForce;
                    }
                    F += val;
                }
                else if (segmentCollision)
                {
                    if (t_min < FP.EN1 * 2)
                    {
                        bMax = true;
                     //   F += (CustomMath.det(velocity, o)* new TSVector2(-o.y, o.x)).normalized * behaviour.baseData.maxForce;
                    }
                    else
                    {
                        val = _k * System.Math.Exp((-t_min / _t0).AsFloat()) / (System.Math.Pow(t_min.AsFloat(), _m)
                            * CustomMath.det(velocity, o)) * (_m / t_min + FP.One / _t0) * new TSVector2(-o.y, o.x);
                    }
                    if (bMax || TSMath.Abs(val.x) > behaviour.baseData.maxForce
                         || TSMath.Abs(val.y) > behaviour.baseData.maxForce)
                    {
                        val = (CustomMath.det(velocity, o) * new TSVector2(-o.y, o.x)).normalized 
                            * behaviour.baseData.maxForce;
                    }
                    F += val;
                }
#if UNITY_EDITOR

                if (PathFindingManager.DEBUG && behaviour.agent != null)
                {
                    if (TSMath.Abs(F.x) > behaviour.baseData.maxForce * 1000
                        || TSMath.Abs(F.y) > behaviour.baseData.maxForce * 1000)
                    {
                        UnityEngine.Debug.Log("F over flow!");
                    }
                }
#endif
            }
#endif
            //if(desiredF.LengthSquared()<F.LengthSquared())
            {
                return F;// + desiredF;
            }
            //else
            //{
            //    return desiredF;
            //}        
        }


        public void update(FP frameTime)
        {
            // update the ForceBasedAgents
            //TSVector2 acceleration = _F;
            // clamp(acceleration, _maxAccel);
            //if(_behaviour.agent!=null)
            //{
            //    _F= TSVector2.ClampMagnitude(acceleration, _behaviour.agent.maxForce);
            //}
  
            //_velocity = _velocity + acceleration * frameTime;
            // _position += _velocity * simEngine.getTimeStep();
        }
    }
}


//                if (other.agent == null)
//                {
//                    //
//                    //TSVector2 wNormal = w.normalized;
//                    //TSVector2 tangent = CustomMath.perpendicular(w);
//                    TSVector2 oPos = CustomMath.TSVecToVec2(behaviour.map.GetWorldPosition(other.position));
//                    //TSVector2 lineCenter =- wNormal * other.colliderRadius + oPos;

//                    //TSVector2 point1 = tangent * other.colliderRadius + lineCenter;
//                    //TSVector2 point2 = -tangent * other.colliderRadius + lineCenter;
//                    //LineObstacle o = new LineObstacle(point1, point2);
//                    //behaviour.neighbourObstacles.Add(o);

//                    //
//                    TSVector2 up = new TSVector2(0,1);
//                    TSVector2 right = new TSVector2(1, 0);
//                    TSVector2 point1 = oPos - up * other.colliderRadius*static_radiusFactor  - right * other.colliderRadius*static_radiusFactor;
//                    TSVector2 point2 = oPos - up * other.colliderRadius*static_radiusFactor + right * other.colliderRadius*static_radiusFactor;
//                    LineObstacle o = new LineObstacle(point1, point2);
//                    behaviour.neighbourObstacles.Add(o);
//#if UNITY_EDITOR
//                    if (PathFindingManager.DEBUG)
//                    {
//                        UnityEngine.Debug.DrawLine(CustomMath.TsVec2ToVector3(point1), CustomMath.TsVec2ToVector3(point2), UnityEngine.Color.red, 30);
//                    }
//#endif
//                    point1 = oPos - up * other.colliderRadius*static_radiusFactor + right * other.colliderRadius*static_radiusFactor;
//                    point2 = oPos + up * other.colliderRadius*static_radiusFactor + right * other.colliderRadius*static_radiusFactor;
//                    o = new LineObstacle(point1, point2);
//                    behaviour.neighbourObstacles.Add(o);
//#if UNITY_EDITOR
//                    if (PathFindingManager.DEBUG)
//                    {
//                        UnityEngine.Debug.DrawLine(CustomMath.TsVec2ToVector3(point1), CustomMath.TsVec2ToVector3(point2), UnityEngine.Color.red, 30);
//                    }
//#endif
//                    point1 = oPos + up * other.colliderRadius*static_radiusFactor  + right * other.colliderRadius*static_radiusFactor;
//                    point2 = oPos + up * other.colliderRadius*static_radiusFactor - right * other.colliderRadius*static_radiusFactor;
//                    o = new LineObstacle(point1, point2);
//                    behaviour.neighbourObstacles.Add(o);
//#if UNITY_EDITOR
//                    if (PathFindingManager.DEBUG)
//                    {
//                        UnityEngine.Debug.DrawLine(CustomMath.TsVec2ToVector3(point1), CustomMath.TsVec2ToVector3(point2), UnityEngine.Color.red, 30);
//                    }
//#endif
//                    point1 = oPos + up * other.colliderRadius*static_radiusFactor  - right * other.colliderRadius*static_radiusFactor;
//                    point2 = oPos - up * other.colliderRadius*static_radiusFactor - right * other.colliderRadius*static_radiusFactor;
//                    o = new LineObstacle(point1, point2);
//                    behaviour.neighbourObstacles.Add(o);
//#if UNITY_EDITOR
//                    if (PathFindingManager.DEBUG)
//                    {
//                        UnityEngine.Debug.DrawLine(CustomMath.TsVec2ToVector3(point1), CustomMath.TsVec2ToVector3(point2), UnityEngine.Color.red, 30);
//                    }                
//#endif
//                    continue;
//                }