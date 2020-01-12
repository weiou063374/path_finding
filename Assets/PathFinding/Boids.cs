///author:huwei
///date:2018.4.5
using System.Collections;
using System;

using TrueSync;
namespace PathFinding
{
   
    internal class Boids
    {
        public const int C_OBSTACLE_REPELL_FORCE = 50;
        #region steering behaviours
        //navigate agent toward desiredDirection
        internal static TSVector SteerTowards(IAgentBehaviour agent, TSVector desiredDirection,TSVector basicVelocity )
        {
            //desired speed
            TSVector desiredVelocity = desiredDirection * (agent.maxSpeed);
            //The velocity change 
            TSVector velocityChange = desiredVelocity - basicVelocity;//agent.velocity
            //force
            return velocityChange * agent.baseData.maxForce * agent.invMaxSpeed;
        }
        static readonly FP r_factor = FP.One*12/10;
        static readonly FP r_staticFactorSqr=(FP.One * 9/10)* (FP.One * 9/ 10);
       // static readonly FP r_dstMaxSqrInv =(FP.One / r_factor)* (FP.One / r_factor)*10;
        //Separation navigation
        internal static void BoidsBehaviourSeparation(IAgentBehaviour agent, IAgentBehaviour a, FP nrSqr, ref TSVector totalForce, ref int neighboursCount, bool bSkipStatic)
        {
            if (a != agent && a.enabled)
            {
                TSVector otherPos = a.position;
                if (agent.agent==null)//static agent
                {
                    if(bSkipStatic)
                    {
                        return;
                    }
                    otherPos= a.map.GetWorldPosition(a.map.GetGridNodeId(a.position));
                }
                TSVector pushForce = (agent.position - otherPos);
                FP distanceSqr = pushForce.sqrMagnitude;
                if (distanceSqr < nrSqr)//&& distanceSqr > 0
                {
                    //Vector to other agent                
                    //var length = pushForce.Normalize(); //
                    FP r = (agent.colliderRadius + a.colliderRadius);
                   // FP dst = distanceSqr;// TSMath.Sqrt(distanceSqr);
                    FP fWeight = 0;
                    FP dstMax = r_factor * r;
                    FP rSqr = r * r;
                    FP dstMaxSqr = dstMax * dstMax;
                    FP dst = TSMath.Sqrt(distanceSqr);
                    // fWeight =  (1 - (dst - r) / (agent.neighbourRadius - r));//(r+CustomMath.FPHalf- dst)/ dst;//
                    if (distanceSqr > rSqr && distanceSqr < dstMaxSqr)
                    {
                        //if ( a.agent != null)
                        {
                            fWeight = (1 - (dst - r) / (agent.neighbourRadius - r));// (dstMax);// (1 - (dst - r) / (agent.neighbourQTRadius - r));
                            if (a.agent == null)//static agent
                            {
                                fWeight *= 10;
                            }
                        }

                    }
                    else if (rSqr >= distanceSqr)
                    {
                        if(distanceSqr > r_staticFactorSqr * rSqr)
                        {
                            fWeight = 1+CustomMath.FPHalf/2;
                        }
                        else if(dst > FP.One / 10*GridMap.SCALE)
                        {
                           
                            {
                                fWeight = (r / dst * 2);
                                fWeight = fWeight * fWeight;
                            }
                        }                        
                        else
                        {
                            if(pushForce==TSVector.zero)
                            {
                                pushForce = agent.velocity-a.velocity;
                            }
                            fWeight = 10;
                        }
                    }
                    fWeight = TSMath.Min(30, fWeight);
                    if(fWeight>0)
                    {
                        totalForce = totalForce + (CustomMath.Normalize(pushForce) * fWeight);
                    }                  
                    neighboursCount++;
                }
            }
        }
        //Alignment navigation
        internal static void BoidsBehaviourAlignment(IAgentBehaviour agent, IAgentBehaviour a, FP nrSqr, ref TSVector averageHeading, ref int neighboursCount)
        {
            if(a.agent==null || !a.enabled)
            {
                return;
            }
            FP distanceSqr = (agent.position - a.position).sqrMagnitude;
            //within the max distance and are moving
            if (distanceSqr < nrSqr && a.velocity != TSVector.zero)
            {
                //Sum up headings    
                averageHeading = averageHeading + a.velocity.normalized;//warning:.normalized;,to be checked
                neighboursCount++;
            }
        }
        //Cohesion navigation
        internal static void BoidsBehaviourCohesion(IAgentBehaviour agent, IAgentBehaviour a, FP nrSqr, ref TSVector centerOfMass, ref int neighboursCount)
        {
            if (a != agent && a.agent!=null && a.enabled)
            {
                FP distanceSqr = (agent.position - a.position).sqrMagnitude;
                if (distanceSqr < nrSqr)
                {
                    //sum up the position of our neighbours
                    centerOfMass = centerOfMass + a.position;
                    neighboursCount++;
                }
            }
        }
        internal static TSVector BoidsBehaviourSeek(IAgentBehaviour agent, TSVector velocity, TSVector dest)
        {

            if (agent.position == dest)
            {
                return TSVector.zero;
            }
            TSVector desired = dest - agent.position;
            //Desired velocity
            desired = CustomMath.Normalize(desired) * (agent.maxSpeed);
            //velocity change
            TSVector velocityChange = desired - velocity;
            // force
            return velocityChange * agent.baseData.maxForce * agent.invMaxSpeed;
        }
        internal static TSVector BoidsBehaviourAvoidObstacle(TSVector agentPos,GridMap map,TSVector towardDir)
        {
            IInt2 floor = IInt2.zero;// GetNearestGridCoordWithoutClamp(agent.position);
            map.GetGridCoord(agentPos, ref floor);

            TSVector aPos = agentPos;// map.GetWorldPosition(map.GetIdx(floor.x,floor.y));
            TSVector desiredDirection = TSVector.zero;
            int iCount = 0;
            towardDir = CustomMath.Normalize(towardDir);
            for (int i = floor.x - 1; i <= floor.x + 1; i++)
            {
                for (int j = floor.y - 1; j <= floor.y + 1; j++)
                {
                    if (!map.IsWalkable(i, j,true))
                    {
                      //  int idx = map.GetIdx(i,j);
                        TSVector pos = map.GetWorldPositionWithOutClamp(i,j);
                        TSVector dir = aPos;// agentPos;
                        dir.Set(aPos.x - pos.x, 0, aPos.z - pos.z);
                        //FP dstSqr = dir.sqrMagnitude;
                        FP dot = 1;// TSVector.Dot(dir,towardDir);
                        //if(dot<=FP.Zero)
                        //{
                        //    dot = FP.One/(9*dstSqr);
                        //}
                        //else
                        //{
                        //    dot = dot / dstSqr;
                        //}
                        desiredDirection = desiredDirection + dir* dot;
                        iCount++;
                    }
                }
            }
            //if (iCount > 0)
            //{
            //    desiredDirection = desiredDirection / iCount;
            //}
            return desiredDirection;//.normalized*C_OBSTACLE_REPELL_FORCE;

        }
        #endregion
    }
}
