using TrueSync;
using System.Collections.Generic;

namespace PathFinding
{
    public class SteerForUnitAvoidance
    {
        /// <summary>
        /// margin for considered radius.
        /// </summary>
        public static readonly FP radiusMargin = FP.One/10 ;

         public static readonly  bool accumulateAvoidVectors = false;

        /// <summary>
        /// The angle at which a perpendicular is computed to the otherwise backwards-directed fleeing avoid vector.
        /// </summary>
        //Range(90 180]
        public static readonly FP headOnCollisionAngle = 175;

        /// <summary>
        /// The minimum avoid vector magnitude required, otherwise the avoid vector is ignored.
        /// </summary>
       public static readonly FP minimumAvoidVectorMagnitude = FP.One/4;

        /// <summary>
        /// When true this unit will attempt to avoid behind other units, instead of avoiding in front of them.
        /// </summary>
        public static bool preventPassingInFront = false;


        /// <summary>
        /// A hardcoded value defining how far into the collision in time that the avoidance vector is computed at.
        /// 0.25 = 25 % inside the collision time interval.
        /// </summary>
        private static readonly FP _collisionTimeFactor = FP.One/4;


        private static readonly FP _fovReverseAngleCos = TSMath.Cos(((360 - 200) / 2) * TSMath.Deg2Rad);
 
        private static readonly FP _cosAvoidAngle = TSMath.Cos(headOnCollisionAngle * TSMath.Deg2Rad);
    
      //  private TSVector _selfCollisionPos;
      //  private TSVector _lastAvoidVector;
      //  private TSVector _lastAvoidPos;
 

        /// <summary>
        /// Gets the desired steering delta velocity
        /// </summary>
        /// <param name="input">The steering input containing relevant information to use when calculating the steering output.</param>
        /// <param name="output">The steering output to be populated.</param>
        public static TSVector GetDesiredSteering(IAgentBehaviour agent)//FP deltaTime,
        {
           // _selfCollisionPos = TSVector.zero;
          //  _lastAvoidVector = TSVector.zero;
          //  _lastAvoidPos = TSVector.zero;

            var otherUnits = agent.neighbours;
            int othersCount = otherUnits.Count;
            if (othersCount == 0)
            {
                // if the scanner has found no units to avoid, exit
                return TSVector.zero;
            }


            TSVector avoidVector = Avoid(otherUnits, agent, othersCount,agent.velocity);
            if (avoidVector.sqrMagnitude < (minimumAvoidVectorMagnitude * minimumAvoidVectorMagnitude))
            {
                // if the computed avoid vector's magnitude is less than the minimumAvoidVectorMagnitude, then discard it
                return TSVector.zero;
            }

            // apply the avoidance force as a full deceleration capped force (not over time)
        
           // _lastAvoidVector = steeringVector;
            return avoidVector;
        }

        /// <summary>
        /// Avoids the specified units.
        /// </summary>
        /// <param name="units">The units list.</param>
        /// <param name="unitsLength">Length of the units list.</param>
        /// <param name="currentVelocity">This unit's current velocity.</param>
        /// <returns>An avoid vector, if there are any to avoid, otherwise TSVector.zero.</returns>
        private static TSVector Avoid(List<IAgentBehaviour> units,IAgentBehaviour agent, int unitsLength, TSVector currentVelocity)
        {
         
            TSVector normalVelocity = CustomMath.Normalize(currentVelocity);
            TSVector selfPos = agent.position + normalVelocity;
            TSVector combinedAvoidVector = TSVector.zero;

            // iterate through scanned units list
            for (int i = 0; i < unitsLength; i++)
            {
                var other = units[i];
                if (!other.enabled)
                {
                    continue;
                }

                //if (_unitData.transientGroup != null && object.ReferenceEquals(other.transientGroup, _unitData.transientGroup))
                //{
                //    // ignore units in same transient unit group
                //   // continue;
                //}

                //if (other.determination < _unitData.determination)
                //{
                //    // ignore units with lower determination
                //    continue;
                //}

                TSVector otherPos = other.position;
                TSVector direction = otherPos-selfPos;
                FP distance = direction.magnitude;
                FP omniAwareRadius = agent.colliderRadius ;
                if (distance > omniAwareRadius && TSVector.Dot(normalVelocity, direction / distance) > _fovReverseAngleCos)
                {
                    // the other unit is behind me and outside my 'omni aware radius', ignore it
                    continue;
                }

                FP combinedRadius = other.colliderRadius + other.colliderRadius + radiusMargin*GridMap.GetNodeSize();
                TSVector otherVelocity = other.velocity;
                TSVector avoidVector = GetAvoidVector(selfPos, currentVelocity, normalVelocity, agent, otherPos, otherVelocity, other, combinedRadius);
                if (accumulateAvoidVectors)
                {
                    // if accumulating, then keep summing avoid vectors up
                    combinedAvoidVector += avoidVector;
                }
                else
                {
                    // if not accumulating, then break after the first avoid vector is found
                    combinedAvoidVector = avoidVector;
                    break;
                }
            }
            return combinedAvoidVector;
        }

        /// <summary>
        /// Gets an avoidance vector.
        /// </summary>
        /// <param name="selfPos">This unit's position.</param>
        /// <param name="currentVelocity">This unit's current velocity.</param>
        /// <param name="normalVelocity">This unit's normalized current velocity.</param>
        /// <param name="unitData">This unit's UnitFacade.</param>
        /// <param name="otherPos">The other unit's position.</param>
        /// <param name="otherVelocity">The other unit's velocity.</param>
        /// <param name="otherData">The other unit's UnitFacade.</param>
        /// <param name="combinedRadius">The combined radius.</param>
        /// <returns>An avoidance vector from the other unit's collision position to this unit's collision position - if a collision actually is detected.</returns>
        private static TSVector GetAvoidVector(TSVector selfPos, TSVector currentVelocity, TSVector normalVelocity, IAgentBehaviour unitData, TSVector otherPos, TSVector otherVelocity, IAgentBehaviour otherData, FP combinedRadius)
        {
            TSVector selfCollisionPos = TSVector.zero;
            TSVector avoidDirection = GetAvoidDirectionVector(selfPos, currentVelocity, otherPos, otherVelocity, combinedRadius,out selfCollisionPos);
            FP avoidMagnitude = avoidDirection.magnitude;
            if (avoidMagnitude == 0)
            {
                // if there is absolutely no magnitude to the found avoid direction, then ignore it
                return TSVector.zero;
            }

            FP vectorLength = combinedRadius * CustomMath.FPHalf;
            if (vectorLength <= 0)
            {
                // if the units' combined radius is 0, then we cannot avoid
                return TSVector.zero;
            }

            // normalize the avoid vector and then set it's magnitude to the desired vector length (half of the combined radius)
            TSVector avoidNormalized = (avoidDirection / avoidMagnitude);
            TSVector avoidVector = avoidNormalized * vectorLength;

            FP dotAngle = TSVector.Dot(avoidNormalized, normalVelocity);
            if (dotAngle <= _cosAvoidAngle)
            {
                // the collision is considered "head-on", thus we compute a perpendicular avoid vector instead
                avoidVector = new TSVector(avoidVector.z, avoidVector.y, -avoidVector.x);
            }
            else if (preventPassingInFront 
                //&& (otherData.determination > unitData.determination)
                && (TSVector.Dot(otherVelocity, avoidVector) > 0 && TSVector.Dot(currentVelocity, otherVelocity) >= 0))
            {
                // if supposed to be preventing front-passing, then check whether we should prevent it in this case and if so compute a different avoid vector
                avoidVector = selfCollisionPos - selfPos;
            }

            // scale the avoid vector depending on the distance to collision, shorter distances need larger magnitudes and vice versa
            FP collisionDistance = TSMath.Max(1, (selfPos- selfCollisionPos).magnitude);
            avoidVector *= currentVelocity.magnitude / collisionDistance;

            return avoidVector;
        }

        /// <summary>
        /// Gets the avoid direction vector.
        /// </summary>
        /// <param name="selfPos">This unit's position.</param>
        /// <param name="currentVelocity">This unit's current velocity.</param>
        /// <param name="otherPos">The other unit's position.</param>
        /// <param name="otherVelocity">The other unit's velocity.</param>
        /// <param name="combinedRadius">The combined radius.</param>
        /// <returns>An avoidance direction vector, if a collision is detected.</returns>
        private static TSVector GetAvoidDirectionVector(TSVector selfPos, TSVector currentVelocity, TSVector otherPos, TSVector otherVelocity, FP combinedRadius, out TSVector selfCollisionPos)
        {
            selfCollisionPos = TSVector.zero;
            // use a 2nd degree polynomial function to determine intersection points between moving units with a velocity and radius
            FP a = ((currentVelocity.x - otherVelocity.x) * (currentVelocity.x - otherVelocity.x)) +
                      ((currentVelocity.z - otherVelocity.z) * (currentVelocity.z - otherVelocity.z));
            FP b = (2 * (selfPos.x - otherPos.x) * (currentVelocity.x - otherVelocity.x)) +
                      (2 * (selfPos.z - otherPos.z) * (currentVelocity.z - otherVelocity.z));
            FP c = ((selfPos.x - otherPos.x) * (selfPos.x - otherPos.x)) +
                      ((selfPos.z - otherPos.z) * (selfPos.z - otherPos.z)) -
                      (combinedRadius * combinedRadius);

            FP d = (b * b) - (4 * a * c);
            if (d <= 0)
            {
                // if there are not 2 intersection points, then skip
                return TSVector.zero;
            }

            // compute "heavy" calculations only once
            FP dSqrt = TSMath.Sqrt(d);
            FP doubleA = 2 * a;

            // compute roots, which in this case are actually time values informing of when the collision starts and ends
            FP t1 = (-b + dSqrt) / doubleA;
            FP t2 = (-b - dSqrt) / doubleA;

            if (t1 < 0 && t2 < 0)
            {
                // if both times are negative, the collision is behind us (compared to velocity direction)
                return TSVector.zero;
            }

            // find the lowest non-negative time, since this will be where the collision time interval starts
            FP time = 0;
            if (t1 < 0)
            {
                time = t2;
            }
            else if (t2 < 0)
            {
                time = t1;
            }
            else
            {
                time = TSMath.Min(t1, t2);
            }

            // the collision time we want is actually 25 % within the collision
            time += TSMath.Abs(t2 - t1) * _collisionTimeFactor;

            // compute actual collision positions
            selfCollisionPos = selfPos + (currentVelocity * time);
           // _selfCollisionPos = selfCollisionPos;
            TSVector otherCollisionPos = otherPos + (otherVelocity * time);
           // _lastAvoidPos = otherPos;

            // return an avoid vector from the other's collision position to this unit's collision position
            return otherCollisionPos-selfCollisionPos;
        }


        //private void OnDrawGizmos()
        //{
        //    if (!drawGizmos)
        //    {
        //        return;
        //    }

        //    if (_unitData == null || _unitData.gameObject == null)
        //    {
        //        return;
        //    }

        //    var renderer = _unitData.gameObject.GetComponent<Renderer>();
        //    if (renderer == null || renderer.material == null)
        //    {
        //        return;
        //    }

        //    var c = renderer.material.color;
        //    Gizmos.color = c;

        //    if (_lastAvoidVector.sqrMagnitude != 0)
        //    {
        //        Gizmos.DrawLine(_unitData.position, _unitData.position + _lastAvoidVector);
        //    }

        //    if (_lastAvoidPos.sqrMagnitude != 0)
        //    {
        //        Gizmos.DrawSphere(_selfCollisionPos, 0.25f);
        //    }
        //}
    }
}