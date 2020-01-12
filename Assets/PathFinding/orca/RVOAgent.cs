//#define USING_ORCA
using System.Collections.Generic;
using TrueSync;

namespace PathFinding
{

    internal class WorkerContext
    {
        public RVOAgent.VOBuffer vos = new RVOAgent.VOBuffer(16);

        public static readonly int KeepCount = 3;
        public TSVector2[] bestPos = new TSVector2[KeepCount];
        public FP[] bestSizes = new FP[KeepCount];
        public FP[] bestScores = new FP[KeepCount + 1];

        public TSVector2[] samplePos = new TSVector2[50];
        public FP[] sampleSize = new FP[50];
    }
    /** Internal agent for the RVO system.
	 * Usually you will interface with the IAgent interface instead.
	 *
	 * \see IAgent
	 */
    public class RVOAgent : IResetable
    {
        //Current values for double buffer calculation
        public RVOAgent() { }
        public void Reset()
        {

        }
        internal PathFindingAgentBehaviour _behaviour=null;
        internal FP radius, height, desiredSpeed, maxSpeed, agentTimeHorizon, obstacleTimeHorizon;
        internal bool locked = false;

        int layer, collidesWith;

        int maxNeighbours;
        internal TSVector2 position;
        FP elevationCoordinate;
        TSVector2 currentVelocity;

        /** Desired target point - position */
        TSVector2 desiredTargetPointInVelocitySpace;
        TSVector2 desiredVelocity;

        TSVector2 nextTargetPoint;
        FP nextDesiredSpeed;
        FP nextMaxSpeed;
        TSVector2 collisionNormal;
        bool manuallyControlled;
        bool debugDraw;

        #region IAgent Properties

        /** \copydoc Pathfinding::RVO::IAgent::Position */
        public TSVector2 Position { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::ElevationCoordinate */
        public FP ElevationCoordinate { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::CalculatedTargetPoint */
        public TSVector2 CalculatedTargetPoint { get; private set; }

        /** \copydoc Pathfinding::RVO::IAgent::CalculatedSpeed */
        public FP CalculatedSpeed { get; private set; }

        /** \copydoc Pathfinding::RVO::IAgent::Locked */
        public bool Locked { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::Radius */
        public FP Radius { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::Height */
        public FP Height { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::AgentTimeHorizon */
        public FP AgentTimeHorizon { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::ObstacleTimeHorizon */
        public FP ObstacleTimeHorizon { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::MaxNeighbours */
        public int MaxNeighbours { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::NeighbourCount */
        public int NeighbourCount { get; private set; }

        /** \copydoc Pathfinding::RVO::IAgent::Layer */
        public int Layer { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::CollidesWith */
        public int CollidesWith { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::DebugDraw */
        public bool DebugDraw
        {
            get
            {
                return debugDraw;
            }
            set
            {
                debugDraw = value ;
            }
        }

        /** \copydoc Pathfinding::RVO::IAgent::Priority */
        public FP Priority { get; set; }

        /** \copydoc Pathfinding::RVO::IAgent::PreCalculationCallback */
        public System.Action PreCalculationCallback { private get; set; }

        #endregion

        #region IAgent Methods

        /** \copydoc Pathfinding::RVO::IAgent::SetTarget */
        public void SetTarget(TSVector2 targetPoint, FP desiredSpeed, FP maxSpeed)
        {
            maxSpeed = TSMath.Max(maxSpeed, 0);
            desiredSpeed = TSMath.Min(TSMath.Max(desiredSpeed, 0), maxSpeed);

            nextTargetPoint = targetPoint;
            nextDesiredSpeed = desiredSpeed;
            nextMaxSpeed = maxSpeed;
        }

        /** \copydoc Pathfinding::RVO::IAgent::SetCollisionNormal */
        public void SetCollisionNormal(TSVector2 normal)
        {
            collisionNormal = normal;
        }

        /** \copydoc Pathfinding::RVO::IAgent::ForceSetVelocity */
        public void ForceSetVelocity(TSVector2 velocity)
        {
            // A bit hacky, but it is approximately correct
            // assuming the agent does not move significantly
            nextTargetPoint = CalculatedTargetPoint = position + velocity * 1000;
            nextDesiredSpeed = CalculatedSpeed = velocity.magnitude;
            manuallyControlled = true;
        }

        #endregion

        /** Used internally for a linked list */
       // internal RVOAgent next;

        FP calculatedSpeed;
        TSVector2 calculatedTargetPoint;

 
        static readonly FP DesiredVelocityWeight = FP.One/10;

        /** Extra weight that walls will have */
        static readonly FP WallWeight = 5;

        //public List<ObstacleVertex> NeighbourObstacles
        //{
        //    get
        //    {
        //        return null;
        //    }
        //}

        public RVOAgent(TSVector2 pos, FP elevationCoordinate)
        {
            AgentTimeHorizon = 2;
            ObstacleTimeHorizon = 2;
            Height = 5;
            Radius = 5;
            MaxNeighbours = 10;
            Locked = false;
            Position = pos;
            ElevationCoordinate = elevationCoordinate;
            Layer = 1;
            CollidesWith =1;
            Priority = CustomMath.FPHalf;
            CalculatedTargetPoint = pos;
            CalculatedSpeed = 0;
            SetTarget(pos, 0, 0);
        }

        /** Reads public properties and stores them in internal fields.
		 * This is required because multithreading is used and if another script
		 * updated the fields at the same time as this class used them in another thread
		 * weird things could happen.
		 *
		 * Will also set CalculatedTargetPoint and CalculatedSpeed to the result
		 * which was last calculated.
		 */
        public void BufferSwitch()
        {
            // <== Read public properties
            radius = Radius;
            height = Height;
            maxSpeed = nextMaxSpeed;
            desiredSpeed = nextDesiredSpeed;
            agentTimeHorizon = AgentTimeHorizon;
            obstacleTimeHorizon = ObstacleTimeHorizon;
            maxNeighbours = MaxNeighbours;
            // Manually controlled overrides the agent being locked
            // (if one for some reason uses them at the same time)
            locked = Locked && !manuallyControlled;
            position = Position;
            elevationCoordinate = ElevationCoordinate;
            collidesWith = CollidesWith;
            layer = Layer;

            if (locked)
            {
                // Locked agents do not move at all
                desiredTargetPointInVelocitySpace = position;
                desiredVelocity = currentVelocity = TSVector2.zero;
            }
            else
            {
                desiredTargetPointInVelocitySpace = nextTargetPoint - position;

                // Estimate our current velocity
                // This is necessary because other agents need to know
                // how this agent is moving to be able to avoid it
                currentVelocity = (CalculatedTargetPoint - position).normalized * CalculatedSpeed;

                // Calculate the desired velocity from the point we want to reach
                desiredVelocity = desiredTargetPointInVelocitySpace.normalized * desiredSpeed;

                if (collisionNormal != TSVector2.zero)
                {
                    collisionNormal.Normalize();
                    var dot = TSVector2.Dot(currentVelocity, collisionNormal);

                    // Check if the velocity is going into the wall
                    if (dot < 0)
                    {
                        // If so: remove that component from the velocity
                        currentVelocity -= collisionNormal * dot;
                    }

                    // Clear the normal
                    collisionNormal = TSVector2.zero;
                }
            }
        }

        public void PreCalculation()
        {
            if (PreCalculationCallback != null)
            {
                PreCalculationCallback();
            }
        }

        public void PostCalculation()
        {
            // ==> Set public properties
            if (!manuallyControlled)
            {
                CalculatedTargetPoint = calculatedTargetPoint;
                CalculatedSpeed = calculatedSpeed;
            }

            manuallyControlled = false;
        }

        /** Square a number */
        static FP Sqr(FP x)
        {
            return x * x;
        }

        /** (x, 0, y) */
        static TSVector FromXZ(TSVector2 p)
        {
            return new TSVector(p.x, 0, p.y);
        }

        /** (x, z) */
        static TSVector2 ToXZ(TSVector p)
        {
            return new TSVector2(p.x, p.z);
        }

        TSVector2 To2D(TSVector p, out FP elevation)
        {
            elevation = p.y;
            return new TSVector2(p.x, p.z);
        }

        static void DrawVO(TSVector2 circleCenter, FP radius, TSVector2 origin)
        {
            FP alpha = TSMath.Atan2((origin - circleCenter).y, (origin - circleCenter).x);
            FP gamma = radius / (origin - circleCenter).magnitude;
            FP delta = gamma <= FP.One ? TSMath.Abs(TSMath.Acos(gamma)) : 0;

           // Draw.Debug.CircleXZ(FromXZ(circleCenter), radius, Color.black, alpha - delta, alpha + delta);
            TSVector2 p1 = new TSVector2(TSMath.Cos(alpha - delta), TSMath.Sin(alpha - delta)) * radius;
            TSVector2 p2 = new TSVector2(TSMath.Cos(alpha + delta), TSMath.Sin(alpha + delta)) * radius;

            TSVector2 p1t = -new TSVector2(-p1.y, p1.x);
            TSVector2 p2t = new TSVector2(-p2.y, p2.x);
            p1 += circleCenter;
            p2 += circleCenter;

          //  Debug.DrawRay(FromXZ(p1), FromXZ(p1t).normalized * 100, Color.black);
          //  Debug.DrawRay(FromXZ(p2), FromXZ(p2t).normalized * 100, Color.black);
        }

        /** Velocity Obstacle.
		 * This is a struct to avoid too many allocations.
		 *
		 * \see https://en.wikipedia.org/wiki/Velocity_obstacle
		 */
        internal struct VO
        {
            TSVector2 line1, line2, dir1, dir2;

            TSVector2 cutoffLine, cutoffDir;
            TSVector2 circleCenter;

            bool colliding;
            FP radius;
            FP weightFactor;
            FP weightBonus;

            TSVector2 segmentStart, segmentEnd;
            bool segment;

            /** Creates a VO for avoiding another agent.
			 * \param center The position of the other agent relative to this agent.
			 * \param offset Offset of the velocity obstacle. For example to account for the agents' relative velocities.
			 * \param radius Combined radius of the two agents (radius1 + radius2).
			 * \param inverseDt 1 divided by the local avoidance time horizon (e.g avoid agents that we will hit within the next 2 seconds).
			 * \param inverseDeltaTime 1 divided by the time step length.
			 */
            public VO(TSVector2 center, TSVector2 offset, FP radius, FP inverseDt, FP inverseDeltaTime)
            {
                // Adjusted so that a parameter weightFactor of 1 will be the default ("natural") weight factor
                this.weightFactor = 1;
                weightBonus = 0;

                //this.radius = radius;
                TSVector2 globalCenter;

                circleCenter = center * inverseDt + offset;
                FP tmp = Sqr(center.LengthSquared() / (radius * radius));//exp(-tmp),
               // tmp = 1 +tmp+tmp*tmp/2+tmp*tmp*tmp/6; //simple use Taylor's Formula
                this.weightFactor = 4 * System.Math.Exp(-tmp.AsFloat()) + 1;// 4 /tmp + 1;//exp()
                // Collision?
                if (center.magnitude < radius)
                {
                    colliding = true;

                    // 0.001 is there to make sure lin1.magnitude is not so small that the normalization
                    // below will return TSVector2.zero as that will make the VO invalid and it will be ignored.
                    line1 = center.normalized * (center.magnitude - radius - FP.One/1000) * 3/10 * inverseDeltaTime;
                    dir1 = new TSVector2(line1.y, -line1.x).normalized;
                    line1 += offset;

                    cutoffDir = TSVector2.zero;
                    cutoffLine = TSVector2.zero;
                    dir2 = TSVector2.zero;
                    line2 = TSVector2.zero;
                    this.radius = 0;
                }
                else
                {
                    colliding = false;

                    center *= inverseDt;
                    radius *= inverseDt;
                    globalCenter = center + offset;

                    // 0.001 is there to make sure cutoffDistance is not so small that the normalization
                    // below will return TSVector2.zero as that will make the VO invalid and it will be ignored.
                    var cutoffDistance = center.magnitude - radius + FP.One/1000;

                    cutoffLine = center.normalized * cutoffDistance;
                    cutoffDir = new TSVector2(-cutoffLine.y, cutoffLine.x).normalized;
                    cutoffLine += offset;

                    FP alpha = TSMath.Atan2(-center.y, -center.x);

                    FP delta = TSMath.Abs(TSMath.Acos(radius / center.magnitude));

                    this.radius = radius;

                    // Bounding Lines

                    // Point on circle
                    line1 = new TSVector2(TSMath.Cos(alpha + delta), TSMath.Sin(alpha + delta));
                    // Vector tangent to circle which is the correct line tangent
                    // Note that this vector is normalized
                    dir1 = new TSVector2(line1.y, -line1.x);

                    // Point on circle
                    line2 = new TSVector2(TSMath.Cos(alpha - delta), TSMath.Sin(alpha - delta));
                    // Vector tangent to circle which is the correct line tangent
                    // Note that this vector is normalized
                    dir2 = new TSVector2(line2.y, -line2.x);

                    line1 = line1 * radius + globalCenter;
                    line2 = line2 * radius + globalCenter;
                }

                segmentStart = TSVector2.zero;
                segmentEnd = TSVector2.zero;
                segment = false;
            }

            /** Complex number multiplication.
			 * Used to rotate vectors in an efficient way.
			 *
			 * \see https://en.wikipedia.org/wiki/Complex_number#Multiplication_and_division
			 */
            static TSVector2 ComplexMultiply(TSVector2 a, TSVector2 b)
            {
                return new TSVector2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x);
            }

            /** Creates a VO for avoiding another agent.
			 * Note that the segment is directed, the agent will want to be on the left side of the segment.
			 */
            public static VO SegmentObstacle(TSVector2 segmentStart, TSVector2 segmentEnd, TSVector2 offset, FP radius, FP inverseDt, FP inverseDeltaTime)
            {
                var vo = new VO();

                // Adjusted so that a parameter weightFactor of 1 will be the default ("natural") weight factor
                vo.weightFactor = 1;
                // Just higher than anything else
                vo.weightBonus = TSMath.Max(radius, 1) * 40;

                var closestOnSegment = CustomMath.ClosestPointOnSegment(segmentStart, segmentEnd, TSVector2.zero);

                // Collision?
                if (closestOnSegment.magnitude <= radius)
                {
                    vo.colliding = true;

                    vo.line1 = closestOnSegment.normalized * (closestOnSegment.magnitude - radius) * 3/10 * inverseDeltaTime;
                    vo.dir1 = new TSVector2(vo.line1.y, -vo.line1.x).normalized;
                    vo.line1 += offset;

                    vo.cutoffDir = TSVector2.zero;
                    vo.cutoffLine = TSVector2.zero;
                    vo.dir2 = TSVector2.zero;
                    vo.line2 = TSVector2.zero;
                    vo.radius = 0;

                    vo.segmentStart = TSVector2.zero;
                    vo.segmentEnd = TSVector2.zero;
                    vo.segment = false;
                }
                else
                {
                    vo.colliding = false;

                    segmentStart *= inverseDt;
                    segmentEnd *= inverseDt;
                    radius *= inverseDt;

                    var cutoffTangent = (segmentEnd - segmentStart).normalized;
                    vo.cutoffDir = cutoffTangent;
                    vo.cutoffLine = segmentStart + new TSVector2(-cutoffTangent.y, cutoffTangent.x) * radius;
                    vo.cutoffLine += offset;

                    // See documentation for details
                    // The call to Max is just to prevent floating point errors causing NaNs to appear
                    var startSqrMagnitude = segmentStart.LengthSquared();
                    var normal1 = -ComplexMultiply(segmentStart, new TSVector2(radius, TSMath.Sqrt(TSMath.Max(0, startSqrMagnitude - radius * radius)))) / startSqrMagnitude;
                    var endSqrMagnitude = segmentEnd.LengthSquared();
                    var normal2 = -ComplexMultiply(segmentEnd, new TSVector2(radius, -TSMath.Sqrt(TSMath.Max(0, endSqrMagnitude - radius * radius)))) / endSqrMagnitude;

                    vo.line1 = segmentStart + normal1 * radius + offset;
                    vo.line2 = segmentEnd + normal2 * radius + offset;

                    // Note that the normals are already normalized
                    vo.dir1 = new TSVector2(normal1.y, -normal1.x);
                    vo.dir2 = new TSVector2(normal2.y, -normal2.x);

                    vo.segmentStart = segmentStart;
                    vo.segmentEnd = segmentEnd;
                    vo.radius = radius;
                    vo.segment = true;
                }

                return vo;
            }

            /** Returns a negative number of if \a p lies on the left side of a line which with one point in \a a and has a tangent in the direction of \a dir.
			 * The number can be seen as the double signed area of the triangle {a, a+dir, p} multiplied by the length of \a dir.
			 * If dir.magnitude=1 this is also the distance from p to the line {a, a+dir}.
			 */
            public static FP SignedDistanceFromLine(TSVector2 a, TSVector2 dir, TSVector2 p)
            {
                return (p.x - a.x) * (dir.y) - (dir.x) * (p.y - a.y);
            }

            /** Gradient and value of the cost function of this VO.
			 * Very similar to the #Gradient method however the gradient
			 * and value have been scaled and tweaked slightly.
			 */
            public TSVector2 ScaledGradient(TSVector2 p, out FP weight)
            {
                var grad = Gradient(p, out weight);

                if (weight > 0)
                {
                    FP Scale = 2;
                    grad *= Scale * weightFactor;
                    weight *= Scale * weightFactor;
                    weight += 1 + weightBonus;
                }

                return grad;
            }

            /** Gradient and value of the cost function of this VO.
			 * The VO has a cost function which is 0 outside the VO
			 * and increases inside it as the point moves further into
			 * the VO.
			 *
			 * This is the negative gradient of that function as well as its
			 * value (the weight). The negative gradient points in the direction
			 * where the function decreases the fastest.
			 *
			 * The value of the function is the distance to the closest edge
			 * of the VO and the gradient is normalized.
			 */
            public TSVector2 Gradient(TSVector2 p, out FP weight)
            {
                if (colliding)
                {
                    // Calculate double signed area of the triangle consisting of the points
                    // {line1, line1+dir1, p}
                    FP l1 = SignedDistanceFromLine(line1, dir1, p);

                    // Serves as a check for which side of the line the point p is
                    if (l1 >= 0)
                    {
                        weight = l1;
                        return new TSVector2(-dir1.y, dir1.x);
                    }
                    else
                    {
                        weight = 0;
                        return new TSVector2(0, 0);
                    }
                }

                FP det3 = SignedDistanceFromLine(cutoffLine, cutoffDir, p);
                if (det3 <= 0)
                {
                    weight = 0;
                    return TSVector2.zero;
                }
                else
                {
                    // Signed distances to the two edges along the sides of the VO
                    FP det1 = SignedDistanceFromLine(line1, dir1, p);
                    FP det2 = SignedDistanceFromLine(line2, dir2, p);
                    if (det1 >= 0 && det2 >= 0)
                    {
                        // We are inside both of the half planes
                        // (all three if we count the cutoff line)
                        // and thus inside the forbidden region in velocity space

                        // Actually the negative gradient because we want the
                        // direction where it slopes the most downwards, not upwards
                        TSVector2 gradient;

                        // Check if we are in the semicircle region near the cap of the VO
                        if (TSVector2.Dot(p - line1, dir1) > 0 && TSVector2.Dot(p - line2, dir2) < 0)
                        {
                            if (segment)
                            {
                                // This part will only be reached for line obstacles (i.e not other agents)
                                if (det3 < radius)
                                {
                                    var closestPointOnLine = CustomMath.ClosestPointOnSegment(segmentStart, segmentEnd, p);
                                    var dirFromCenter = p - closestPointOnLine;
                                    FP distToCenter;
                                    gradient = CustomMath.VecNormalize(dirFromCenter, out distToCenter);
                                    // The weight is the distance to the edge
                                    weight = radius - distToCenter;
                                    return gradient;
                                }
                            }
                            else
                            {
                                var dirFromCenter = p - circleCenter;
                                FP distToCenter;
                                gradient = CustomMath.VecNormalize(dirFromCenter, out distToCenter);
                                // The weight is the distance to the edge
                                weight = radius - distToCenter;
                                return gradient;
                            }
                        }

                        if (segment && det3 < det1 && det3 < det2)
                        {
                            weight = det3;
                            gradient = new TSVector2(-cutoffDir.y, cutoffDir.x);
                            return gradient;
                        }

                        // Just move towards the closest edge
                        // The weight is the distance to the edge
                        if (det1 < det2)
                        {
                            weight = det1;
                            gradient = new TSVector2(-dir1.y, dir1.x);
                        }
                        else
                        {
                            weight = det2;
                            gradient = new TSVector2(-dir2.y, dir2.x);
                        }

                        return gradient;
                    }

                    weight = 0;
                    return TSVector2.zero;
                }
            }
        }

        /** Very simple list.
		 * Cannot use a List<T> because when indexing into a List<T> and T is
		 * a struct (which VO is) then the whole struct will be copied.
		 * When indexing into an array, that copy can be skipped.
		 */
        internal class VOBuffer
        {
            public VO[] buffer;
            public int length;

            public void Clear()
            {
                length = 0;
            }

            public VOBuffer(int n)
            {
                buffer = new VO[n];
                length = 0;
            }

            public void Add(VO vo)
            {
                if (length >= buffer.Length)
                {
                    var nbuffer = new VO[buffer.Length * 2];
                    buffer.CopyTo(nbuffer, 0);
                    buffer = nbuffer;
                }
                buffer[length++] = vo;
            }
        }

        internal void CalculateVelocity(WorkerContext context, FP deltaTime)
        {
            if (manuallyControlled)
            {
                return;
            }

            if (locked)
            {
                calculatedSpeed = 0;
                calculatedTargetPoint = position;
                return;
            }

            // Buffer which will be filled up with velocity obstacles (VOs)
            var vos = context.vos;
            vos.Clear();

            //GenerateObstacleVOs(vos);
#if USING_RVO
            GenerateNeighbourAgentVOs(vos,deltaTime);
#endif
            bool insideAnyVO = BiasDesiredVelocity(vos, ref desiredVelocity, ref desiredTargetPointInVelocitySpace, FP.One / 10);

            if (!insideAnyVO)
            {
                // Desired velocity can be used directly since it was not inside any velocity obstacle.
                // No need to run optimizer because this will be the global minima.
                // This is also a special case in which we can set the
                // calculated target point to the desired target point
                // instead of calculating a point based on a calculated velocity
                // which is an important difference when the agent is very close
                // to the target point
                // TODO: Not actually guaranteed to be global minima if desiredTargetPointInVelocitySpace.magnitude < desiredSpeed
                // maybe do something different here?
                calculatedTargetPoint = desiredTargetPointInVelocitySpace + position;
                calculatedSpeed = desiredSpeed;
           //     if (DebugDraw) Draw.Debug.CrossXZ(FromXZ(calculatedTargetPoint), Color.white);
                return;
            }

            TSVector2 result = TSVector2.zero;

            result = GradientDescent(vos, currentVelocity, desiredVelocity);

          //  if (DebugDraw) Draw.Debug.CrossXZ(FromXZ(result + position), Color.white);
            //Debug.DrawRay (To3D (position), To3D (result));

            calculatedTargetPoint = position + result;
            calculatedSpeed = TSMath.Min(result.magnitude, maxSpeed);
        }
#if UNITY_EDITOR&& PATHMANAGER_DEBUG
        static UnityEngine.Color Rainbow(FP v)
        {
            UnityEngine.Color c = new UnityEngine.Color(v.AsFloat(), 0, 0);

            if (c.r > 1) { c.g = c.r - 1; c.r = 1; }
            if (c.g > 1) { c.b = c.g - 1; c.g = 1; }
            return c;
        }
#endif

        //void GenerateObstacleVOs(VOBuffer vos)
        //{
        //    var range = maxSpeed * obstacleTimeHorizon;

        //    // Iterate through all obstacles that we might need to avoid
        //    for (int i = 0; i < simulator.obstacles.Count; i++)
        //    {
        //        var obstacle = simulator.obstacles[i];
        //        var vertex = obstacle;
        //        // Iterate through all edges (defined by vertex and vertex.dir) in the obstacle
        //        do
        //        {
        //            // Ignore the edge if the agent should not collide with it
        //            if (vertex.ignore || (vertex.layer & collidesWith) == 0)
        //            {
        //                vertex = vertex.next;
        //                continue;
        //            }

        //            // Start and end points of the current segment
        //            FP elevation1, elevation2;
        //            var p1 = To2D(vertex.position, out elevation1);
        //            var p2 = To2D(vertex.next.position, out elevation2);

        //            TSVector2 dir = (p2 - p1).normalized;

        //            // Signed distance from the line (not segment, lines are infinite)
        //            // TODO: Can be optimized
        //            FP dist = VO.SignedDistanceFromLine(p1, dir, position);

        //            if (dist >= -FP.One/100 && dist < range)
        //            {
        //                FP factorAlongSegment = TSVector2.Dot(position - p1, p2 - p1) / (p2 - p1).sqrMagnitude;

        //                // Calculate the elevation (y) coordinate of the point on the segment closest to the agent
        //                var segmentY = TSMath.Lerp(elevation1, elevation2, factorAlongSegment);

        //                // Calculate distance from the segment (not line)
        //                var sqrDistToSegment = (TSVector2.Lerp(p1, p2, factorAlongSegment) - position).sqrMagnitude;

        //                // Ignore the segment if it is too far away
        //                // or the agent is too high up (or too far down) on the elevation axis (usually y axis) to avoid it.
        //                // If the XY plane is used then all elevation checks are disabled
        //                if (sqrDistToSegment < range * range && (simulator.movementPlane == MovementPlane.XY || (elevationCoordinate <= segmentY + vertex.height && elevationCoordinate + height >= segmentY)))
        //                {
        //                    vos.Add(VO.SegmentObstacle(p2 - position, p1 - position, TSVector2.zero, radius * FP.One/100, 1 / ObstacleTimeHorizon, 1 / simulator.DeltaTime));
        //                }
        //            }

        //            vertex = vertex.next;
        //        } while (vertex != obstacle && vertex != null && vertex.next != null);
        //    }
        //}
#if  USING_RVO
        void GenerateNeighbourAgentVOs(VOBuffer vos, FP deltaTime)
        {
            FP inverseAgentTimeHorizon = 1/ agentTimeHorizon;

            // The RVO algorithm assumes we will continue to
            // move in roughly the same direction
            TSVector2 optimalVelocity = currentVelocity;
            int count = _behaviour.neighbours.Count;
            for (int o = 0; o < count; o++)
            {
                IAgentBehaviour other = _behaviour.neighbours[o];

                // Don't avoid ourselves
                if (other == this)
                    continue;

                // Interval along the y axis in which the agents overlap
                FP maxY = TSMath.Min(elevationCoordinate + height, other.OrcaAgent.elevationCoordinate + other.OrcaAgent.height);
                FP minY = TSMath.Max(elevationCoordinate, other.OrcaAgent.elevationCoordinate);

                // The agents cannot collide since they are on different y-levels
                if (maxY - minY < 0)
                {
                    continue;
                }

                FP totalRadius = radius + other.colliderRadius;

                // Describes a circle on the border of the VO
                TSVector2 voBoundingOrigin = CustomMath.TSVecToVec2(other.position - _behaviour.position);

                FP avoidanceStrength;
                if (other.OrcaAgent.locked || other.OrcaAgent.manuallyControlled)
                {
                    avoidanceStrength = 1;
                }
                else if (other.OrcaAgent.Priority > CustomMath.EPSILON || Priority > CustomMath.EPSILON)
                {
                    avoidanceStrength = other.OrcaAgent.Priority / (Priority + other.OrcaAgent.Priority);
                }
                else
                {
                    // Both this agent's priority and the other agent's priority is zero or negative
                    // Assume they have the same priority
                    avoidanceStrength = CustomMath.FPHalf;
                }

                // We assume that the other agent will continue to move with roughly the same velocity if the priorities for the agents are similar.
                // If the other agent has a higher priority than this agent (avoidanceStrength > 0.5) then we will assume it will move more along its
                // desired velocity. This will have the effect of other agents trying to clear a path for where a high priority agent wants to go.
                // If this is not done then even high priority agents can get stuck when it is really crowded and they have had to slow down.
                TSVector2 otherOptimalVelocity = TSVector2.Lerp(other.OrcaAgent.currentVelocity, other.OrcaAgent.desiredVelocity, 2 * avoidanceStrength - 1);

                var voCenter = TSVector2.Lerp(optimalVelocity, otherOptimalVelocity, avoidanceStrength);

                vos.Add(new VO(voBoundingOrigin, voCenter, totalRadius, inverseAgentTimeHorizon, 1 / deltaTime));
                if (DebugDraw)
                    DrawVO(position + voBoundingOrigin * inverseAgentTimeHorizon + voCenter, totalRadius * inverseAgentTimeHorizon, position + voCenter);
            }
        }
#endif
        TSVector2 GradientDescent(VOBuffer vos, TSVector2 sampleAround1, TSVector2 sampleAround2)
        {
            FP score1;
            var minima1 = Trace(vos, sampleAround1, out score1);

          //  if (DebugDraw) Draw.Debug.CrossXZ(FromXZ(minima1 + position), Color.yellow, 0.5f);

            // Can be uncommented for higher quality local avoidance
            // for ( int i = 0; i < 3; i++ ) {
            //	TSVector2 p = desiredVelocity + new TSVector2(TSMath.Cos(TSMath.PI*2*(i/3)), TSMath.Sin(TSMath.PI*2*(i/3)));
            //	FP score;TSVector2 res = Trace ( vos, p, velocity.magnitude*simulator.qualityCutoff, out score );
            //
            //	if ( score < best ) {
            //		result = res;
            //		best = score;
            //	}
            // }

            FP score2;
            TSVector2 minima2 = Trace(vos, sampleAround2, out score2);
          //  if (DebugDraw) Draw.Debug.CrossXZ(FromXZ(minima2 + position), Color.magenta, 0.5f);

            return score1 < score2 ? minima1 : minima2;
        }


        /** Bias towards the right side of agents.
		 * Rotate desiredVelocity at most [value] number of radians. 1 radian ≈ 57°
		 * This breaks up symmetries.
		 *
		 * The desired velocity will only be rotated if it is inside a velocity obstacle (VO).
		 * If it is inside one, it will not be rotated further than to the edge of it
		 *
		 * The targetPointInVelocitySpace will be rotated by the same amount as the desired velocity
		 *
		 * \returns True if the desired velocity was inside any VO
		 */
        static bool BiasDesiredVelocity(VOBuffer vos, ref TSVector2 desiredVelocity, ref TSVector2 targetPointInVelocitySpace, FP maxBiasRadians)
        {
            var desiredVelocityMagn = desiredVelocity.magnitude;
            FP maxValue = FP.Zero;

            for (int i = 0; i < vos.length; i++)
            {
                FP value;
                // The value is approximately the distance to the edge of the VO
                // so taking the maximum will give us the distance to the edge of the VO
                // which the desired velocity is furthest inside
                vos.buffer[i].Gradient(desiredVelocity, out value);
                maxValue = TSMath.Max(maxValue, value);
            }

            // Check if the agent was inside any VO
            var inside = maxValue > 0;

            // Avoid division by zero below
            if (desiredVelocityMagn < FP.One / 1000)
            {
                return inside;
            }

            // Rotate the desired velocity clockwise (to the right) at most maxBiasRadians number of radians
            // Assuming maxBiasRadians is small, we can just move it instead and it will give approximately the same effect
            // See https://en.wikipedia.org/wiki/Small-angle_approximation
            var angle = TSMath.Min(maxBiasRadians, maxValue / desiredVelocityMagn);
            desiredVelocity += new TSVector2(desiredVelocity.y, -desiredVelocity.x) * angle;
            targetPointInVelocitySpace += new TSVector2(targetPointInVelocitySpace.y, -targetPointInVelocitySpace.x) * angle;
            return inside;
        }

        /** Evaluate gradient and value of the cost function at velocity p */
        TSVector2 EvaluateGradient(VOBuffer vos, TSVector2 p, out FP value)
        {
            TSVector2 gradient = TSVector2.zero;

            value = 0;

            // Avoid other agents
            for (int i = 0; i < vos.length; i++)
            {
                FP w;
                var grad = vos.buffer[i].ScaledGradient(p, out w);
                if (w > value)
                {
                    value = w;
                    gradient = grad;
                }
            }

            // Move closer to the desired velocity
            var dirToDesiredVelocity = desiredVelocity - p;
            var distToDesiredVelocity = dirToDesiredVelocity.magnitude;
            if (distToDesiredVelocity > CustomMath.EPSILON)
            {
                gradient += dirToDesiredVelocity * (DesiredVelocityWeight / distToDesiredVelocity);
                value += distToDesiredVelocity * DesiredVelocityWeight;
            }

            // Prefer speeds lower or equal to the desired speed
            // and avoid speeds greater than the max speed
            var sqrSpeed = p.LengthSquared();
            if (sqrSpeed > desiredSpeed * desiredSpeed)
            {
                var speed = TSMath.Sqrt(sqrSpeed);

                if (speed > maxSpeed)
                {
                    FP MaxSpeedWeight = 3;
                    value += MaxSpeedWeight * (speed - maxSpeed);
                    gradient -= MaxSpeedWeight * (p / speed);
                }

                // Scale needs to be strictly greater than DesiredVelocityWeight
                // otherwise the agent will not prefer the desired speed over
                // the maximum speed
                FP scale = 2 * DesiredVelocityWeight;
                value += scale * (speed - desiredSpeed);
                gradient -= scale * (p / speed);
            }

            return gradient;
        }

        /** Traces the vector field constructed out of the velocity obstacles.
		 * Returns the position which gives the minimum score (approximately).
		 *
		 * \see https://en.wikipedia.org/wiki/Gradient_descent
		 */
        TSVector2 Trace(VOBuffer vos, TSVector2 p, out FP score)
        {
            // Pick a reasonable initial step size
            FP stepSize = TSMath.Max(radius, FP.One / 5 * desiredSpeed);

            FP bestScore = FP.MaxValue;
            TSVector2 bestP = p;

            // TODO: Add momentum to speed up convergence?

            const int MaxIterations = 50;

            for (int s = 0; s < MaxIterations; s++)
            {
                FP step = 1 - (s*FP.One /MaxIterations);
                step = Sqr(step) * stepSize;

                FP value;
                var gradient = EvaluateGradient(vos, p, out value);

                if (value < bestScore)
                {
                    bestScore = value;
                    bestP = p;
                }

                // TODO: Add cutoff for performance
                gradient.Normalize();
                gradient *= step;
                TSVector2 prev = p;
                p += gradient;

               // if (DebugDraw) Debug.DrawLine(FromXZ(prev + position), FromXZ(p + position), Rainbow(s * 0.1f) * new Color(1, 1, 1, 1f));
            }

            score = bestScore;
            return bestP;
        }
        public TSVector2 CalculateMovementDelta(TSVector2 prePos, FP deltaTime)
        {
            return TSVector2.ClampMagnitude(CalculatedTargetPoint - prePos, CalculatedSpeed * deltaTime);
        }
        public void ClearData()
        {
            desiredVelocity = currentVelocity = TSVector2.zero;
        }
    }
}
