/*
date:2018.4.25
 */

using System;
using System.Collections.Generic;
using TrueSync;
namespace PathFinding
{
    public class ORCAAgent:IResetable
    {
        public IAgentBehaviour _behaviour;
       // internal IList<KeyValuePair<FP, ORCAAgent>> ORCAAgentNeighbors_ = new List<KeyValuePair<FP, ORCAAgent>>();
        internal List<FP> obstacleSqrDst = new List<FP>();
        internal List<Obstacle> obstacleNeighbors_ = new List<Obstacle>();

        internal IList<Line> orcaLines_ = new List<Line>();
        //internal TSVector2 position_;
        internal TSVector2 prefVelocity_;
        internal TSVector2 velocity_;
        //internal int id_ = 0;
        //internal int maxNeighbors_ = 0;
        //internal FP maxSpeed_ = FP.Zero;
       // internal FP neighborDist_ = FP.Zero;
        //internal FP radius_ = FP.Zero;
        static internal FP timeHorizon_ = CustomMath.FPHalf/2;
        static internal FP timeHorizonObst_ = CustomMath.FPHalf/2;

        private TSVector2 newVelocity_;

        internal void computeNeighbours()
        {
            
        }
        internal void caculateObstacleNeighbours()
        {
            FP radius_ = _behaviour.colliderRadius;
            TSVector2 position_ = CustomMath.TSVecToVec2(_behaviour.position);
            FP invTimeHorizonObst = FP.One / timeHorizonObst_;
            /* Create obstacle ORCA lines. */
            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {

                Obstacle obstacle1 = obstacleNeighbors_[i];
                Obstacle obstacle2 = obstacle1.next_;

                TSVector2 relativePosition1 = obstacle1.point_ - position_;
                TSVector2 relativePosition2 = obstacle2.point_ - position_;

                /*
                 * Check if velocity obstacle of obstacle is already taken care
                 * of by previously constructed obstacle ORCA lines.
                 */
                bool alreadyCovered = false;

                for (int j = 0; j < orcaLines_.Count; ++j)
                {
                    if (CustomMath.det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * _behaviour.colliderRadius >= -CustomMath.EPSILON && CustomMath.det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * _behaviour.colliderRadius >= -CustomMath.EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                /* Not yet covered. Check for collisions. */
                FP distSq1 = (relativePosition1).LengthSquared();
                FP distSq2 = (relativePosition2).LengthSquared();

                FP radiusSq = CustomMath.sqr(_behaviour.colliderRadius);

                TSVector2 obstacleVector = obstacle2.point_ - obstacle1.point_;
                FP s = (-relativePosition1 * obstacleVector) / obstacleVector.LengthSquared();
                FP distSqLine = (-relativePosition1 - s * obstacleVector).LengthSquared();

                Line line;

                if (s < FP.Zero && distSq1 <= radiusSq)
                {
                    /* Collision with left vertex. Ignore if non-convex. */
                    if (obstacle1.convex_)
                    {
                        line.point = TSVector2.zero;
                        TSVector2 vec = TSVector2.zero;
                        vec.x = -relativePosition1.y;
                        vec.y = relativePosition1.x;
                        line.direction =vec.normalized;
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s > FP.One && distSq2 <= radiusSq)
                {
                    /*
                     * Collision with right vertex. Ignore if non-convex or if
                     * it will be taken care of by neighboring obstacle.
                     */
                    if (obstacle2.convex_ && CustomMath.det(relativePosition2, obstacle2.direction_) >= FP.Zero)
                    {
                        line.point = TSVector2.zero;
                        TSVector2 vec= TSVector2.zero;
                        vec.x = -relativePosition2.y;
                        vec.y = relativePosition2.x;
                        line.direction = vec.normalized;
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s >= FP.Zero && s < FP.One && distSqLine <= radiusSq)
                {
                    /* Collision with obstacle segment. */
                    line.point = TSVector2.zero;
                    line.direction = -obstacle1.direction_;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * No collision. Compute legs. When obliquely viewed, both legs
                 * can come from a single vertex. Legs extend cut-off line when
                 * non-convex vertex.
                 */

                TSVector2 leftLegDirection, rightLegDirection;

                if (s < FP.Zero && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that left vertex
                     * defines velocity obstacle.
                     */
                    if (!obstacle1.convex_)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle2 = obstacle1;

                    FP leg1 = CustomMath.sqrt(distSq1 - radiusSq);

                    TSVector2 vec = TSVector2.zero;
                    vec.x = relativePosition1.x * leg1 - relativePosition1.y * radius_;
                    vec.y = relativePosition1.x * radius_ + relativePosition1.y * leg1;
                    leftLegDirection = vec / distSq1;
                    vec.x = relativePosition1.x * leg1 + relativePosition1.y * radius_;
                    vec.y = -relativePosition1.x * radius_ + relativePosition1.y * leg1;
                    rightLegDirection = vec / distSq1;
                }
                else if (s > FP.One && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that
                     * right vertex defines velocity obstacle.
                     */
                    if (!obstacle2.convex_)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle1 = obstacle2;

                    FP leg2 = CustomMath.sqrt(distSq2 - radiusSq);
                    TSVector2 vec = TSVector2.zero;
                    vec.x = relativePosition2.x * leg2 - relativePosition2.y * radius_;
                    vec.y = relativePosition2.x * radius_ + relativePosition2.y * leg2;
                    leftLegDirection = vec / distSq2;
                    vec.x = relativePosition2.x * leg2 + relativePosition2.y * radius_;
                    vec.y = -relativePosition2.x * radius_ + relativePosition2.y * leg2;
                    rightLegDirection =vec / distSq2;
                }
                else
                {
                    /* Usual situation. */
                    if (obstacle1.convex_)
                    {
                        FP leg1 = CustomMath.sqrt(distSq1 - radiusSq);
                        TSVector2 vec = TSVector2.zero;
                        vec.x = relativePosition1.x * leg1 - relativePosition1.y * radius_;
                        vec.y = relativePosition1.x * radius_ + relativePosition1.y * leg1;
                        leftLegDirection = vec/ distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -obstacle1.direction_;
                    }

                    if (obstacle2.convex_)
                    {
                        FP leg2 = CustomMath.sqrt(distSq2 - radiusSq);
                        TSVector2 vec = TSVector2.zero;
                        vec.x = relativePosition2.x * leg2 + relativePosition2.y * radius_;
                        vec.y = -relativePosition2.x * radius_ + relativePosition2.y * leg2;
                        rightLegDirection =vec/ distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = obstacle1.direction_;
                    }
                }

                /*
                 * Legs can never point into neighboring edge when convex
                 * vertex, take cutoff-line of neighboring edge instead. If
                 * velocity projected on "foreign" leg, no constraint is added.
                 */

                Obstacle leftNeighbor = obstacle1.previous_;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1.convex_ && CustomMath.det(leftLegDirection, -leftNeighbor.direction_) >= FP.Zero)
                {
                    /* Left leg points into obstacle. */
                    leftLegDirection = -leftNeighbor.direction_;
                    isLeftLegForeign = true;
                }

                if (obstacle2.convex_ && CustomMath.det(rightLegDirection, obstacle2.direction_) <= FP.Zero)
                {
                    /* Right leg points into obstacle. */
                    rightLegDirection = obstacle2.direction_;
                    isRightLegForeign = true;
                }

                /* Compute cut-off centers. */
                TSVector2 leftCutOff = invTimeHorizonObst * (obstacle1.point_ - position_);
                TSVector2 rightCutOff = invTimeHorizonObst * (obstacle2.point_ - position_);
                TSVector2 cutOffVector = rightCutOff - leftCutOff;

                /* Project current velocity on velocity obstacle. */

                /* Check if current velocity is projected on cutoff circles. */
                FP t = obstacle1 == obstacle2 ? CustomMath.FPHalf : ((velocity_ - leftCutOff) * cutOffVector) / cutOffVector.LengthSquared();
                FP tLeft = (velocity_ - leftCutOff) * leftLegDirection;
                FP tRight = (velocity_ - rightCutOff) * rightLegDirection;

                if ((t < FP.Zero && tLeft < FP.Zero) || (obstacle1 == obstacle2 && tLeft < FP.Zero && tRight < FP.Zero))
                {
                    /* Project on left cut-off circle. */
                    TSVector2 unitW =(velocity_ - leftCutOff).normalized;

                    TSVector2 vec = TSVector2.zero;
                    vec.x = unitW.y;
                    vec.y = -unitW.x;
                    line.direction = vec;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }
                else if (t > FP.One && tRight < FP.Zero)
                {
                    /* Project on right cut-off circle. */
                    TSVector2 unitW =(velocity_ - rightCutOff).normalized;

                    TSVector2 vec = TSVector2.zero;
                    vec.x = unitW.y;
                    vec.y = -unitW.x;
                    line.direction = vec;
                    line.point = rightCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * Project on left leg, right leg, or cut-off line, whichever is
                 * closest to velocity.
                 */
                FP distSqCutoff = (t < FP.Zero || t > FP.One || obstacle1 == obstacle2) ? FP.PositiveInfinity : (velocity_ - (leftCutOff + t * cutOffVector)).LengthSquared();
                FP distSqLeft = tLeft < FP.Zero ? FP.PositiveInfinity : (velocity_ - (leftCutOff + tLeft * leftLegDirection)).LengthSquared();
                FP distSqRight = tRight < FP.Zero ? FP.PositiveInfinity : (velocity_ - (rightCutOff + tRight * rightLegDirection)).LengthSquared();

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* Project on cut-off line. */
                    line.direction = -obstacle1.direction_;
                    TSVector2 vec = TSVector2.zero;
                    vec.x = -line.direction.y;
                    vec.y = line.direction.x;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * vec;
                    orcaLines_.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* Project on left leg. */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    TSVector2 vec = TSVector2.zero;
                    vec.x = -line.direction.y;
                    vec.y = line.direction.x;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * vec;
                    orcaLines_.Add(line);

                    continue;
                }

                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                TSVector2 vec1 = TSVector2.zero;
                vec1.x = -line.direction.y;
                vec1.y = line.direction.x;
                line.point = rightCutOff + radius_ * invTimeHorizonObst * vec1;
                orcaLines_.Add(line);
            }
        }
        readonly FP static_radiusFacotr = FP.One *8/ 10;
        /**
         * Computes the new velocity of this ORCAAgent.
         */
        internal void computeNewVelocity(FP frameTime)
        {
            if(_behaviour==null)
            {
                return;
            }
          
            orcaLines_.Clear();

            FP invTimeHorizonObst = FP.One / timeHorizonObst_;

            //caculateObstacleNeighbours();//todo

            int numObstLines = orcaLines_.Count;

            FP invTimeHorizon = FP.One / timeHorizon_;

            /* Create ORCAAgent ORCA lines. */
            List<IAgentBehaviour> neighbours = _behaviour.neighbours;
            int icount = neighbours.Count;
            //int extraIdx = -1;
            TSVector extrPos = TSVector.zero;
            for (int i = 0; i < icount; ++i)
            {
                IAgentBehaviour other = neighbours[i];
#if USING_ORCA
                if (other.ACAgent==null)//|| other.velocity==TSVector.zero || other.agent == null 
                {
                    continue;
                }
#endif
                TSVector otherPos = other.position;
                TSVector otherVelocity = other.velocity;
                if (other.agent==null)//&& PathFindingManager.StopToSpecialPos  check static agent wheter on SpecPos or not
                {
                    //if( extraIdx == -1)
                    //{
                    //    extrPos = other.map.GetWorldPosition(other.map.GetGridNodeId(other.position));
                    //   if (extrPos != other.position)
                    //   {
                    //        extraIdx = i;
                    //        i = i - 1;
                    //   }
                    //}
                    //else
                    {
                        extrPos = other.map.GetWorldPosition(other.map.GetGridNodeId(other.position));
                        otherPos = extrPos;
                        otherVelocity = TSVector.zero;
                       // extraIdx = -1;
                    }
                }
               
                TSVector2 relativePosition =CustomMath.TSVecToVec2(otherPos - _behaviour.position);
                TSVector2 relativeVelocity =  CustomMath.TSVecToVec2(_behaviour.velocity- otherVelocity);// velocity_ - other.OrcaAgent.velocity_;
                FP distSq =relativePosition.LengthSquared();
                FP combinedRadius = _behaviour.colliderRadius + other.colliderRadius;
                if (other.agent == null)
                {
                    combinedRadius *= static_radiusFacotr;
                }
                FP combinedRadiusSq = CustomMath.sqr(combinedRadius);

                Line line;
                TSVector2 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    TSVector2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    FP wLengthSq = w.LengthSquared();
                    FP dotProduct1 = w * relativePosition;

                    if (dotProduct1 < FP.Zero && CustomMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        FP wLength = CustomMath.sqrt(wLengthSq);
                        TSVector2 unitW = w / wLength;

                        TSVector2 vec = TSVector2.zero;
                        vec.x = unitW.y;
                        vec.y = -unitW.x;
                        line.direction = vec;
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on legs. */
                        FP leg = CustomMath.sqrt(distSq - combinedRadiusSq);

                        if (CustomMath.det(relativePosition, w) > FP.Zero)
                        {
                            /* Project on left leg. */
                            TSVector2 vec = TSVector2.zero;
                            vec.x = relativePosition.x * leg - relativePosition.y * combinedRadius;
                            vec.y = relativePosition.x * combinedRadius + relativePosition.y * leg;
                            line.direction = vec / distSq;
                        }
                        else
                        {
                            /* Project on right leg. */
                            TSVector2 vec = TSVector2.zero;
                            vec.x = relativePosition.x * leg + relativePosition.y * combinedRadius;
                            vec.y = -relativePosition.x * combinedRadius + relativePosition.y * leg;
                            line.direction = -vec / distSq;
                        }

                        FP dotProduct2 = relativeVelocity * line.direction;
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    FP invTimeStep = CustomMath.invTimeStep;

                    /* Vector from cutoff center to relative velocity. */
                    TSVector2 w = relativeVelocity - invTimeStep * relativePosition;

                    FP wLength = CustomMath.abs(w);
                    TSVector2 unitW = w / wLength;

                    TSVector2 vec = TSVector2.zero;
                    vec.x = unitW.y;
                    vec.y = -unitW.x;
                    line.direction = vec;
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.point = velocity_ + CustomMath.FPHalf * u;
                orcaLines_.Add(line);
            }

            int lineFail = linearProgram2(orcaLines_, _behaviour.maxSpeed, prefVelocity_, false, ref newVelocity_);

            if (lineFail < orcaLines_.Count)
            {
                linearProgram3(orcaLines_, numObstLines, lineFail, _behaviour.maxSpeed, ref newVelocity_);
            }            
        }

        internal void insertObstacleNeighbor(Obstacle obstacle, FP rangeSq)
        {
            TSVector2 position_ = CustomMath.TSVecToVec2(_behaviour.position);
            Obstacle nextObstacle = obstacle.next_;

            FP distSq = CustomMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position_);

            if (distSq < rangeSq)
            {
                obstacleSqrDst.Add(distSq);
                obstacleNeighbors_.Add( obstacle);

                int i = obstacleNeighbors_.Count - 1;

                while (i != 0 && distSq < obstacleSqrDst[i - 1])
                {
                    obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                    --i;
                }
                obstacleSqrDst[i] = distSq;
                obstacleNeighbors_[i] =  obstacle;
            }
        }
        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this ORCAAgent.</summary>
         */
        internal void Loop(FP frameTime)//TSVector2 velocity,
        {
           // prefVelocity_ = velocity;
            computeNewVelocity(frameTime);
            velocity_ = newVelocity_;
           // position_ += velocity_ * frameTime;
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private bool linearProgram1(IList<Line> lines, int lineNo, FP radius, TSVector2 optVelocity, bool directionOpt, ref TSVector2 result)
        {
            FP dotProduct = lines[lineNo].point * lines[lineNo].direction;
            FP discriminant = CustomMath.sqr(dotProduct) + CustomMath.sqr(radius) - (lines[lineNo].point).LengthSquared();

            if (discriminant < FP.Zero)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            FP sqrtDiscriminant = CustomMath.sqrt(discriminant);
            FP tLeft = -dotProduct - sqrtDiscriminant;
            FP tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                FP denominator = CustomMath.det(lines[lineNo].direction, lines[i].direction);
                FP numerator = CustomMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (CustomMath.fabs(denominator) <= CustomMath.EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < FP.Zero)
                    {
                        return false;
                    }

                    continue;
                }

                FP t = numerator / denominator;

                if (denominator >= FP.Zero)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = TSMath.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = TSMath.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (optVelocity * lines[lineNo].direction > FP.Zero)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                FP t = lines[lineNo].direction * (optVelocity - lines[lineNo].point);

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private int linearProgram2(IList<Line> lines, FP radius, TSVector2 optVelocity, bool directionOpt, ref TSVector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (optVelocity.LengthSquared() > CustomMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = optVelocity.normalized * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (CustomMath.det(lines[i].direction, lines[i].point - result) > FP.Zero)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    TSVector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }
        //for performance temp value
        List<Line> _projLines = new List<Line>();
        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, FP radius, ref TSVector2 result)
        {
            FP distance = FP.Zero;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (CustomMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    _projLines.Clear();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        _projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        FP determinant = CustomMath.det(lines[i].direction, lines[j].direction);

                        if (CustomMath.fabs(determinant) <= CustomMath.EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (lines[i].direction * lines[j].direction > FP.Zero)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = CustomMath.FPHalf * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (CustomMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction =(lines[j].direction - lines[i].direction).normalized;
                        _projLines.Add(line);
                    }

                    TSVector2 tempResult = result;
                    TSVector2 vec = TSVector2.zero;
                    vec.x = -lines[i].direction.y;
                    vec.y = lines[i].direction.x;
                    if (linearProgram2(_projLines, radius,vec, true, ref result) < _projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * FPing point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = CustomMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }

        public void Reset()
        {
            _behaviour = null;
            ClearData();
        }
        public void ClearData()
        {
            prefVelocity_ = newVelocity_ = velocity_ = TSVector2.zero;
        }
    }
}
