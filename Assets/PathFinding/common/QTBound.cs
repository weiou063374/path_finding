///author:huwei
///date:2018.4.5
//using UnityEngine;
using System.Collections;
using System;
using TrueSync;
namespace PathFinding
{
    public struct QTBound
    {
        public QTBound(TSVector _min, TSVector _max)
        {
            min = _min;
            max = _max;
        }
        public static QTBound zero = new QTBound(TSVector.zero, TSVector.zero);
        public static  QTBound MinMaxQTBound(TSVector _min, TSVector _max)
        {
            QTBound bound = zero;
            bound.min = _min;
            bound.max = _max;
           // bound.center
            return bound;
        }
        public static QTBound MinMaxQTBound(int minX, int minY, int maxX, int maxY)
        {
            QTBound bound = zero;
            TSVector min = TSVector.zero;
            min.x = minX;
            min.z = minY;

            TSVector max = TSVector.zero;
            max.x = maxX;
            max.z = maxY;

            bound.min = min;
            bound.max = max;
            // bound.center
            return bound;
        }
        public static bool operator ==(QTBound lhs, QTBound rhs)
        {
            return ((lhs.min == rhs.min) && (lhs.max == rhs.max));
        }

        public static bool operator !=(QTBound lhs, QTBound rhs)
        {
            return !(lhs == rhs);
        }

		public TSVector max;
		public TSVector min;

        public TSVector center
        {
            get { return (max + min)* CustomMath.FPHalf; }
        }

        public FP ComputeSquaredDistanceToPoint(TSVector Point)
        {
            // 
            FP DistSquared = FP.Zero;

            if (Point.x < min.x)
            {
                DistSquared += TSMath.Sqrt(Point.x - min.x);
            }
            else if (Point.x > max.x)
            {
                DistSquared += TSMath.Sqrt(Point.x - max.x);
            }

            if (Point.z< min.z)
            {
                DistSquared += TSMath.Sqrt(Point.z- min.z);
            }
            else if (Point.z> max.z)
            {
                DistSquared += TSMath.Sqrt(Point.z- max.z);
            }

            return DistSquared;
        }

        public QTBound ExpandBy(FP W)
        {
            TSVector vec =TSVector.zero ;
            vec.Set(W,0,W);
            min = min - vec;
            max = max + vec;
            return this;
        }

        public FP GetArea()
        {
            return (max.x - min.x) * (max.z - min.z);
        }

        public TSVector GetExtent()
        {
            return  (max - min)*CustomMath.FPHalf;
        }

        public TSVector GetSize()
        {
            return max - min;
        }

        public bool IsInside(TSVector TestPoint)
        {
            return ((TestPoint.x > min.x) && (TestPoint.x < max.x) && (TestPoint.z > min.z) && (TestPoint.z < max.z));
        }

        public bool IsInside(QTBound Other)
        {
            return (IsInside(Other.min) && IsInside(Other.max));
        }

        public TSVector GetClosestPointTo(TSVector Point)
        {
            // 
            TSVector ClosestPoint = Point;

            //clamp
            if (Point.x < min.x)
            {
                ClosestPoint.x = min.x;
            }
            else if (Point.x > max.x)
            {
                ClosestPoint.x = max.x;
            }

            // clamp 
            if (Point.z< min.z)
            {
                ClosestPoint.z= min.z;
            }
            else if (Point.z> max.z)
            {
                ClosestPoint.z= max.z;
            }

            return ClosestPoint;
        }


        public bool Intersect(QTBound Other)
        {
            if ((min.x > Other.max.x) || (Other.min.x > max.x))
            {
                return false;
            }

            if ((min.z> Other.max.z) || (Other.min.z> max.z))
            {
                return false;
            }

            return true;
        }

        public override bool Equals(object other)
        {
            QTBound bound = (QTBound)other;
            
            return bound == this;
        }
        public override int GetHashCode()
        {
            return (min.GetHashCode() + max.GetHashCode()).GetHashCode();
        }

    }
}
