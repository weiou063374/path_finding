using TrueSync;
namespace PathFinding
{
    public struct RayCastOutput
    {
        public TSVector2 normal;
        public FP fraction;
    };
    public struct RayCastInput
    {
        public TSVector2 p1;
        public TSVector2 p2;
        public FP maxFraction;
    };
    public struct AABB
    {
        public TSVector2 lowerBound;  ///< the lower vertex
        public TSVector2 upperBound;    ///< the upper vertex
                                        /// Verify that the bounds are sorted.
        public bool IsValid()//warning
        {
            TSVector2 d = upperBound - lowerBound;
            bool valid = d.x >= FP.Zero && d.y >= FP.Zero;
            valid = valid && CustomMath.IsValid(lowerBound) && CustomMath.IsValid(upperBound);//to be checked
            return valid;
        }

        /// Get the center of the AABB.
        public TSVector2 GetCenter()
        {
            return CustomMath.FPHalf * (lowerBound + upperBound);
        }

        /// Get the extents of the AABB (half-widths).
        public TSVector2 GetExtents()
        {
            return CustomMath.FPHalf * (upperBound - lowerBound);
        }

        /// Get the perimeter length
        public FP GetPerimeter()
        {
            FP wx = upperBound.x - lowerBound.x;
            FP wy = upperBound.y - lowerBound.y;
            return 2 * (wx + wy);
        }

        /// Combine an AABB into this one.
        public void Combine(ref AABB aabb)
        {
            lowerBound = TSVector2.Min(lowerBound, aabb.lowerBound);
            upperBound = TSVector2.Max(upperBound, aabb.upperBound);
        }

        /// Combine two AABBs into this one.
        public void Combine(ref AABB aabb1, ref AABB aab)
        {
            lowerBound = TSVector2.Min(aabb1.lowerBound, aab.lowerBound);
            upperBound = TSVector2.Max(aabb1.upperBound, aab.upperBound);
        }

        /// Does this aabb contain the provided AABB.
        public bool Contains(ref AABB aabb)
        {
            bool result = true;
            result = result && lowerBound.x <= aabb.lowerBound.x;
            result = result && lowerBound.y <= aabb.lowerBound.y;
            result = result && aabb.upperBound.x <= upperBound.x;
            result = result && aabb.upperBound.y <= upperBound.y;
            return result;
        }

        public bool RayCast(ref RayCastOutput output, ref RayCastInput input)
        {
            FP tmin = FP.MinValue;
            FP tmax = FP.MaxValue;

            TSVector2 p = input.p1;
            TSVector2 d = input.p2 - input.p1;
            TSVector2 absD = d;
            absD.x = TSMath.Abs(absD.x);
            absD.y = TSMath.Abs(absD.y);

            TSVector2 normal = TSVector2.zero;

            for (int i = 0; i < 2; ++i)
            {
                FP pi = CustomMath.Get(p, i);
                FP li = CustomMath.Get(lowerBound, i);
                FP ui = CustomMath.Get(upperBound, i);
                FP di = CustomMath.Get(d, i);
                if (CustomMath.Get(absD, i) < CustomMath.EPSILON)
                {
                    // Parallel.             
                    if (pi < li || ui < pi)
                    {
                        return false;
                    }
                }
                else
                {
                    FP inv_d = 1 / di;
                    FP t1 = (li - pi) * inv_d;
                    FP t2 = (ui - pi) * inv_d;

                    // Sign of the normal vector.
                    FP s = -1;

                    if (t1 > t2)
                    {
                        CustomMath.Swap<FP>(ref t1, ref t2);
                        s = 1;
                    }

                    // Push the min up
                    if (t1 > tmin)
                    {
                        normal = TSVector2.zero;
                        CustomMath.Set(ref normal, i, s);
                        tmin = t1;
                    }

                    // Pull the max down
                    tmax = TSMath.Min(tmax, t2);

                    if (tmin > tmax)
                    {
                        return false;
                    }
                }
            }

            // Does the ray start inside the box?
            // Does the ray intersect beyond the max fraction?
            if (tmin < 0 || input.maxFraction < tmin)
            {
                return false;
            }

            // Intersection.
            output.fraction = tmin;
            output.normal = normal;
            return true;
        }


    }

}
