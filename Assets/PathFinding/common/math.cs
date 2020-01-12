
using System;
#if UNITY_5_5_OR_NEWER
using UnityEngine;
#endif
using TrueSync;
using System.Collections.Generic;

public struct CustomMath
{
    public const int Precision = 100000;
    public static readonly FP EPSILON = new FP(1) / Precision;
    public static readonly FP FPHalf = FP.One / 2;
    public static FP DiagonalCost = new FP(14) / 10;
    public static TSVector XZone = new TSVector(1, 0, 1);
    public static readonly FP MsToSecond = FP.One / 1000;
  	public static readonly int SecondToMs = 1000;   
    public static readonly FP invTimeStep = FP.One * 30;
    static readonly FP C_E_Val = 271828 / 100000;
    const long LOG2MAX = 0x1F00000000;
    const long LOG2MIN = -0x2000000000;
    const int FRACTIONAL_PLACES = 32;
    const long LN2 = 0xB17217F7;
    public static readonly FP EXP = 27183 * FP.EN4;
    //static readonly FP Log2Max = new FP(LOG2MAX); 
    //static readonly FP Log2Min = new FP(LOG2MIN);
    //
  	public static  int SecondToMS(FP val)
    {
        return (1000 * val).AsInt();
    }    public static int Clamp(int value, int min, int max)
    {
        value = (value > max) ? max : value;
        value = (value < min) ? min : value;
        return value;
    }
    public static TSVector Clamp(TSVector val, TSVector min, TSVector max)
    {
        TSVector vec = TSVector.zero;
        vec.Set(TSMath.Clamp(val.x, min.x, max.x), TSMath.Clamp(val.y, min.y, max.y), TSMath.Clamp(val.z, min.z, max.z));
        return vec;
    }
    public static FP FloatToFP(float val)
    {
        FP fp = FP.Zero;
        fp = (int)(Math.Round(val * Precision));
        return fp / Precision;
    }
#if UNITY_5_5_OR_NEWER
    #region for debug
    public static Vector3 TsVecToVector3(TSVector vector)
    {
        Vector3 vec = Vector3.zero;
        vec.Set(vector.x.AsFloat(), vector.y.AsFloat(), vector.z.AsFloat());
        return vec;
    }
    public static Vector3 TsVec2ToVector3(TSVector2 vector)
    {
        Vector3 vec = Vector3.zero;
        vec.Set(vector.x.AsFloat(),0, vector.y.AsFloat());
        return vec;
    }
    public static Vector2 TsVecToVector2(TSVector vector)
    {
        Vector2 vec = Vector2.zero;
        vec.Set(vector.x.AsFloat(), vector.z.AsFloat());
        return vec;
    }
    public static TSVector Vector3ToTsvec(Vector3 vector)
    {
        TSVector vec = TSVector.zero;
        vec.Set(vector.x, vector.y, vector.z);
        return vec;
    }
    public static TSVector Vector2ToTsvec(Vector2 vector)
    {
        TSVector vec = TSVector.zero;
        vec.Set(vector.x, 0, vector.y);
        return vec;
    }
    #endregion
#endif
    public static FP abs(TSVector2 vector)
    {
        return vector.magnitude;
    }
    public static TSVector2 Abs(TSVector2 v)
    {
        v.x = TSMath.Abs(v.x);
        v.y = TSMath.Abs(v.y);
        return v;
    }
    //public static FP absSq(TSVector2 vector)
    //{
    //    return vector.x * vector.x + vector.y * vector.y;
    //}
    //public static TSVector2 normalize(TSVector2 vector)
    //{
    //    return vector / abs(vector);
    //}

    internal static FP det(TSVector2 vector1, TSVector2 vector2)
    {
        return vector1.x * vector2.y - vector1.y * vector2.x;
    }
    internal static FP det(TSVector vector1, TSVector vector2)
    {
        return vector1.x * vector2.z - vector1.z * vector2.x;
    }
    internal static FP distSqPointLineSegment(TSVector2 vector1, TSVector2 TSVector2, TSVector2 vector3)
    {
        FP r = ((vector3 - vector1) * (TSVector2 - vector1)) / (TSVector2 - vector1).LengthSquared();

        if (r < FP.Zero)
        {
            return (vector3 - vector1).LengthSquared();
        }

        if (r > FP.One)
        {
            return (vector3 - TSVector2).LengthSquared();
        }

        return (vector3 - (vector1 + r * (TSVector2 - vector1))).LengthSquared();
    }
    internal static FP fabs(FP scalar)
    {
        return TrueSync.TSMath.Abs(scalar);
    }
    internal static TSVector2 TSVecToVec2(TSVector vec)
    {
        TSVector2 vec2 = TSVector2.zero;
        vec2.x = vec.x;
        vec2.y = vec.z;
        return vec2;
    }
    internal static TSVector TSVec2ToVec(TSVector2 vec)
    {
        TSVector vec2 = TSVector.zero;
        vec2.x = vec.x;
        vec2.z = vec.y;
        return vec2;
    }
#if UNITY_5_5_OR_NEWER
    internal static UnityEngine.Quaternion TSQua2ToQua(TSQuaternion vec)
    {
        UnityEngine.Quaternion vec2 = UnityEngine.Quaternion.identity;
        vec2.x = vec.x.AsFloat();
        vec2.y = vec.y.AsFloat();
        vec2.z = vec.z.AsFloat();
        vec2.w = vec.w.AsFloat();
        return vec2;
    }
#endif
    /**
     * <summary>Computes the signed distance from a line connecting the
     * specified points to a specified point.</summary>
     *
     * <returns>Positive when the point c lies to the left of the line ab.
     * </returns>
     *
     * <param name="a">The first point on the line.</param>
     * <param name="b">The second point on the line.</param>
     * <param name="c">The point to which the signed distance is to be
     * calculated.</param>
     */
    internal static FP leftOf(TSVector2 a, TSVector2 b, TSVector2 c)
    {
        return det(a - c, b - a);
    }
    internal static FP sqr(FP scalar)
    {
        return scalar * scalar;
    }
    internal static FP sqrt(FP scalar)
    {
        return TrueSync.TSMath.Sqrt(scalar);
    }
    public static TSVector Normalize(TSVector v, out FP magnitude)
    {
        magnitude = v.magnitude;
        // This is the same constant that Unity uses
        if (magnitude > EPSILON)
        {
            return v / magnitude;
        }
        else
        {
            return TSVector.zero;
        }
    }
    public static TSVector Normalize(TSVector v)
    {
        FP magnitude = v.magnitude;
        // This is the same constant that Unity uses
        if (magnitude > EPSILON)
        {
            return v / magnitude;
        }
        else
        {
            return TSVector.zero;
        }
    }
    public static FP LineCircleIntersectionFactor(TSVector circleCenter, TSVector linePoint1, TSVector linePoint2, FP radius)
    {
        FP segmentLength;
        var normalizedDirection = Normalize(linePoint2 - linePoint1, out segmentLength);
        var dirToStart = linePoint1 - circleCenter;

        var dot = TSVector.Dot(dirToStart, normalizedDirection);
        var discriminant = dot * dot - (dirToStart.sqrMagnitude - radius * radius);

        if (discriminant < 0)
        {
            // No intersection, pick closest point on segment
            discriminant = 0;
        }

        var t = -dot + TSMath.Sqrt(discriminant);
        return segmentLength > EPSILON ? t / segmentLength : 1;
    }
    public static FP Get(TSVector2 v, int idx)
    {
        return idx == 0 ? v.x : v.y;
    }
    public static void Set(ref TSVector2 v, int idx, FP val)
    {
        if (idx == 0)
        {
            v.x = val;
        }
        else
        {
            v.y = val;
        }
    }
    public static void Swap<T>(ref T a, ref T b)
    {
        T tmp = a;
        a = b;
        b = tmp;
    }
    public static bool IsValid(FP v)
    {
        return v != FP.MaxValue && v != FP.MinValue;
    }
    public static bool IsValid(TSVector2 v)
    {
        return IsValid(v.x) && IsValid(v.y);
    }
    /// Perform the cross product on two vectors. In 2D this produces a scalar.
    public static FP Cross(TSVector2 a, TSVector2 b)
    {
        return a.x * b.y - a.y * b.x;
    }

    /// Perform the cross product on a vector and a scalar. In 2D this produces
    /// a vector.
    public static TSVector2 Cross(TSVector2 a, FP s)
    {
        return new TSVector2(s * a.y, -s * a.x);
    }

    /// Perform the cross product on a scalar and a vector. In 2D this produces
    /// a vector.
    public static TSVector2 Cross(FP s, TSVector2 a)
    {
        return new TSVector2(-s * a.y, s * a.x);
    }
    public static TSVector ClosestPointOnSegment(TSVector lineStart, TSVector lineEnd, TSVector point)
    {
        var dir = lineEnd - lineStart;
        FP sqrMagn = dir.sqrMagnitude;

        if (sqrMagn <= EPSILON) return lineStart;

        FP factor = TSVector.Dot(point - lineStart, dir) / sqrMagn;
        return lineStart + TSMath.Clamp(factor, 0, 1) * dir;
    }
    public static TSVector2 ClosestPointLineSegment(TSVector2 line_start,TSVector2 line_end, TSVector2 p)
	{
		FP dota = (p - line_start) * (line_end - line_start);
		if (dota <= 0) // point line_start is closest to p
			return line_start;

		FP dotb = (p - line_end) * (line_start - line_end);
		if (dotb <= 0) // point line_end is closest to p
			return line_end;
	
		// find closest point
		FP slope = dota / (dota + dotb);
		return line_start + (line_end - line_start)*slope;
	}
public static TSVector2 ClosestPointOnSegment(TSVector2 lineStart, TSVector2 lineEnd, TSVector2 point)
    {
        return TSVecToVec2(ClosestPointOnSegment(TSVec2ToVec(lineStart), TSVec2ToVec(lineEnd), TSVec2ToVec(point)));
    }
    public static TSVector2 VecNormalize(TSVector2 v, out FP magnitude)
    {
        magnitude = v.magnitude;
        // This is the same constant that Unity uses
        if (magnitude > EPSILON)
        {
            return v / magnitude;
        }
        else
        {
            return TSVector2.zero;
        }
    }

    /// <summary>
    /// Performs multiplication without checking for overflow.
    /// Useful for performance-critical code where the values are guaranteed not to cause overflow
    /// </summary>
    public static FP FastMul(FP x, FP y)
    {

        var xl = x._serializedValue;
        var yl = y._serializedValue;

        var xlo = (ulong)(xl & 0x00000000FFFFFFFF);
        var xhi = xl >> FP.FRACTIONAL_PLACES;
        var ylo = (ulong)(yl & 0x00000000FFFFFFFF);
        var yhi = yl >> FP.FRACTIONAL_PLACES;

        var lolo = xlo * ylo;
        var lohi = (long)xlo * yhi;
        var hilo = xhi * (long)ylo;
        var hihi = xhi * yhi;

        var loResult = lolo >> FP.FRACTIONAL_PLACES;
        var midResult1 = lohi;
        var midResult2 = hilo;
        var hiResult = hihi << FP.FRACTIONAL_PLACES;

        var sum = (long)loResult + midResult1 + midResult2 + hiResult;
        FP val = FP.Zero;
        val._serializedValue = sum;
        return val;
    }
    internal static FP Pow2(FP x)
    {
        if (x.RawValue == 0)
        {
            return FP.One;
        }

        // Avoid negative arguments by exploiting that exp(-x) = 1/exp(x).
        bool neg = x.RawValue < 0;
        if (neg)
        {
            x = -x;
        }
        // static readonly FP Log2Max = new FP(LOG2MAX);
        // static readonly FP Log2Min = new FP(LOG2MIN);
        if (x == FP.One)
        {
            return neg ? FP.Half : 2;
        }
        if (x >= LOG2MAX)
        {
            return neg ? FP.One / FP.MaxValue : FP.MaxValue;
        }
        if (x <= LOG2MIN)
        {
            return neg ? FP.MaxValue : FP.Zero;
        }

        /* The algorithm is based on the power series for exp(x):
         * http://en.wikipedia.org/wiki/Exponential_function#Formal_definition
         * 
         * From term n, we get term n+1 by multiplying with x/n.
         * When the sum term drops to zero, we can stop summing.
         */

        int integerPart = TSMath.Floor(x).AsInt();
        // Take fractional part of exponent
        x._serializedValue = x._serializedValue & 0x00000000FFFFFFFF;

        var result = FP.One;
        var term = FP.One;
        int i = 1;
        while (term._serializedValue != 0)
        {
            term = FastMul(FastMul(x, term), LN2) / i;
            result += term;
            i++;
        }

        result._serializedValue = (result._serializedValue << integerPart);
        if (neg)
        {
            result = FP.One / result;
        }

        return result;
    }
    public static FP Pow(FP b, FP exp)
    {
        if (b == FP.One)
        {
            return FP.One;
        }
        if (exp._serializedValue == 0)
        {
            return FP.One;
        }
        if (b._serializedValue == 0)
        {
            if (exp._serializedValue < 0)
            {
                throw new DivideByZeroException();
            }
            return FP.Zero;
        }

        FP log2 = Log2(b);
        return Pow2(exp * log2);
    }
    internal static FP Log2(FP x)
    {
        if (x._serializedValue <= 0)
        {
            throw new ArgumentOutOfRangeException("Non-positive value passed to Ln", "x");
        }

        // This implementation is based on Clay. S. Turner's fast binary logarithm
        // algorithm (C. S. Turner,  "A Fast Binary Logarithm Algorithm", IEEE Signal
        //     Processing Mag., pp. 124,140, Sep. 2010.)

        long b = 1U << (FRACTIONAL_PLACES - 1);
        long y = 0;

        long rawX = x._serializedValue;
        while (rawX < FP.ONE)
        {
            rawX <<= 1;
            y -= FP.ONE;
        }

        while (rawX >= (FP.ONE << 1))
        {
            rawX >>= 1;
            y += FP.ONE;
        }

        var z = FP.Zero;
        z._serializedValue = (rawX);

        for (int i = 0; i < FRACTIONAL_PLACES; i++)
        {
            z = FastMul(z, z);
            if (z._serializedValue >= (FP.ONE << 1))
            {
                z = FP.Zero;
                z._serializedValue = (z._serializedValue >> 1);
                y += b;
            }
            b >>= 1;
        }
        FP val = FP.Zero;
        val._serializedValue = y;
        return val;
    }
    public static TSVector2 perpendicular(TSVector2 v) 
    {
        return new TSVector2(-v.y, v.x);
    }
    public static TSVector perpendicular(TSVector v)
    {
        return new TSVector(v.z, 0,-v.x);
    }
 public static FP PowInt(FP val,int pow)
    {
        if(pow==0)
        {
            return FP.One;
        }
        if(pow==1)
        {
            return val;
        }
        FP preVal = val;
        while(pow>1)
        {
            preVal = preVal * preVal;
            pow = pow / 2;
        }
        if(pow%2==1)
        {
            preVal= preVal * val;
        }
        if(val<0)
        {
            preVal = 1 / preVal;
        }
        return preVal;
    }
    ///-----------
    
    public static FP ApproximateExp2(FP x)//around 0,Taylor's formula 
    {
        FP xSqr = x * x;
        return 1 + x + xSqr * FP.EN1 * 5;
    }
    public static FP ApproximateExp3(FP x)//around 0,Taylor's formula
    {
        FP xSqr = x * x;
        return 1 + x + xSqr * FP.EN1 * 5+ xSqr*x*FP.EN4*1667;//
    }
#if UNITY_5_5_OR_NEWER
    //y=0,
    /*
     L1=p1+t1(D1);D1=p2-p1
     L2=p0+tD0;

        ((p0-p1)xD0)*(D1xD0)
     t1=-------------------  ,0<=t1<=1,deprecated
             ||D1xD0||
    */
    public static bool IsPointInPolygon(List<UnityEngine.Vector3> polygonPoints, UnityEngine.Vector3 p0)
    {
        int count = polygonPoints.Count;
        int j = count - 1;
        bool oddNodes = false;
        float y = p0.z;
        float x = p0.x;
        for (int i=1;i<count;i++)
        {
            if ((polygonPoints[i].z < y && polygonPoints[j].z >= y
                || polygonPoints[j].z < y && polygonPoints[i].z >= y)
                && (polygonPoints[i].x <= x || polygonPoints[j].x <= x))
            {
                oddNodes ^= (polygonPoints[i].x + (y - polygonPoints[i].z)
                    / (polygonPoints[j].z - polygonPoints[i].z) * (polygonPoints[j].x - polygonPoints[i].x) < x);
            }
            j = i;
        }
        return oddNodes;
    }
#endif
}
