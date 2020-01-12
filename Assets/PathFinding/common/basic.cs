///author:huwei
///date:2018.4.5
#if UNITY_EDITOR
using UnityEngine;
#endif
using System.Collections;
using System;
using TrueSync;
//namespace PathFinding
//{

    [Serializable]
    public struct IInt3
    {
        public int x;
        public int y;
        public int z;
        public static IInt3 zero = new IInt3(0, 0, 0);
#if UNITY_5_5_OR_NEWER && UNITY_EDITOR
    public IInt3(Vector3 pos)
        {
            this.x = (int)(pos.x*FInt.FloatPrecision);
            this.y = (int)(pos.y*FInt.FloatPrecision);
            this.z = (int)(pos.z*FInt.FloatPrecision);
        }
    #endif
        public IInt3(int x, int y, int z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public void Set(int x, int y, int z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public long sqrMagnitudeLong
        {
            get
            {
                return (long)x * (long)x + (long)y * (long)y + (long)z * (long)z;
            }
        }

        public static IInt3 operator +(IInt3 a, IInt3 b)
        {
            return new IInt3(a.x + b.x, a.y + b.y, a.z + b.z);
        }

        public static IInt3 operator -(IInt3 a, IInt3 b)
        {
            return new IInt3(a.x - b.x, a.y - b.y, a.z - b.z);
        }

        public static bool operator ==(IInt3 a, IInt3 b)
        {
            return a.x == b.x && a.y == b.y && a.z == b.z;
        }

        public static bool operator !=(IInt3 a, IInt3 b)
        {
            return a.x != b.x && a.y == b.y && a.z == b.z;
        }

        /** Dot product of the two coordinates */
        public static long DotLong(IInt3 a, IInt3 b)
        {
            return (long)a.x * (long)b.x + (long)a.y * (long)b.y + (long)a.z * (long)b.z;
        }

        public override bool Equals(System.Object o)
        {
            if (o == null) return false;
            var rhs = (IInt3)o;

            return x == rhs.x && y == rhs.y && z == rhs.z;
        }

        public override int GetHashCode()
        {
            return x * 73856093 ^ y * 19349663 ^ z * 83492791;
        }
#if UNITY_5_5_OR_NEWER && UNITY_EDITOR
    public static explicit operator Vector3(IInt3 a)
        {
           Vector3 vec;
            vec.x= (a.x*FInt.PrecisionFactor).AsFloat();
            vec.y= (a.y * FInt.PrecisionFactor).AsFloat();
            vec.z= (a.z * FInt.PrecisionFactor).AsFloat();
            return vec;
        }
    #endif
        public static explicit operator TSVector(IInt3 a)
        {
            TSVector vec;
            vec.x = (a.x * FInt.PrecisionFactor);
            vec.y = (a.y * FInt.PrecisionFactor);
            vec.z = (a.z * FInt.PrecisionFactor);
            return vec;
        }
#if UNITY_5_5_OR_NEWER && UNITY_EDITOR
    public static explicit operator IInt3(Vector3 a)
        {
            IInt3 vec;
            vec.x = (int)(a.x * FInt.FloatPrecision);
            vec.y = (int)(a.y * FInt.FloatPrecision);
            vec.z = (int)(a.z * FInt.FloatPrecision);
            return vec;
        }
    #endif
        public static IInt3 Min(IInt3 a, IInt3 b)
        {
            return new IInt3(System.Math.Min(a.x, b.x), System.Math.Min(a.y, b.y), System.Math.Min(a.z, b.z));
        }

        public static IInt3 Max(IInt3 a, IInt3 b)
        {
            return new IInt3(System.Math.Max(a.x, b.x), System.Math.Max(a.y, b.y), System.Math.Max(a.z, b.z));
        }

        public override string ToString()
        {
            return "(" + x + ", " + y + ", " + z + ")";
        }
    }
    public struct IVec3
    {
        public int x;
        public int y;
        public int z;

        public static readonly FP FloatPrecision = FInt.FloatPrecision;
        public static readonly FP PrecisionFactor = FInt.PrecisionFactor;
        private static IVec3 _zero = new IVec3(0, 0, 0);
        public static IVec3 MaxValue = new IVec3(int.MaxValue, int.MaxValue, int.MaxValue);
        public static IVec3 MinValue = new IVec3(int.MinValue, int.MinValue, int.MinValue);
        public static IVec3 zero { get { return _zero; } }

        public static IVec3 _xzOne = new IVec3((int)FloatPrecision,0, (int)FloatPrecision);
        public static IVec3 _up = new IVec3(0, (int)FloatPrecision, 0);
        public static IVec3 _right = new IVec3((int)FloatPrecision, 0, 0);
        public static IVec3 _forward = new IVec3(0, 0, (int)FloatPrecision);
        public static IVec3 _northEst= new IVec3(70711, 0, 70711);
        public static IVec3 _northWest = new IVec3(-70711, 0, 70711);
        public static IVec3 _southEst = new IVec3(70711, 0, -70711);
        public static IVec3 _southWest = new IVec3(-70711, 0, -70711);
        public static IVec3 up { get { return _up; } }
        public void Set(int x1, int y1, int z1)
        {
            x = x1;
            y = y1;
            z = z1;
        }
        //public void Set(float x1, float y1, float z1)
        //{
        //    x =FInt.Int(x1);
        //    y = FInt.Int(y1);
        //    z = FInt.Int(z1);
        //}

        public static float ConverPrecision(float val)
        {
            return (float)System.Math.Round(val, 5, MidpointRounding.AwayFromZero);
        }
        //public static int Int(float val)
        //{
        //    return (int)(FloatPrecision * val);
        //}
        //public static int Int(double val)
        //{
        //    return (int)(FloatPrecision * val);
        //}
        //public static float Float(int val)
        //{
        //    return (float)(PrecisionFactor * val);
        //}
      
        public IVec3(int _x, int _y, int _z)
        {
            x = _x;
            y = _y;
            z = _z;
        }
        public static IVec3 Min(IVec3 a, IVec3 b)
        {
            a.x = System.Math.Min(a.x, b.x);
            a.y = System.Math.Min(a.y, b.y);
            a.z = System.Math.Min(a.z, b.z);
            return a;
        }
       

        public static IVec3 Max(IVec3 a, IVec3 b)
        {
            a.x = System.Math.Max(a.x, b.x);
            a.y = System.Math.Max(a.y, b.y);
            a.z = System.Math.Max(a.z, b.z);
            return a;
        }
        public static bool operator ==(IVec3 lhs, IVec3 rhs)
        {
            return lhs.x == rhs.x &&
                   lhs.y == rhs.y &&
                   lhs.z == rhs.z;
        }

        public static bool operator !=(IVec3 lhs, IVec3 rhs)
        {
            return lhs.x != rhs.x ||
                   lhs.y != rhs.y ||
                   lhs.z != rhs.z;
        }

       
        public static IVec3 operator -(IVec3 lhs, IVec3 rhs)
        {
            lhs.x -= rhs.x;
            lhs.y -= rhs.y;
            lhs.z -= rhs.z;
            return lhs;
        }

        public static IVec3 operator -(IVec3 lhs)
        {
            lhs.x = -lhs.x;
            lhs.y = -lhs.y;
            lhs.z = -lhs.z;
            return lhs;
        }

        // REMOVE!
        public static IVec3 operator +(IVec3 lhs, IVec3 rhs)
        {
            lhs.x += rhs.x;
            lhs.y += rhs.y;
            lhs.z += rhs.z;
            return lhs;
        }

        public static IVec3 operator *(IVec3 lhs, int rhs)
        {
            lhs.x *= rhs;
            lhs.y *= rhs;
            lhs.z *= rhs;

            return lhs;
        }

        //public static IVec3 operator *(IVec3 lhs, float rhs)
        //{
        //    lhs.x = (int)System.Math.Round(lhs.x * rhs);
        //    lhs.y = (int)System.Math.Round(lhs.y * rhs);
        //    lhs.z = (int)System.Math.Round(lhs.z * rhs);

        //    return lhs;
        //}

        //public static IVec3 operator *(IVec3 lhs, double rhs)
        //{
        //    lhs.x = (int)System.Math.Round(lhs.x * rhs);
        //    lhs.y = (int)System.Math.Round(lhs.y * rhs);
        //    lhs.z = (int)System.Math.Round(lhs.z * rhs);

        //    return lhs;
        //}
    

        //public static IVec3 operator /(IVec3 lhs, float rhs)
        //{
        //    lhs.x = (int)System.Math.Round(lhs.x / rhs);
        //    lhs.y = (int)System.Math.Round(lhs.y / rhs);
        //    lhs.z = (int)System.Math.Round(lhs.z / rhs);
        //    return lhs;
        //}

        public int this[int i]
        {
            get
            {
                return i == 0 ? x : (i == 1 ? y : z);
            }
            set
            {
                if (i == 0) x = value;
                else if (i == 1) y = value;
                else z = value;
            }
        }

        /** Angle between the vectors in radians */
        //public static float Angle(IVec3 lhs, IVec3 rhs)
        //{
        //    double cos = DotLong(lhs, rhs) / ((double)lhs.magnitude * (double)rhs.magnitude);

        //    cos = cos < -1 ? -1 : (cos > 1 ? 1 : cos);
        //    return (float)System.Math.Acos(cos);
        //}
    
        public static long Dot(IVec3 lhs, IVec3 rhs)
        {
            return
                ((long)lhs.x) * rhs.x +
                ((long)lhs.y) * rhs.y +
                ((long)lhs.z) * rhs.z;
        }

        //public static IVec3 Cross(IVec3 lhs, IVec3 rhs)
        //{
        //    long lhsX = (long)lhs.x;
        //    long lhsY = (long)lhs.y;
        //    long lhsZ = (long)lhs.z;
        //    IVec3 vec;
        //    vec.x = FInt.IntFromLong(lhsY * rhs.z - lhsZ * rhs.y);
        //    vec.y = FInt.IntFromLong(-lhsX * rhs.z + lhsZ * rhs.x);
        //    vec.z = FInt.IntFromLong(lhsX * rhs.y - lhsY * rhs.x);
        //    return vec;
        //}
        public static long XZSqrMagnitude(IVec3 a, IVec3 b)
        {
            long dx = b.x - a.x;
            long dz = b.z - a.z;

            return dx * dx + dz * dz;
        }
        public static long DotLong(IVec3 lhs, IVec3 rhs)
        {
            return
                (long)lhs.x * (long)rhs.x +
                (long)lhs.y * (long)rhs.y +
                (long)lhs.z * (long)rhs.z;
        } 

        public FInt magnitude
        {
            get
            {
                double _x = x;
                double _y = y;
                double _z = z;

                int val= (int)System.Math.Round(System.Math.Sqrt(_x * _x + _y * _y + _z * _z));
                FInt fv = FInt.zero;
                fv.Set(val);
                return fv;
            }
        }
        //public int costMagnitude
        //{
        //    get
        //    {
        //        return (int)magnitude;// System.Math.Round(magnitude);
        //    }
        //}

    
        //public static IVec3 Lerp(IVec3 p1, IVec3 p2, float f)
        //{
        //    return (p1 * (1 - f) + p2 * f);
        //}

        public IVec3 normalized
        {
            get
            {
                IVec3 vec;
                double magnitude = System.Math.Sqrt(sqrMagnitudeLong);
                if (magnitude == 0)
                {
                    vec.x = 0;
                    vec.y = 0;
                    vec.z = 0;
                }
                else
                {
                    vec.x = FInt.Int(x / magnitude);
                    vec.y = FInt.Int(y / magnitude);
                    vec.z = FInt.Int(z / magnitude);
                }
                return vec;
            }
        }
        /** The squared magnitude of the vector */
        public long sqrMagnitudeLong
        {
            get
            {
                long _x = x;
                long _y = y;
                long _z = z;
                return (_x * _x + _y * _y + _z * _z);
            }
        }

        public static implicit operator string(IVec3 ob)
        {
            return ob.ToString();
        }

        /** Returns a nicely formatted string representing the vector */
        public override string ToString()
        {
            return "( " + x + ", " + y + ", " + z + ")";
        }

        public override bool Equals(System.Object o)
        {
            if (o == null) return false;

            var rhs = (IVec3)o;

            return x == rhs.x &&
                   y == rhs.y &&
                   z == rhs.z;
        }

        public override int GetHashCode()
        {
            return x * 73856093 ^ y * 19349663 ^ z * 83492791;
        }
    }
    public struct IInt2
    {
        public int x;
        public int y;
        public static IInt2 MaxValue = new IInt2(int.MaxValue, int.MaxValue);
        public static IInt2 MinValue = new IInt2(int.MinValue, int.MinValue);
        public static IInt2 zero = new IInt2(0, 0);
        public IInt2(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
        public void Set(int x, int y)
        {
            this.x = x;
            this.y = y;
        }
        public long sqrMagnitudeLong
        {
            get
            {
                return (long)x * (long)x + (long)y * (long)y;
            }
        }
        public int sqrMagnitudeInt
        {
            get
            {
                return x * x + y * y;
            }
        }
    public static IInt2 operator +(IInt2 a, IInt2 b)
        {
            return new IInt2(a.x + b.x, a.y + b.y);
        }

        public static IInt2 operator -(IInt2 a, IInt2 b)
        {
            return new IInt2(a.x - b.x, a.y - b.y);
        }

        public static bool operator ==(IInt2 a, IInt2 b)
        {
            return a.x == b.x && a.y == b.y;
        }

        public static bool operator !=(IInt2 a, IInt2 b)
        {
            return a.x != b.x || a.y != b.y;
        }

        /** Dot product of the two coordinates */
        public static long DotLong(IInt2 a, IInt2 b)
        {
            return (long)a.x * (long)b.x + (long)a.y * (long)b.y;
        }

        public override bool Equals(System.Object o)//hashcode is valid
        {
            if(PathFinding.PathFindingManager.DEBUG)
            {
                if (!(o is IInt2))
                {
                    return false;
                }
                IInt2 rhs = (IInt2)o;
                bool bEqual = x == rhs.x && y == rhs.y;
#if UNITY_EDITOR && !MULTI_THREAD
                if (!bEqual)
                {
                    UnityEngine.Debug.LogError("IInt2  unequal ");
                }
#endif
            return bEqual;
            }

        return true;      
    }
        public bool Equals(IInt2 rhs)
        {
            return x == rhs.x && y == rhs.y;
        }
    public override int GetHashCode()
        {
            return x * 49157 + y * 98317;
        }


        private static readonly int[] Rotations = {
            1, 0,  //Identity matrix
			0, 1,

            0, 1,
            -1, 0,

            -1, 0,
            0, -1,

            0, -1,
            1, 0
        };

        /** Returns a new IInt2 rotated 90*r degrees around the origin.
		 * \deprecated Deprecated becuase it is not used by any part of the A* Pathfinding Project
		 */
        [System.Obsolete("Deprecated becuase it is not used by any part of the A* Pathfinding Project")]
        public static IInt2 Rotate(IInt2 v, int r)
        {
            r = r % 4;
            return new IInt2(v.x * Rotations[r * 4 + 0] + v.y * Rotations[r * 4 + 1], v.x * Rotations[r * 4 + 2] + v.y * Rotations[r * 4 + 3]);
        }

        public static IInt2 Min(IInt2 a, IInt2 b)
        {
            return new IInt2(System.Math.Min(a.x, b.x), System.Math.Min(a.y, b.y));
        }

        public static IInt2 Max(IInt2 a, IInt2 b)
        {
            return new IInt2(System.Math.Max(a.x, b.x), System.Math.Max(a.y, b.y));
        }

        public static IInt2 FromIVec3XZ(IVec3 o)
        {
            return new IInt2(o.x, o.z);
        }

        public static IVec3 ToIVec3XZ(IInt2 o)
        {
            return new IVec3(o.x, 0, o.y);
        }

        public override string ToString()
        {
            return "(" + x + ", " + y + ")";
        }
    }

    public struct FInt
    {
        public const int FloatPrecision = 100000;
        public static readonly FP PrecisionFactor = FP.One/ FloatPrecision;
        public static FInt MaxValue = new FInt(int.MaxValue);
        public static FInt zero = new FInt(0);
        public static FInt MinValue = new FInt(int.MinValue);
        public static FInt one = new FInt(FloatPrecision);
        int _value;
        public int Value { get { return _value; } }
   //     public float FloatValue { get { return Float(_value); } }
        public FInt(FInt val)
        {
            _value = val._value;
        }
        public void Set(int val)
        {
            _value = val;
        }
        public void Set(float val)
        {
            _value = Int(val);
        }
        public FInt(int val)
        {
            _value = val;
        }
        public FInt(float val)
        {
            _value = (int)System.Math.Round(FloatPrecision * val);
        }
        public static int Int(double val)
        {
            return (int)System.Math.Round(val * FloatPrecision);
        }
        public static int Int(float val)
        {
            return (int)System.Math.Round(val * FloatPrecision);
        }
        //public static float Float(int val)
        //{
        //    return (float)(PrecisionFactor * val);
        //}
  
        //public static explicit operator float(FInt ob)
        //{            
        //    return ob.FloatValue;
        //}
        public static explicit operator FInt(float ob)
        {
            FInt f;
            f._value = Int(ob);
            return f;
        }
        public static explicit operator FInt(double ob)
        {
            FInt f;
            f._value = Int(ob);
            return f;
        }
    //public static explicit operator FInt(int ob)
    //{
    //    FInt f;
    //    f._value = Int(ob);
    //    return f;
    //}
    public static FInt operator *(FInt a, FInt b)
        {
            a._value = (a._value + b._value);
            return a;
        }
        public static float operator /(FInt a, FInt b)
        {
            return (a._value / (float)b._value);
        }
        public static FInt operator +(FInt a, FInt b)
        {
            a._value = (a._value + b._value);
            return a;
        }

        public static FInt operator -(FInt a, FInt b)
        {
            a._value = (a._value - b._value);
            return a;
        }
        public static FInt operator *(FInt lhs, int rhs)
        {
            lhs._value = lhs._value * rhs;
            return lhs;
        }

        public static FInt operator *(FInt lhs, float rhs)
        {
            lhs._value = (int)System.Math.Round(lhs._value * rhs);
            return lhs;
        }
        public static bool operator ==(FInt a, FInt b)
        {
            return a._value == b._value;
        }
        public override bool Equals(System.Object o)
        {
            if (!(o is FInt))
            {
                return false;
            }
            var rhs = (FInt)o;

            return _value == rhs._value;
        }
        public override int GetHashCode()
        {
            return _value.GetHashCode();
        }
        public static bool operator !=(FInt a, FInt b)
        {
            return a._value != b._value;
        }
        public static FInt Max(FInt a, FInt b)
        {
            return a._value > b._value ? a : b;
        }
        public static FInt Min(FInt a, FInt b)
        {
            return a._value < b._value ? a : b;
        }
   
        public static bool operator >(FInt a, FInt b)
        {
            return a._value > b._value;
        }
        public static bool operator <(FInt a, FInt b)
        {
            return a._value < b._value;
        }
        public static bool operator >=(FInt a, FInt b)
        {
            return a._value >= b._value;
        }
        public static bool operator <=(FInt a, FInt b)
        {
            return a._value <= b._value;
        }
    }

    public struct MatrixInt4x4
    {
        public int m00;
        public int m10;
        public int m20;
        public int m30;
        public int m01;
        public int m11;
        public int m21;
        public int m31;
        public int m02;
        public int m12;
        public int m22;
        public int m32;
        public int m03;
        public int m13;
        public int m23;
        public int m33;
        public int this[int row, int column]
        {
            get
            {
                return this[row + (column * 4)];
            }
            set
            {
                this[row + (column * 4)] = value;
            }
        }
        public int this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return this.m00;

                    case 1:
                        return this.m10;

                    case 2:
                        return this.m20;

                    case 3:
                        return this.m30;

                    case 4:
                        return this.m01;

                    case 5:
                        return this.m11;

                    case 6:
                        return this.m21;

                    case 7:
                        return this.m31;

                    case 8:
                        return this.m02;

                    case 9:
                        return this.m12;

                    case 10:
                        return this.m22;

                    case 11:
                        return this.m32;

                    case 12:
                        return this.m03;

                    case 13:
                        return this.m13;

                    case 14:
                        return this.m23;

                    case 15:
                        return this.m33;
                }
                throw new IndexOutOfRangeException("Invalid matrix index!");
            }
            set
            {
                switch (index)
                {
                    case 0:
                        this.m00 = value;
                        break;

                    case 1:
                        this.m10 = value;
                        break;

                    case 2:
                        this.m20 = value;
                        break;

                    case 3:
                        this.m30 = value;
                        break;

                    case 4:
                        this.m01 = value;
                        break;

                    case 5:
                        this.m11 = value;
                        break;

                    case 6:
                        this.m21 = value;
                        break;

                    case 7:
                        this.m31 = value;
                        break;

                    case 8:
                        this.m02 = value;
                        break;

                    case 9:
                        this.m12 = value;
                        break;

                    case 10:
                        this.m22 = value;
                        break;

                    case 11:
                        this.m32 = value;
                        break;

                    case 12:
                        this.m03 = value;
                        break;

                    case 13:
                        this.m13 = value;
                        break;

                    case 14:
                        this.m23 = value;
                        break;

                    case 15:
                        this.m33 = value;
                        break;

                    default:
                        throw new IndexOutOfRangeException("Invalid matrix index!");
                }
            }
        }
    
        public IVec3 MultiplyVector(IVec3 v)
        {
            IVec3 vector;
            vector.x = ((this.m00 * v.x) + (this.m01 * v.y)) + (this.m02 * v.z);
            vector.y = ((this.m10 * v.x) + (this.m11 * v.y)) + (this.m12 * v.z);
            vector.z = ((this.m20 * v.x) + (this.m21 * v.y)) + (this.m22 * v.z);
            return vector;
        }

        public static MatrixInt4x4 Scale(IVec3 v)
        {
            return new MatrixInt4x4 { m00 = v.x, m01 = 0, m02 = 0, m03 = 0, m10 = 0, m11 = v.y, m12 = 0, m13 = 0, m20 = 0, m21 = 0, m22 = v.z, m23 = 0, m30 = 0, m31 = 0, m32 = 0, m33 = 1 };
        }

        public static MatrixInt4x4 zero
        {
            get
            {
                return new MatrixInt4x4 { m00 = 0, m01 = 0, m02 = 0, m03 = 0, m10 = 0, m11 = 0, m12 = 0, m13 = 0, m20 = 0, m21 = 0, m22 = 0, m23 = 0, m30 = 0, m31 = 0, m32 = 0, m33 = 0 };
            }
        }
        public static MatrixInt4x4 identity
        {
            get
            {
                return new MatrixInt4x4 { m00 = 1, m01 = 0, m02 = 0, m03 = 0, m10 = 0, m11 = 1, m12 = 0, m13 = 0, m20 = 0, m21 = 0, m22 = 1, m23 = 0, m30 = 0, m31 = 0, m32 = 0, m33 = 1 };
            }
        }
         public static MatrixInt4x4 operator *(MatrixInt4x4 lhs, MatrixInt4x4 rhs)
        {
            return new MatrixInt4x4 { m00 = (((lhs.m00 * rhs.m00) + (lhs.m01 * rhs.m10)) + (lhs.m02 * rhs.m20)) + (lhs.m03 * rhs.m30), m01 = (((lhs.m00 * rhs.m01) + (lhs.m01 * rhs.m11)) + (lhs.m02 * rhs.m21)) + (lhs.m03 * rhs.m31), m02 = (((lhs.m00 * rhs.m02) + (lhs.m01 * rhs.m12)) + (lhs.m02 * rhs.m22)) + (lhs.m03 * rhs.m32), m03 = (((lhs.m00 * rhs.m03) + (lhs.m01 * rhs.m13)) + (lhs.m02 * rhs.m23)) + (lhs.m03 * rhs.m33), m10 = (((lhs.m10 * rhs.m00) + (lhs.m11 * rhs.m10)) + (lhs.m12 * rhs.m20)) + (lhs.m13 * rhs.m30), m11 = (((lhs.m10 * rhs.m01) + (lhs.m11 * rhs.m11)) + (lhs.m12 * rhs.m21)) + (lhs.m13 * rhs.m31), m12 = (((lhs.m10 * rhs.m02) + (lhs.m11 * rhs.m12)) + (lhs.m12 * rhs.m22)) + (lhs.m13 * rhs.m32), m13 = (((lhs.m10 * rhs.m03) + (lhs.m11 * rhs.m13)) + (lhs.m12 * rhs.m23)) + (lhs.m13 * rhs.m33), m20 = (((lhs.m20 * rhs.m00) + (lhs.m21 * rhs.m10)) + (lhs.m22 * rhs.m20)) + (lhs.m23 * rhs.m30), m21 = (((lhs.m20 * rhs.m01) + (lhs.m21 * rhs.m11)) + (lhs.m22 * rhs.m21)) + (lhs.m23 * rhs.m31), m22 = (((lhs.m20 * rhs.m02) + (lhs.m21 * rhs.m12)) + (lhs.m22 * rhs.m22)) + (lhs.m23 * rhs.m32), m23 = (((lhs.m20 * rhs.m03) + (lhs.m21 * rhs.m13)) + (lhs.m22 * rhs.m23)) + (lhs.m23 * rhs.m33), m30 = (((lhs.m30 * rhs.m00) + (lhs.m31 * rhs.m10)) + (lhs.m32 * rhs.m20)) + (lhs.m33 * rhs.m30), m31 = (((lhs.m30 * rhs.m01) + (lhs.m31 * rhs.m11)) + (lhs.m32 * rhs.m21)) + (lhs.m33 * rhs.m31), m32 = (((lhs.m30 * rhs.m02) + (lhs.m31 * rhs.m12)) + (lhs.m32 * rhs.m22)) + (lhs.m33 * rhs.m32), m33 = (((lhs.m30 * rhs.m03) + (lhs.m31 * rhs.m13)) + (lhs.m32 * rhs.m23)) + (lhs.m33 * rhs.m33) };
        }



    }
//}