
using TrueSync;
namespace PathFinding
{
    public struct LineObstacle
    {
        public TSVector2 _p1;
        /// The second endpoint of the obstacle. 
        public TSVector2 _p2;
        /// The normal vector of the line obstacle. 
        public TSVector2 _normal;
        public LineObstacle(TSVector2 a, TSVector2 b)
        {
            _p1 = a;
            _p2 = b;
            _normal =CustomMath.perpendicular((_p2 - _p1).normalized);
        }
    }

    public struct CircleObstacle
    {
        public static CircleObstacle zero =new CircleObstacle(TSVector2.zero,FP.Zero);
        //center position of the circle
        public TSVector2 _center;
        /// radius of the circle
        public FP _radius;
        public CircleObstacle(TSVector2 center, FP r)
        {
            _center = center;
            _radius = r;
        }
    }
    public struct CircleObstacleAngleData
    {
        public static CircleObstacle zero = new CircleObstacle(TSVector2.zero, FP.Zero);
        //center position of the circle
        public TSVector2 _center;
        /// radius of the circle
        public FP _radius;
        public EBlockType _blockType;
        public static readonly CircleObstacleAngleData max =new CircleObstacleAngleData(TSVector2.zero,0);
        public CircleObstacleAngleData(TSVector2 center, FP r)
        {
            _center = center;
            _radius = r;
            quadrant = 4;
            cos = 1;
            sin = 0;
            _blockType = EBlockType.none;
        }
        public int quadrant;
        public FP cos;
        public FP sin;
    }
}