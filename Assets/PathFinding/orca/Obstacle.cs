
///date:2018.4.25
using TrueSync;
namespace PathFinding
{    public struct Line
    {
        public TSVector2 direction;
        public TSVector2 point;
    }
    internal class Obstacle
    {        
        internal Obstacle next_=null;
        internal Obstacle previous_=null;
        internal TSVector2 direction_=TSVector2.zero;
        internal TSVector2 point_ = TSVector2.zero;
        internal int id_=0;
        internal bool convex_=false;
    }
}
