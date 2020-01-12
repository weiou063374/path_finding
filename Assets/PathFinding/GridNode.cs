///author:huwei
///date:2018.4.17

using TrueSync;
using AStarMachine;
namespace PathFinding
{
    class GridNode : AStarNode
    {
        public IInt2 gridPos;                 // Tile XY
        public GridNode()
        {
            gridPos.x = 0;
            gridPos.y = 0;
        }
        public override void Reset()
        {
            base.Reset();
            gridPos.x = 0;
            gridPos.y = 0;
        }
    }
}
