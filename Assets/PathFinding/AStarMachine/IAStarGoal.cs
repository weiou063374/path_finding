///author:huwei
///date:2018.4.13
using TrueSync;
namespace AStarMachine
{
   public interface IAStarGoal:IResetable
    {

        AStarNode StartNode { get; set; }
        AStarNode TargetNode { get; set; }
        AStarNode CurrentNode { get; set; } // { get { return CurrentNode; } private set { CurrentNode = value; } }
   
        //int originPosIdx { get; set; }
        // void InitNode(TSVector startPos,TSVector tagetPos);
        //calculate Heuristic Distance
        FP CalculateHDist(AStarNode nodeOne, AStarNode nodeTwo);
        // FP CalculateNeighbourCost(AStarNode nodeOne, AStarNode nodeTwo);
         bool IsAStarFinished(AStarNode currNode);
         bool IsNodeWalkable(AStarNode currNode);
         //void Reset();

        // int RevsPerRun { get; set; }
     
        // int RevsCurrentRun { get; set; }
    }
}
