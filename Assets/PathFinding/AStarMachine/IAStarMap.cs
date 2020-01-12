///author:huwei
///date:2018.4.13
using TrueSync;
namespace AStarMachine
{
    public enum EDynamicBlockStatus
    {
        none,
        ignore,
        blocked,
        stationaryBlocked
    }
    public interface IAStarMap:IResetable
    {
       
        int GetNeighboursCount(AStarNode pAStarNode); 
        FP GetAStarNeighbourCost(int iNeighbor);
        FP GetAStarNeighbourCost(int id1,int id2,IAStarGoal goal,int curPosIdx);
        // AStarNode CreateANode(short id);
        // AStarNode.ENodeState GetAStarFlags(short NodeID);
        // void SetAStarFlags(short NodeID, AStarNode.ENodeState flag);
        bool CompareNodes(AStarNode node1, AStarNode node2);
        bool CheckNodeValid(AStarNode node);
        //bool IsWalkable(int x,int y);
        //int GetIdx(int x, int y);
        // void Reset();
        EDynamicBlockStatus DynamicBlockedStatus(int posIdx, int startPosIdx, int targetId, int curPosIdx);
        int GetIdx(int x, int y);//
    }
}
