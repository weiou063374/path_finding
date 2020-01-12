///author:huwei
///date:2018.4.13
using System;
using System.Collections.Generic;
using TrueSync;
namespace AStarMachine
{
    public class AStarNode:IResetable, IBinaryHeapNode
    {
        public enum ENodeState
        {
            unknown = 0,
            open = 1,
            closed = 2,
            unwalkable = 3,//examined, node is not blocked
            walkable ,  //examined, node is blocked
            target
        }

        public int pathId;
        public int NodeID=-1;
        public FP G;
        public FP H;
        public FP F;
        public FP extraCost;
        public List<AStarNode> children=null;
        public int PreviousCount=0;
       // public AStarNode Previous;
        public AStarNode Parent;
        public ENodeState Flag;
        int _heapIndex;
        public int HeapIndex { get { return _heapIndex; }set { _heapIndex = value; } }

        public AStarNode()
        {
            children = new List<AStarNode>();
            Reset();
        }
        public virtual void Reset()
        {
            Flag = ENodeState.unknown;
            G = H = F = FP.MaxValue;
            extraCost = FP.Zero;
            NodeID = -1;
            PreviousCount = 0;
            Parent = null;
            _heapIndex = -1;
            if(children!=null)
            {
                children.Clear();
            }
            pathId = -1;
        }
    }
}
