///author:huwei
///date:2018.4.13
using System;
using System.Collections.Generic;

namespace AStarMachine
{
   class AStarStorageBase:IResetable
    {
        Stack<AStarNode> _propagationStack = null;
        BinaryHeapM<AStarNode> _heap = null;
        public int PropagationCount { get { return _propagationStack == null ? 0 : _propagationStack.Count; } }
        public void Push(AStarNode node)
        {
            _propagationStack.Push(node);
        }
        public AStarNode Pop()
        {
            if(_propagationStack.Count>0)
            {
                return _propagationStack.Pop();
            }
            return null;            
        }
        public bool BinaryHeapMCompare(AStarNode a, AStarNode b)
        {
            return a.F > b.F;
        }
        public AStarStorageBase()
        {
            _heap = new BinaryHeapM<AStarNode>(32, BinaryHeapMCompare);
            _propagationStack = new Stack<AStarNode>();
        }
        public virtual void AddToOpenSet(AStarNode node, IAStarMap map)
        {
            _heap.Add(node);
            node.Flag = AStarNode.ENodeState.open;
        }
       public virtual void AddToClosedSet(AStarNode node, IAStarMap map)
       {
            node.Flag =AStarNode.ENodeState.closed;
           // node.Next = null;
           // node.Previous = null;
        }
        public virtual void UpdateOpenSetNodeValue(AStarNode node)
        {
            if(IsInOpenSet(node))
            {
                _heap.UpdateNode(node.HeapIndex);
            }
       
        }
        public virtual bool IsInOpenSet(AStarNode node)
        {
            return node.HeapIndex >= 0;
        }
        public virtual bool IsInCloseSet(AStarNode node)
        {
            return node.HeapIndex ==-1 && node.Flag == AStarNode.ENodeState.closed;
        }
        //public virtual AStarNode FindInOpenSet(AStarNode node)
        //{

        //}
        //public virtual AStarNode FindInClosedSet(short node);
      //  public virtual void RemoveFromClosedSet(short nodeId, IAStarMap map);
       //public virtual void RemoveFromOpenSet(AStarNode node, IAStarMap map)
       //{
       //     _heap.Remove(node.HeapIndex);
       //     // map.re
       //     node.Flag = AStarNode.ENodeState.closed;
       //}
       public virtual AStarNode RemoveCheapestOpenNode(IAStarMap map)
        {
            if(_heap.numberOfItems>0)
            {
                return _heap.Remove(0);
            }
            return null;
        }
       public virtual bool CheckStorage()
        {
            return _heap.numberOfItems > 0;
        }
       public virtual void Reset()
        {
            _heap.Clear();
            _propagationStack.Clear();
        }
        public virtual AStarNode GetAStarNode(int id, int searchId, IAStarMap map, bool isTarget = false)
        {
            return null;
        }
        public virtual AStarNode GetAStarNeighbour(AStarNode pAStarNode, int iNeighbor, int searchId, IAStarMap map,IInt2 curPos)
        {
            return null;
        }
        //public virtual void ResetStorage()//IAStarMap map
        // {
        //     // map.Clear();
        //     Clear();
        // }
    }

}
