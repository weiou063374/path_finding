///author:huwei
///date:2018.4.17
using System;
using System.Collections;
using System.Collections.Generic;
using AStarMachine;
using TrueSync;
namespace PathFinding
{
    public enum Heuristic
    {
        Manhattan,
        DiagonalManhattan,
        Euclidean,
        None
    }
    internal class GridAStarGoal : IAStarGoal
    {
       
        private AgentBase _Owner;
        //private AStarGridMap _Map;
        readonly Heuristic _heuristic = Heuristic.DiagonalManhattan;
        GridNode _startNode = null;
        GridNode _targetNode = null;
        AStarNode _closestNodeToTarget;//closest node to target node
        //
        public AStarNode _currentNode;
        public AStarNode CurrentNode {
            get { return _currentNode; }
            set {
                _currentNode = value;
                if(_currentNode!=null)
                {
                    if ((_closestNodeToTarget == null || _currentNode.PreviousCount > _closestNodeToTarget.PreviousCount))
                    {
                        _closestNodeToTarget = _currentNode;
                        // _closestNodeToTargetDst =CalculateHDist(_currentNode,_t);
                    }
                }                
            }
        }
        public AStarNode ClosestNodeToTarget
        {
            get { return _closestNodeToTarget; }
        }
        static readonly FP HeuristicFactor = new FP(1001) / new FP(1000);
        //revolutions per run
        // int _revsPerRun = 0;
        //  public int RevsPerRun { get { return _revsPerRun; } set { _revsPerRun = value; } }
        // int _revsCurrentRun = 0;
        //  public int RevsCurrentRun { get { return _revsCurrentRun; } set { _revsCurrentRun = value; } }
        //
        public AStarNode StartNode
        {
            get { return _startNode; }
            set { _startNode = (GridNode)value ; }
        }
        public AStarNode TargetNode
        {
            get { return _targetNode; }
            set { _targetNode = (GridNode)value; }
        }

        public Heuristic Heuristic
        {
            get
            {
                return _heuristic;
            }
            
        }
        
        internal static FP CalculateHValue(IInt2 gridPos1, IInt2 gridPos2, Heuristic heuristic)
        {
            FP h = FP.Zero;
            switch (heuristic)
            {
                case Heuristic.Euclidean:
                    h= TSMath.Sqrt((gridPos1 - gridPos2).sqrMagnitudeLong) * HeuristicFactor;
                    break;
                case Heuristic.Manhattan:
                    h = (Math.Abs(gridPos2.x - gridPos1.x) + Math.Abs(gridPos2.y - gridPos1.y)) * HeuristicFactor;
                    //  return h;
                    break;
                case Heuristic.DiagonalManhattan:
                    IInt2 p = gridPos2 - gridPos1;
                    p.x = Math.Abs(p.x);
                    p.y = Math.Abs(p.y);
                    int diag = Math.Min(p.x, p.y);
                    int diag2 = Math.Max(p.x, p.y);
                    h = ((CustomMath.DiagonalCost* diag + (diag2 - diag))) * HeuristicFactor;
                    break;
            }
            return h*GridMap.GetNodeSize();
        }

        internal FP CalculateHValue(GridNode node1, GridNode node2, Heuristic heuristic)
        {
            FP h = CalculateHValue(node1.gridPos,node2.gridPos, heuristic);
            return h;
        }

        public void Ini(AgentBase ai)//, AStarGridMap map
        {
            _Owner = ai;
           // _Goal = goal;
        }
        public  FP CalculateHDist(AStarNode node1, AStarNode node2)
        {
            return CalculateHValue((GridNode)node1, (GridNode)node2, _heuristic);
        }
        
        public bool IsAStarFinished(AStarNode currNode)
        {
            if (currNode == null || _targetNode==null)
                return true;

            GridNode node = (GridNode)(currNode);

            if (node.gridPos==_targetNode.gridPos)
                return true;

            return false;
        }

        public bool IsNodeWalkable(AStarNode currNode) {
            return currNode.Flag== AStarNode.ENodeState.walkable;
        }


        public  void Reset()
        {
            _startNode = null;
            _targetNode = null;
            _currentNode = null;
            _closestNodeToTarget = null;
           // _closestNodeToTargetDst = FP.Zero;
            //if(_Map!=null)
            //{
            //    _Map.Clear();
            //}
        }
    }
}
