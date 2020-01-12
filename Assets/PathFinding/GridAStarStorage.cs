///author:huwei
///date:2018.4.17
using System;
using System.Collections.Generic;
using AStarMachine;
using TrueSync;
namespace PathFinding
{
    class GridAStarStorage:AStarStorageBase
    {
        Dictionary<int, GridNode> _gridNodes = null;
        static SingleObjectPool<GridNode> _gridNodesPool = new SingleObjectPool<GridNode>(1000);
        public void Ini()
        {
            if (_gridNodes == null)
            {
                _gridNodes = new Dictionary<int, GridNode>();
            }
            Reset();
        }
      
        public override AStarNode GetAStarNeighbour(AStarNode node, int index, int pathId, IAStarMap map,IInt2 curPos)
        {
            GridNode gNode = (GridNode)node;
         
            if (gNode != null && index >= 0 &&  index < GridMap.neighbourOffset.Length)
            {
                int x = gNode.gridPos.x + GridMap.neighbourOffset[index].x;
                int z = gNode.gridPos.y + GridMap.neighbourOffset[index].y;
                bool bSkip = false;
#if !DYNAMIC_FORCE
                IInt2 pos = IInt2.zero;
                pos.x = x;
                pos.y = z;
                IInt2 diff = curPos - pos;
                int posIdx = map.GetIdx(pos.x, pos.y);
                int curposIdx = map.GetIdx(curPos.x, curPos.y);
                EDynamicBlockStatus status= map.DynamicBlockedStatus(posIdx, -1, -1, curposIdx);
                bSkip = (status== EDynamicBlockStatus.blocked && diff.sqrMagnitudeInt> GridMap.c_dynamicRangeSqr)
                    ||status== EDynamicBlockStatus.ignore;
#endif
                return GetAStarNode(x, z, pathId, map, bSkip);
            }
            return null;
        }
        public override AStarNode GetAStarNode(int id, int pathId, IAStarMap map, bool isIgnored = false)
        {
            GridMap gMap = map as GridMap;
            IInt2 pos = gMap.GetGridPos(id);
            return GetAStarNode(pos.x, pos.y, pathId, gMap, isIgnored);
        }
        public AStarNode GetAStarNode(int x, int y, int pathId, IAStarMap map,bool isIgnored = false)
        {
            GridMap gMap = map as GridMap;
            int idx = gMap.GetIdx(x, y);
            GridNode nNode;
            if (_gridNodes.TryGetValue(idx, out nNode) || gMap.IsWalkable(x, y) || isIgnored)
            {
                if (nNode == null)
                {
                    nNode = _gridNodesPool.New();                  
                    _gridNodes[idx] = nNode;
                }
                if (nNode.pathId != pathId)
                {
                    nNode.Reset();
                    nNode.NodeID = idx;
                    nNode.gridPos.x = x;
                    nNode.gridPos.y = y;
                  
                    nNode.pathId = pathId;
                }
                return nNode;
            }
            return null;
        }
        public override void Reset()
        {
            if (_gridNodes != null)
            {
                foreach (KeyValuePair<int, GridNode> kv in _gridNodes)
                {                   
                    _gridNodesPool.Store(kv.Value);
                }
                _gridNodes.Clear();
            }
            base.Reset();
        }
    }
}
