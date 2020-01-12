///author:huwei
///date:2018.4.13
#define ASTARDEBUG
using System;
using System.Collections.Generic;
using TrueSync;
namespace AStarMachine
{
    interface IAgent
    {

    }

    internal class AStarMachine : IResetable
    {
        //three base modules
        private IAStarGoal _goal;
        private IAStarMap _map;
        private AStarStorageBase _storage;

        public IAStarGoal Goal { get { return _goal; } }
        public IAStarMap Map { get { return _map; } }
        public AStarStorageBase Storage { get { return _storage; } }
        public const int MAX_REVS = 52500;
        int _curPosIdx;
        int _revsCurrentRun = 0;
        //current frame revoutions
        public int RevsCurrentRun
        {
            get { return _revsCurrentRun; }
        }
        int _maxRevolutions = C_MaxRevolutions;
        int _pauseCount = -1;
        public int MaxRevolutions
        {
            get { return _maxRevolutions; }
            set { _maxRevolutions = value; }
        }
#if ASTARDEBUG
        public int _totalRevs = 0;
#endif
        public const int C_MaxRevolutions = 8;
       // static int s_SearchId = 0;

        int _searchId = -1;
        public IAgent _agent = null;
        //
        public IAgent Agent { get { return _agent; } }
        //
        bool _isWorking = false;
        public bool IsWorking { get { return _isWorking; } set { _isWorking = value; } }
        public bool _inQueue = false;
        public int SearchId
        {
            get
            {
                return _searchId;
            }

            set
            {
                _searchId = value;
            }
        }

        //
        public void SetPauseCount(int count)
        {
            _pauseCount = count;
        }
        public void AddToMaxRevolutions(int revs)
        {
            if(int.MaxValue- revs<= _maxRevolutions)
            {
                _maxRevolutions = int.MaxValue;
            }
            else
            {
                _maxRevolutions += revs;
            }     
        }
        public bool ShouldPause(int revs)
        {
            return _pauseCount>=0 && _pauseCount<=revs;
        }
  
        public AStarMachine()
        {
            
        }

        public void Ini(IAgent agent, IAStarGoal goal, AStarStorageBase storage, IAStarMap aStarMap)
        {
            Reset();
           
            _agent = agent;
            _goal = goal;
            _storage = storage;
            _map = aStarMap;
       
        }
      
        public void StartSearch(int startId, int endId,bool bRunDirect,TSVector startPos,int searchId,int originPosIdx)
        {
            ResetMachineData();

            _goal.Reset();
            _storage.Reset();
          //  _map.Reset();
            _searchId = searchId;//
#if UNITY_EDITOR && !MULTI_THREAD
            if(PathFinding.PathFindingManager.DEBUG && bRunDirect)
            {
                UnityEngine.Debug.Log("StartSearch");
            }
#endif
            _goal.StartNode = _storage.GetAStarNode(startId, _searchId, _map,true);
            _goal.TargetNode = _storage.GetAStarNode(endId, _searchId, _map,true);
        
            if(_goal.StartNode == null)
            {
#if UNITY_EDITOR
                UnityEngine.Debug.Log("StartSearch,targetNode is null:"+ startId);
#endif
                IsWorking = false;
                return;
            }
            _goal.StartNode.G = FP.Zero;
            _goal.StartNode.H = _goal.StartNode.F =_goal.CalculateHDist(_goal.StartNode, _goal.TargetNode);
            _storage.AddToOpenSet(_goal.StartNode, _map);
            _isWorking = true;
            if(bRunDirect)
            {
                RunAStar();
            }       
        }

        void UpdateParents(AStarNode node)
        {
            FP g = node.G;
            int c = node.children==null?0: node.children.Count;

            AStarNode kid = null;
            for (int i = 0; i < c; i++)
            {
                kid = node.children[i];
                //bool ignoredDynamicCost = kid.NodeID == _goal.TargetNode.NodeID//|| kid.NodeID == _goal.StartNode.NodeID
                //    || node.NodeID == ;//|| node.NodeID == ;
                FP cost=_map.GetAStarNeighbourCost(kid.NodeID,node.NodeID, _goal,_curPosIdx);
                if (g + cost < kid.G)
                {
                    kid.G = g + cost;
                    kid.F = kid.G + kid.H;
                    kid.Parent = node;
                    kid.PreviousCount = node.PreviousCount + 1;
                    _storage.Push(kid);
                    _storage.UpdateOpenSetNodeValue(kid);
                }
            }

            while (_storage.PropagationCount>0)
            {
                AStarNode parent = _storage.Pop();
                if(parent==null)
                {
                    continue;
                }
                c = parent.children.Count;
                for (int i = 0; i < c; i++)
                {
                    kid = parent.children[i];
                    //bool ignoredDynamicCost = kid.NodeID == _goal.TargetNode.NodeID//|| kid.NodeID == _goal.StartNode.NodeID
                    //    || node.NodeID == _goal.TargetNode.NodeID;// || node.NodeID == _goal.StartNode.NodeID;
                    FP cost = _map.GetAStarNeighbourCost(kid.NodeID, node.NodeID, _goal, _curPosIdx);
                    if (parent.G + cost < kid.G)
                    {
                        kid.G = parent.G + cost;
                        kid.F = kid.G + kid.H;
                        kid.Parent = parent;
                        kid.PreviousCount = parent.PreviousCount + 1;
                        _storage.Push(kid);
                       _storage.UpdateOpenSetNodeValue(kid);
                    }
                }
            }
        }
        //
        void LinkChild(AStarNode node, AStarNode temp,int idx)
        {
            //bool ignoredDynamicCost = temp.NodeID == _goal.TargetNode.NodeID //|| temp.NodeID == _goal.StartNode.NodeID
            //    || node.NodeID == _goal.TargetNode.NodeID;// || node.NodeID == _goal.StartNode.NodeID;
            if(node==null || temp==null)
            {
                return;
            }
            FP g = node.G + _map.GetAStarNeighbourCost(temp.NodeID, node.NodeID, _goal, _curPosIdx);// _map.GetAStarNeighbourCost(idx);

            bool bInOpen = _storage.IsInOpenSet(temp);
            if(bInOpen)
            {
                node.children.Add(temp);
                // A better route found,update
                if (g <temp.G)
                {
                    temp.Parent = node;
                    temp.PreviousCount = node.PreviousCount + 1;
                    temp.G = g;
                    temp.F = g + temp.H;
                    _storage.UpdateOpenSetNodeValue(temp);
                }
            }
            else if (_storage.IsInCloseSet(temp))
            {
                node.children.Add(temp);
                if (g < temp.G)
                {
                    temp.Parent = node;
                    temp.PreviousCount = node.PreviousCount + 1;
                    temp.G = g;
                    temp.F = g + temp.H;
                    // The fun part...
                    UpdateParents(temp);
                   // _storage.UpdateOpenSetNodeValue(temp);
                }
            }
            else
            {
                temp.Parent = node;
                temp.PreviousCount = node.PreviousCount + 1;
                temp.G = g;
                temp.H = _goal.CalculateHDist(_goal.TargetNode,temp);
                temp.F = temp.G + temp.H;

                _storage.AddToOpenSet(temp,_map);
                node.children.Add(temp);
            }
        }

        /// <summary>
        /// RunAStar
        /// </summary>
        public void RunAStar()
        {
            _revsCurrentRun = 0;
            PathFinding.AStarAgent agent = _agent as PathFinding.AStarAgent;
            _curPosIdx = -1;
            IInt2 curPos = IInt2.zero;
            if (agent!=null)
            {
                curPos.x = -1;
                curPos.y = -1;
                _curPosIdx = agent._map.GetGridNodeId(agent._behaviour.position);
                curPos=agent._map.GetGridPos(_curPosIdx);
            }            
            while (_goal != null)
            {
                //Get the node with the min f from Storage
                _goal.CurrentNode = _storage.RemoveCheapestOpenNode(_map);
#if UNITY_EDITOR && !MULTI_THREAD
                if (PathFinding.PathFindingManager.DEBUG)
                { 
                        if (_goal.CurrentNode != null)
                        {
                            UnityEngine.Vector3 pos = CustomMath.TsVecToVector3(((PathFinding.GridMap)_map).GetWorldPosition(_goal.CurrentNode.NodeID));
                            UnityEngine.Debug.DrawLine(pos - UnityEngine.Vector3.right * 0.2f, pos + UnityEngine.Vector3.right * 0.2f, UnityEngine.Color.red, 1);
                        }
                }
#endif
                if (_totalRevs > MAX_REVS)//need find another target
                {
                    _goal.CurrentNode = null;
                }
                //Add the node to the closed list
                if (_goal.CurrentNode != null &&_map.CheckNodeValid(_goal.CurrentNode))
                {
                    _storage.AddToClosedSet(_goal.CurrentNode, _map);
                }                   
                else
                {
                    IsWorking = false;
                    break;
                }
                //_currentNode = newNode;
                if (_goal.IsAStarFinished(_goal.CurrentNode))
                {
                    IsWorking = false;
                    break;
                }
                int numberOfNeighbours = _map.GetNeighboursCount(_goal.CurrentNode);
              
                //scan its neighbours
                for (short i = 0; i < numberOfNeighbours; i++)
                {                    
                    AStarNode neighbour = _storage.GetAStarNeighbour(_goal.CurrentNode, i,_searchId,_map,curPos);
                    if(neighbour!=null)
                    {
                        LinkChild(_goal.CurrentNode, neighbour,i);
                    }
                }
#if ASTARDEBUG
                _totalRevs++;
#endif
                if (_maxRevolutions >= 0 && _revsCurrentRun++ >= _maxRevolutions)
                {
                    break;
                }
                if (ShouldPause(_revsCurrentRun))
                    break;
            }
        }
        /**
         * reset
         */
        public void Reset()
        {
            PathFinding.AStarAgent agent = _agent as PathFinding.AStarAgent;
            if (agent!=null && agent._asm!=null)
            {
                _goal = null;
            }
            _goal = null;//.Reset();
            _map = null;//.Reset();
            _storage = null;//.Reset();
#if UNITY_EDITOR && !MULTI_THREAD
            if(PathFinding.PathFindingManager.DEBUG)
            {
                if(_agent!=null &&((PathFinding.AStarAgent)_agent)._asm==this)
                {
                   // UnityEngine.Debug.LogError("reset path error");
                }
            }
#endif
            _inQueue = false;
            _revsCurrentRun = 0;
            _curPosIdx = -1;
            _agent = null;
            _maxRevolutions = C_MaxRevolutions;
            ResetMachineData();
#if ASTARDEBUG
            _totalRevs=0;
#endif
        }
        void ResetMachineData()
        {
#if ASTARDEBUG
            _totalRevs = 0;
#endif
            _searchId = -1;
            _pauseCount = -1;

            _isWorking = false;
        }
    }
}
