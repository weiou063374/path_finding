///author:huwei
///date:2018.4.17
using System.Collections;
using System.Collections.Generic;
using AStarMachine;
using TrueSync;

namespace PathFinding
{
   public enum EPathType
    {
        none,
        quickPath,
        fullPath,
        splicePath,
        priorityPath

        // finished
    }
    public enum EPathReturnStatus
    {
        ok,
        fail,
        wait
    }
    interface IPathAgent
    {
      //  AStarMachine.AStarMachine Asm{ get; set; }
    }
    internal delegate void SearchFinished(AStarMachine.AStarMachine asm, EPathType type,bool canAddRevs);
   public class PathManager//: Singleton<PathManager>
    {
        //  internal WorkerContext context=new WorkerContext();
        public List<PathFindingAgentBehaviour> _agentIdx = new List<PathFindingAgentBehaviour>();
        
        public int _pathId = 0;
        public void AddAgent(PathFindingAgentBehaviour agent)
        {
            _agentIdx.Add(agent);
        }
        public void RemoveAgent(PathFindingAgentBehaviour agent)
        {
            _agentIdx.Remove(agent);
        }
        //
        public Stack<int> _queryStack = new Stack<int>();
        //
        Queue<AStarMachine.AStarMachine> _pathQueue = new Queue<AStarMachine.AStarMachine>();
       // Dictionary<int, AStarMachine.AStarMachine> _dicAgentPath = new Dictionary<int, AStarMachine.AStarMachine>();
        int _revsCompleted = 0;
        int _reqRevs= 0;
        int _preReqRevs = 0;
        readonly int MaxRevsPerGameTick=50* AStarMachine.AStarMachine.C_MaxRevolutions;
        //
        static SingleObjectPool<AStarMachine.AStarMachine> _pathPool = new SingleObjectPool<AStarMachine.AStarMachine>(10);
        static SingleObjectPool<GridAStarStorage> _storagePool = new SingleObjectPool<GridAStarStorage>(10);

        static SingleObjectPool<GridAStarGoal> _goalPool = new SingleObjectPool<GridAStarGoal>(10);
        //public AStarMachine.AStarMachine FindPath()
        //{
        //    AStarMachine.AStarMachine p = _pathPool.New();
        //    return p;
        //}
        public void Loop(FP ft)
        {          
        
          //  return;
            while (_pathQueue.Count > 0 && _revsCompleted < MaxRevsPerGameTick)
            {
               // return;//test
                AStarMachine.AStarMachine thePath = _pathQueue.Peek();
                if (!thePath.IsWorking)
                {
                     _pathQueue.Dequeue();
                    thePath._inQueue = false;
                    ClearFinishedPath(thePath);
                    continue;
                }               
                thePath.RunAStar();
                _revsCompleted += thePath.RevsCurrentRun;
               // UnityEngine.Debug.Log("_revsCompleted:"+ _revsCompleted);
                // If not  working,done
                if (!thePath.IsWorking)
                {
#if UNITY_EDITOR && PATHMANAGER_DEBUG && !MULTI_THREAD
                    if(PathFindingManager.DEBUG)
                    {
                        if (thePath.Agent != null && ((AgentBase)thePath.Agent)._behaviour != null)
                        {
                            PathFindingAgentBehaviour bg = ((PathFindingAgentBehaviour)(((AgentBase)thePath.Agent)._behaviour));
                            string name = bg.name;
                            if (thePath._totalRevs > 1500)
                            {
                                UnityEngine.Debug.LogError("full path fail:" + name + ",target:" +CustomMath.TsVecToVector3((bg.targetPos)).ToString());
                            }
                           // UnityEngine.Debug.Log("full path succeed:" + name + " revs:" + thePath._totalRevs);
                        }
                    }
#endif
                    // Post processing
                    thePath._inQueue = false;
                    _pathQueue.Dequeue();
                    this.ProcessFinishedPath(thePath, EPathType.fullPath,true);
                }
                else
                {
                    if (thePath.MaxRevolutions < MaxRevsPerGameTick)
                    {
                        thePath.MaxRevolutions += 1;
                    }
                    // Take it off the front of the queue
                    // and put it on the rear of the queue.
                    _pathQueue.Enqueue(thePath);
                    thePath._inQueue = true;

                    // Remove
                   // thePath._inQueue = false;
                    _pathQueue.Dequeue();
                }             
            }
        }
        internal bool Enqueue(AStarMachine.AStarMachine m)
        {
            if(m._inQueue)
            {
                return false;
            }
            _pathQueue.Enqueue(m);
            m._inQueue = true;
            return true;
        }

        void StartFindingPath(AStarMachine.AStarMachine asm, TSVector start, TSVector target,bool bRunDirect,int curPosIdx)
        {         
            
            //-----------------
            int sIdx = ((GridMap)asm.Map).GetGridNodeId(start);// map.GetIdx(start.x, start.y);
            int tIdx = ((GridMap)asm.Map).GetGridNodeId(target);// map.GetIdx(target.x, target.y);      
            asm.StartSearch(sIdx, tIdx, bRunDirect, start,_pathId++,curPosIdx);
            asm.MaxRevolutions = AStarMachine.AStarMachine.C_MaxRevolutions;//reset revolution
        }
        public bool RequestRevs(TSVector start, TSVector target,int reqRevs,int preRevsAvailable)
        {          
            if (start == target)//|| !(asm.Map is GridMap)
            {
                return false;
            }       
            if (MaxRevsPerGameTick <= _revsCompleted)
            {
                _reqRevs += reqRevs;
                return false;
            }
            int revs = AStarMachine.AStarMachine.C_MaxRevolutions;
            if (reqRevs== AStarMachine.AStarMachine.MAX_REVS && _revsCompleted< MaxRevsPerGameTick/2)//priorityPath
            {
                revs = reqRevs;
                _revsCompleted += revs;
                _reqRevs += revs;
                return true;
            }
            if(_preReqRevs> MaxRevsPerGameTick && preRevsAvailable>0)//next frame
            {
                _reqRevs += reqRevs;
                return false;
            }
            _reqRevs += reqRevs;
            // if (_pathQueue.Count > 0)
            {
                revs = System.Math.Min(MaxRevsPerGameTick - _revsCompleted, reqRevs);
            }
            if(revs<reqRevs)
            {
                return false; 
            }
            //asm.MaxRevolutions = revs;
            _revsCompleted += revs;// asm.RevsCurrentRun;
            return true;
        }
        public void Reset()
        {
            _revsCompleted = 0;
            _preReqRevs = _reqRevs;
            _reqRevs = 0;
          //  _agentIdx.Clear();
           // _pathQueue.Clear(); ...................----------------todo
        }
        public void ClearPath()
        {
            _pathQueue.Clear();
        }
        public bool CanFindPathNow()
        {
            return _revsCompleted < AStarMachine.AStarMachine.MAX_REVS;
        }
        public void CreateAstarPath(AStarAgent agent ,TSVector start, TSVector target, GridMap map,int curPosIdx)
        {
            if (agent._asm==null)
            {
                AStarMachine.AStarMachine asm = _pathPool.New();
                GridAStarGoal goal = _goalPool.New();
                goal.Ini(agent);

                GridAStarStorage storage = _storagePool.New();
                storage.Ini();
                asm.Ini(agent, goal, storage, map);

                agent._asm = asm;
                StartFindingPath(asm, start,target,false, curPosIdx);
            }
               
        }
        /// <summary>
        /// three step:first step,find fast path,maybe not a valid path navigating to target position 
        /// </summary>
        /// <param name="agent"></param>
        /// <param name="start"></param>
        /// <param name="target"></param>
        /// <param name="mapId"></param>
        /// <param name="isTried">is tried to find quick path and finished </param>
        /// <returns></returns>
        public EPathReturnStatus FindQuickPath(AgentBase agent,int revs, TSVector start, TSVector target,GridMap map,bool isTried)//, List<TSVector> vecPath
        {         
            //GridMap map = GridMapManager.instance.GetMap(mapId);
            if (map.GetGridNodeId(start) == map.GetGridNodeId(target))
            {
                return EPathReturnStatus.ok;
            }
            AStarAgent aAgent = ((AStarAgent)agent);
            AStarMachine.AStarMachine asm = isTried?null: aAgent._asm;
            bool bInqueue = asm != null &&asm.IsWorking ;
            if (asm==null)
            {
                asm = _pathPool.New();
                GridAStarGoal goal = _goalPool.New();
                goal.Ini(agent);

                GridAStarStorage storage = _storagePool.New();
                storage.Ini();
                asm.Ini(agent, goal, storage, map);
            }
            int curIdx = map.GetGridNodeId(aAgent._behaviour.position);
            StartFindingPath(asm,start,target,true, curIdx);
           
            if (asm.IsWorking)
            {
                int startIdx = map.GetGridNodeId(start);
                int tIdx = map.GetGridNodeId(target);
                IInt2 pos = map.GetGridPos(tIdx);
                bool isReachable = map.IsExistingNeighbourWalkable(pos.x, pos.y, startIdx, tIdx, curIdx);
                if (!isTried && isReachable)
                {
                    ProcessFinishedPath(asm, EPathType.quickPath,false);//quick path is valid
                }
              
                if(!isReachable)
                {
                    aAgent._behaviour.AddUnReachablePos(map, tIdx);
                }
                if (!isTried && isReachable)//target possible reachable
                {
                    int sIdx = map.GetGridNodeId(aAgent._vecRunningPath[0]);
                    asm.StartSearch(sIdx, tIdx, false, start, _pathId++, startIdx);//
                    //if (!bInqueue)
                    //{
                    //    _pathQueue.Enqueue(asm);//find full path
                    //}
                    aAgent._asm = asm;
                }
                else//can't reach target
                {
                    asm.IsWorking = false;
                   // asm.Reset();
                    ClearFinishedPath(asm);

                    if (!isTried)
                    {
                        return EPathReturnStatus.fail;
#if UNITY_EDITOR && !MULTI_THREAD

                        if (PathFindingManager.DEBUG)
                        { 
                            UnityEngine.Debug.LogWarning("can't reach target:" + pos.x + "," + pos.y);
                        }
#endif
                    }
                    else
                    {
                        return EPathReturnStatus.fail;
                    }
                 
                }
                // asm._start=
            }
            else
            {
                if(isTried)//remove working asm
                {
                    if(aAgent._asm!=null)
                    {
                        aAgent._asm.IsWorking = false;
                        ClearFinishedPath(aAgent._asm);
                    }
                    aAgent._asm = asm;
                }
                ProcessFinishedPath(asm, EPathType.quickPath);//quick path is valid
            }
            if(aAgent._vecRunningPath.Count>0)
            {
                return EPathReturnStatus.ok;
            }
            return EPathReturnStatus.fail;//warning
        }
        /// <summary>
        /// find a splice path between quick path and full path,return finding splice path succeed or not
        /// </summary>
        internal bool FindSplicePath(AStarMachine.AStarMachine asm, TSVector start, TSVector target,
            bool canAddRevs,int curPosIdx)
        {
            int revs = AStarMachine.AStarMachine.C_MaxRevolutions*2;
            asm.SetPauseCount(-1);
            asm.MaxRevolutions = revs;
            
            if(canAddRevs)//from queue
            {
                _revsCompleted += revs;  //two case:1.from queue;from main thread or seperate thread ,2.from priority:the revs is enghou
            }
            int sIdx = ((GridMap)asm.Map).GetGridNodeId(start);// map.GetIdx(start.x, start.y);
            int tIdx = ((GridMap)asm.Map).GetGridNodeId(target);// map.GetIdx(target.x, target.y);
            asm.StartSearch(sIdx, tIdx,true, start, _pathId++, curPosIdx);
            //if(asm.IsWorking)
            //{
            //    UnityEngine.Debug.LogError("FindSplicePath fail to target with revs:"+((AgentBase)(asm.Agent)).gameObject.name);
            //   // ((AStarAgent)(asm.Agent)).MergePath(EPathType.fullPath);
            //}
            //else
            //{
            //  //  ProcessFinishedPath(asm, EPathType.splicePath);
            //}
            // asm.IsWorking = false;      
            return !asm.IsWorking;
        }
        /// <summary>
        /// when the agent will reach quick path target ,and is still finding the full path,
        /// then request a priority path to finish the full path finding
        /// </summary>
        /// <param name="agent"></param>
        internal void DoPriorityPath(AgentBase agent)
        {
            //if(_revsCompleted>AStarMachine.AStarMachine.MAX_REVS)
            //{
            //    return;
            //}
            //
           // UnityEngine.Debug.Log(agent.gameObject.name+ " DoPriorityPath");
            AStarMachine.AStarMachine thePath = ((AStarAgent)agent)._asm;
        
            if (thePath != null &&thePath.IsWorking)
	        {
		        // not to pause
		        thePath.SetPauseCount(-1);
                // 
                thePath.AddToMaxRevolutions(AStarMachine.AStarMachine.MAX_REVS);//
                //
                thePath.RunAStar();
                //_revsCompleted += thePath.RevsCurrentRun;
#if UNITY_EDITOR&& PATHMANAGER_DEBUG && !MULTI_THREAD
                if(PathFindingManager.DEBUG)
                {
                    if (thePath.IsWorking)
                    {
                        UnityEngine.Debug.LogError("DoPriorityPath failed:" + ((PathFindingAgentBehaviour)(agent._behaviour)).name);
                    }
                    else
                    {
                        UnityEngine.Debug.Log("DoPriorityPath succeed:" + ((PathFindingAgentBehaviour)(agent._behaviour)).name + " revs:" + thePath._totalRevs);
                    }
                }              
#endif
                thePath.IsWorking = false;
                ProcessFinishedPath((AStarMachine.AStarMachine)thePath,EPathType.fullPath,false);
      
               // ClearFinishedPath(thePath);
            }
        }
        //public void RemovePath(IPathAgent agent)
        //{
        //    if(agent!=null)
        //    {
        //        _pathPool.Store((AStarMachine.AStarMachine)agent.Asm);
        //        agent.Asm = null;
        //    }
        //}
        internal void ClearFinishedPath(AStarMachine.AStarMachine path)
        {
            if(path.Agent!=null)
            {
                AStarAgent agent =path.Agent as AStarAgent;
                if(agent._asm==path)
                {
                    agent._asm = null;
                }
            }         
        
            if(path.Goal!=null && !path.IsWorking && !path._inQueue)    //avoid storing the same obj repeatly
            {
                _goalPool.Store((GridAStarGoal)path.Goal);
                _storagePool.Store((GridAStarStorage)path.Storage);
                _pathPool.Store(path);
            }      
        }

        internal void ProcessFinishedPath(AStarMachine.AStarMachine path,EPathType pathType,bool canAddRevs=false)//todo
        {
            if (path.Agent != null)
            {
                AgentBase agent = path.Agent as AgentBase;
                if (agent != null && agent.searchFinished != null)
                {
                    agent.searchFinished(path,pathType, canAddRevs);
                }
                if(!path.IsWorking)
                {
                    ((AStarAgent)agent)._asm = null;
                } 
            }
            if (!path.IsWorking && !path._inQueue)
            {
                ClearFinishedPath(path);
            }
        }
        
    }
}

