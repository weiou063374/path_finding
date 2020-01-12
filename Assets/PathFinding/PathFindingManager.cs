///author:huwei
///date:2018.4.17
//#define SEARCH_ENEMY
//#define MULTI_THREAD
#define USING_DYNAMIC_TREE
using System;
using System.Collections.Generic;
using System.Collections;
using TrueSync;
using System.Threading;
namespace PathFinding
{
    public delegate bool IsWalkable(int x, int y, bool SkipDynamic);
    public delegate bool IsPosWalkable(TSVector pos);

#if _UNITY_EDITOR
    public class PathFindingManager : Singleton<PathFindingManager>
#else
    public class PathFindingManager
#endif
    {
        public const int C_FrameCountPerUpdate = 1;
        public const bool c_useSmoothPath = true;
#if PATHMANAGER_DEBUG
        public const bool DEBUG = true;
#else
        public const bool DEBUG = false;
#endif
        public const bool StopToNodeCenter = false;
        public const bool StopToSpecialPos = false;
#if DYNAMIC_FORCE
        public const bool c_useAvoidUnit = true;
#else
        public const bool c_useAvoidUnit = false;
#endif
        // bool _destroyed = false;
#if USING_DYNAMIC_TREE
        DynamicTree<PathFindingAgentBehaviour> _dynamicTree = null;
#else
        Dictionary<int, Quadtree<QTData>> _dicQdTree = new Dictionary<int, Quadtree<QTData>>();//key:camp
        Dictionary<int, List<IAgentBehaviour>> _agents = new Dictionary<int, List<IAgentBehaviour>>();
        List<int> _camps = new List<int>();
#endif
        // public List<PathFindingAgentBehaviour> _allAgents = new List<PathFindingAgentBehaviour>();      
        int _allAgentCount = 0;

        HashSet<int> _hsSpecPos = new HashSet<int>();
        public EAgentLoopStep _eCurrentAgentLoopStep = EAgentLoopStep.max;
        public List<AgentsTask> _tasks = null;
        //  public ManualResetEvent[] _taskEvents = null;
        public FlowFieldManager _flowManager = null;
        internal PathManager[] _pathManagers = null;
        //public int _threadFinishedCount = 0;
        // int _mSIdx;//main thread start idx in _allAgents
        //  int _mEIdx;//main thread end idx in _allAgents
        public PathFindingManager(FP scale)
        {
            _tasks = new List<AgentsTask>();
            _flowManager = new FlowFieldManager();
            int taskCount = 0;
       //     _destroyed = false;
#if MULTI_THREAD
            int cps;
            ThreadPool.GetMinThreads(out taskCount, out cps);
            //#if UNITY_EDITOR
            UnityEngine.Debug.LogError("thread count______:" + taskCount);
            //#endif
            taskCount = 2;
#endif
            //  _taskEvents = new ManualResetEvent[taskCount];
            _pathManagers = new PathManager[taskCount + 1];
            int i = 0;
            for (i = 0; i < taskCount; i++)
            {
                AgentsTask task = AgentsTask.Create(i, this);
                _tasks.Add(task);
                // _taskEvents[i] = task._finished;
                _pathManagers[i] = new PathManager();
            }
            _pathManagers[i] = new PathManager();
#if USING_DYNAMIC_TREE
            _dynamicTree = new DynamicTree<PathFindingAgentBehaviour>();
            _dynamicTree.c_aabbExtension *= scale;
#endif
        }
#if !USING_DYNAMIC_TREE
        public Quadtree<QTData> GetQDTree(int campId) {
                Quadtree<QTData> tree;
                _dicQdTree.TryGetValue(campId, out tree);
                return tree;
        }
        void BuildQuadtree( List<IAgentBehaviour> agents, Quadtree<QTData> tree)
        {
           // foreach(KeyValuePair< int, List < IAgentBehaviour >> kv in _agents)
            {      
                
                tree.Clear();
                if (agents.Count > 0)
                {
                    QTBound bounds = QTBound.MinMaxQTBound(agents[0].position, agents[0].position);
                    int count = agents.Count;
                    for (int i = 1; i < count; i++)
                    {
                        //if(agents[i]==null || agents[i]==null)
                        //{
                        //    continue;
                        //}
                        TSVector p = agents[i].position;
                        bounds = QTBound.MinMaxQTBound(TSVector.Min(bounds.min, p), TSVector.Max(bounds.max, p));
                    }
                    tree.SetBounds(bounds);

                    for (int i = 0; i < count; i++)
                    {
                        tree.Insert(agents[i]);
                    }
                  //  tree.DebugDraw();
                }
                tree.CalculateSpeeds();
            }
          
        }
#endif

        public void RefreshAllPath()
        {
#if USING_FLOW_FIELD
            if (_flowManager != null)
            {
                _flowManager.UpdateFlowFields();
            }
#endif
            int taskCount = _tasks.Count;
            int pathMcount = taskCount + 1;
            for (int k = 0; k < pathMcount; k++)
            {
                int aCount = _pathManagers[k]._agentIdx.Count;
                for (int j = 0; j < aCount; j++)
                {
                    if (_pathManagers[k]._agentIdx[j].AgentType == EAgentType.astar)
                    {
                        AStarAgent aa = _pathManagers[k]._agentIdx[j].agent as AStarAgent;
                        if (aa != null)
                        {
                            aa.ClearPath();
                        }
                    }
                }
                _pathManagers[k].Reset();
                _pathManagers[k].ClearPath();
            }
        }
        public void AddCamp(System.Int64 camp)
        {
#if !USING_DYNAMIC_TREE
            if (!_agents.ContainsKey(camp))
            {
                _agents[camp] = new List<IAgentBehaviour>();
            }
            if (!_dicQdTree.ContainsKey(camp))
            {
                _dicQdTree[camp] = new Quadtree<QTData>();
                _camps.Add(camp);
                _agentUpdateNeighbourIdx[camp] = 0;
            }
#endif
        }

        //
        public void AddAgent(IAgentBehaviour agent)
        {
            PathFindingAgentBehaviour af = agent as PathFindingAgentBehaviour;
#if !USING_DYNAMIC_TREE
            List<IAgentBehaviour> list;
            if(!_agents.TryGetValue(agent.campId,out list))//
            {
                list = new List<IAgentBehaviour>();
                _agents[agent.campId] =list;
            }
            Quadtree<QTData> tree;
            if (!_dicQdTree.TryGetValue(agent.campId, out tree))
            {
                tree = new Quadtree<QTData>();
                _dicQdTree[agent.campId] = tree;
                _camps.Add(agent.campId);
                _agentUpdateNeighbourIdx[agent.campId] = 0;
            }
            list.Add(agent);
#else
            AABB ab = af.aabb;
            af.proxyId = _dynamicTree.CreateProxy(ref ab, af);
#endif
            _allAgentCount++;

            int pathMcount = _pathManagers.Length;
            int idx = _allAgentCount % (pathMcount);

            af.pathManager = _pathManagers[idx];
            af.pathManager.AddAgent(af);
            // _allAgents.Add(af);
        }
        public void RemoveAgent(IAgentBehaviour agent)
        {
            PathFindingAgentBehaviour af = agent as PathFindingAgentBehaviour;
#if !USING_DYNAMIC_TREE
            List<IAgentBehaviour> list;
            if (_agents.TryGetValue(agent.campId, out list))
            {
                list.Remove(agent);
            }
#else
            if (agent.proxyId >= 0 && _dynamicTree!=null)
            {
                _dynamicTree.DestroyProxy(af.proxyId);
            }
            af.proxyId = -1;
#endif
            af.pathManager.RemoveAgent(af);
            af.pathManager = null;
            // _allAgents.Remove(af);
            _allAgentCount--;
        }
        //
        FP _deltaTime;
        int _curTime;
        public FP DeltaTime { get { return _deltaTime; } }

        public int CurTime
        {
            get
            {
                return _curTime;
            }

            set
            {
                _curTime = value;
            }
        }
        //
        Dictionary<int, int> _agentUpdateNeighbourIdx = new Dictionary<int, int>();
        public int _updateIdx = 0;
        //int _totalAgentCount = 0;
#if !USING_DYNAMIC_TREE
        //deprecated
        public void CalculateNeighbours()
        {
            if (_camps.Count > 0)
            {
                foreach (KeyValuePair<int, Quadtree<QTData>> qdTree in _dicQdTree)
                {
                    int nIdx = _agentUpdateNeighbourIdx[qdTree.Key];
                    int uCount = System.Math.Min(_allAgentCount, 8);
                    for (int i = 0; i < uCount; i++)
                    {
                        bool bBuildQTree = nIdx == 0;
                        nIdx = nIdx % _allAgentCount;
                        if (bBuildQTree)
                        {
                            BuildQuadtree(_agents[qdTree.Key], qdTree.Value);
                        }
                        List<IAgentBehaviour> agents = _agents[_camps[0]];
                        int cCount = _camps.Count;
                        int cIdx = 0;
                        int tCount = 0;
                        int curIdx = 0;
                        for (cIdx = 0; cIdx < cCount; cIdx++)
                        {
                            curIdx = nIdx - tCount;
                            tCount += _agents[_camps[cIdx]].Count;
                            if (tCount - 1 >= nIdx)
                            {
                                agents = _agents[_camps[cIdx]];
                                break;
                            }
                        }
                        //if(curIdx>agents.Count-1)
                        //{
                        //    continue;
                        //}
                        agents[curIdx].CalculateNeighbours(qdTree.Key, qdTree.Value);
                        nIdx = (nIdx + 1) % _allAgentCount;
                    }
                    _agentUpdateNeighbourIdx[qdTree.Key] = nIdx;
                }
            }
        }
                   //deprecated
        public void UpdateAgents(int frameTime)
        {
            _updateIdx = (_updateIdx + 1) % C_FrameCountPerUpdate;
            PathManager pm = _pathManagers[_pathManagers.Length - 1];
            foreach (KeyValuePair<int, List<IAgentBehaviour>> kv in _agents)
            {
                List<IAgentBehaviour> agents = kv.Value;
                int agentCount = agents.Count;
                for (int i = agentCount - 1; i >= 0; i--)
                {
                    IAgentBehaviour agent = agents[i];
                    if (agent != null)
                    {
                        agent.Loop(frameTime, _updateIdx != i % C_FrameCountPerUpdate);
                    }
                }

            }
        }
#endif
#if !USING_DYNAMIC_TREE
        public void CalculateAgentNeighbours(IAgentBehaviour agent)
        {

            foreach (KeyValuePair<int, Quadtree<QTData>> qdTree in _dicQdTree)
            {
                agent.CalculateNeighbours(qdTree.Key, qdTree.Value);
            }
        }
#else
        public PathFindingAgentBehaviour GetAgentByProxyId(int id)
        {
            return _dynamicTree.GetUserData(id);
        }
        public void MoveProxy(PathFindingAgentBehaviour agent, TSVector diff)
        {
            AABB ab = new AABB();
            ab.lowerBound = new TSVector2(agent.position.x - agent.colliderRadius, agent.position.z - agent.colliderRadius);
            ab.upperBound = new TSVector2(agent.position.x + agent.colliderRadius, agent.position.z + agent.colliderRadius);
            TSVector2 displace = CustomMath.TSVecToVec2(diff);
            if (agent.proxyId < 0)
            {
                return;
            }
            _dynamicTree.MoveProxy(agent.proxyId, ref ab, ref displace);
        }
        public bool HasNeighbours(TSVector pos, FP radius, Stack<int> queryStack)
        {
            AABB ab = new AABB();
            ab.lowerBound = new TSVector2(pos.x - radius, pos.z - radius);
            ab.upperBound = new TSVector2(pos.x + radius, pos.z + radius);
            return _dynamicTree.Query(null, ref ab, queryStack, true);
        }
        public bool HasNeighboursBetween2Point(int excludeProxyId, TSVector pos1, TSVector pos2, Stack<int> queryStack)
        {
#if USING_DYNAMIC_TREE
            TSVector2 vec1 = CustomMath.TSVecToVec2(pos1);
            TSVector2 vec2 = CustomMath.TSVecToVec2(pos2);
            if (_dynamicTree != null)
            {
                RayCastInput rc = new RayCastInput();
                rc.p1 = vec1;
                rc.p2 = vec2;
                rc.maxFraction = 1;// GridMap.nodeSize * FP.EN1 * 5;/// 20*nodeSize;
                int proxyId = _dynamicTree.RayCast(null, ref rc, queryStack, true, excludeProxyId);
                if (proxyId >= 0)
                {
                    // PathFindingAgentBehaviour blockedAgent = _dynamicTree.GetUserData(proxyId);
                    return true;
                }
            }
            return false;
#else
            return false;
#endif
        }
        public void CalculateIntersectAgents(AABB rect, DynamicTreeQueryCallback callback, Stack<int> queryStack)
        {
            queryStack.Clear();
            if (_dynamicTree != null)
            {
                _dynamicTree.Query(callback, ref rect, queryStack, false);
            }
        }
        public void CalculateAgentNeighbours(PathFindingAgentBehaviour agent, DynamicTreeQueryCallback callback, FP radius)
        {
            if (agent.proxyId < 0)
            {
                return;
            }
            AABB ab = new AABB();
            ab.lowerBound = new TSVector2(agent.position.x - radius, agent.position.z - radius);
            ab.upperBound = new TSVector2(agent.position.x + radius, agent.position.z + radius);
            agent.pathManager._queryStack.Clear();
            if (_dynamicTree != null)
            {
                _dynamicTree.Query(callback, ref ab, agent.pathManager._queryStack, false);
            }      
        }
        public void CalculateColliders(TSVector pos, FP radius, Stack<int> queryStack, DynamicTreeQueryCallback callback)
        {
            if(_dynamicTree==null)
            {
                return;
            }
            AABB ab = new AABB();
            ab.lowerBound = new TSVector2(pos.x - radius, pos.z - radius);
            ab.upperBound = new TSVector2(pos.x + radius, pos.z + radius);
            _dynamicTree.Query(callback, ref ab, queryStack, false);
        }
#endif
        public void Update(int frameTime)//test
        {
            _curTime += frameTime;
            _deltaTime = frameTime * CustomMath.MsToSecond;
            //
#if !USING_DYNAMIC_TREE
            foreach (KeyValuePair<int, Quadtree<QTData>> qdTree in _dicQdTree)
            {
                BuildQuadtree(_agents[qdTree.Key], qdTree.Value);
            }
#else

#endif
            // CalculateNeighbours();
            //UpdateAgents
            UpdateAgentsWithMultiThread(_deltaTime);
            //PathManager.instance.Loop(frameTime);
            //   FlowFieldManager.instance.Loop(frameTime);//update by PathFingdingAgentBehaviour
        }
        public void UpdateAgentsWithMultiThread(FP deltaTime)
        {
            int taskCount = _tasks.Count;
            //int agentCount = _allAgents.Count;
            //int perACount = agentCount / (taskCount + 1);//plus main thread
            //for (int i = 0; i < taskCount; i++)
            //{
            //    //int leftCount = agentCount - i * perACount;
            //    _tasks[i].SetIdx(i * perACount, perACount);
            //}
            //  _mSIdx = (taskCount) * perACount;
            // _mEIdx = agentCount - 1;
            //
            int pathMcount = taskCount + 1;
            for (int i = 0; i < pathMcount; i++)
            {
                _pathManagers[i].Reset();
            }
            //---------------------------
            int eCount = (int)EAgentLoopStep.max;
            for (int i = 0; i < eCount; i++)
            {
                _eCurrentAgentLoopStep = (EAgentLoopStep)i;
                //   UnityEngine.Profiling.Profiler.BeginSample("Path finding update" +i);
                if (_eCurrentAgentLoopStep == EAgentLoopStep.checkAction
                   // || _eCurrentAgentLoopStep == EAgentLoopStep.checkAvailabelRevs
                    || _eCurrentAgentLoopStep == EAgentLoopStep.FinalMoveHandle
                   // || _eCurrentAgentLoopStep == EAgentLoopStep.postFindPath
                   )
                {
                    //main thread
                    //for (int k = 0; k <= agentCount - 1; k++)
                    for (int k = 0; k < pathMcount; k++)
                    {
                        int aCount = _pathManagers[k]._agentIdx.Count;
                        for (int j = 0; j < aCount; j++)
                        {
                            _pathManagers[k]._agentIdx[j].MultiThreadLoop(deltaTime, false, _eCurrentAgentLoopStep);
                        }
                    }
                }
                else//multi thread:tasks +main thread
                {
                   // _threadFinishedCount = 0;
                    for (int j = 0; j < taskCount; j++)
                    {
                        _tasks[j].Execute(0);
                        // ThreadPool.QueueUserWorkItem(_tasks[j].AgentLoop);                      
                    }
#if PATHMANAGER_DEBUG
                    try
                    {
#endif
                    PathManager pm = _pathManagers[pathMcount - 1];
                    int aCount = pm._agentIdx.Count;
                    for (int j = 0; j < aCount; j++)
                    {
                        pm._agentIdx[j].MultiThreadLoop(deltaTime, false, _eCurrentAgentLoopStep);
                    }
#if PATHMANAGER_DEBUG
                    }
                    catch (Exception e)
                    {
#if UNITY_EDITOR
                        UnityEngine.Debug.LogError(e.ToString()+e.StackTrace);
#endif
                    }
#endif

                    if (taskCount > 0)
                    {
                        //  WaitHandle.WaitAll(_taskEvents);
                        for (int j = 0; j < taskCount; j++) _tasks[j]._finished.WaitOne();
                       // CheckThread(taskCount);
                    }
                }
                //  UnityEngine.Profiling.Profiler.EndSample();
            }
            // UnityEngine.Profiling.Profiler.BeginSample("_pathManagers loop");
           // _threadFinishedCount = 0;
            for (int i = 0; i < pathMcount; i++)
            {
                if (i < taskCount)
                {
                    _tasks[i].Execute(1);
                    //ThreadPool.QueueUserWorkItem(_tasks[i].PathMangerUpdate);
                }
                else
                {
                    _pathManagers[i].Loop(deltaTime);
                }
            }

            //if (taskCount > 0)
            //{
            //    WaitHandle.WaitAll(_taskEvents);
            //}
            if (taskCount > 0)
            {
                //  WaitHandle.WaitAll(_taskEvents);
                 for (int j = 0; j < taskCount; j++) _tasks[j]._finished.WaitOne();
               // CheckThread(taskCount);
            }
            
            //   UnityEngine.Profiling.Profiler.EndSample();
        }
//        void CheckThread(int taskCount)
//        {
//            while (!_destroyed)
//            {
//                bool bOk = _threadFinishedCount == taskCount;
//                //for (int j = 0; j < taskCount; j++)
//                //{
//                //    if (!_tasks[j]._finished)
//                //    {
//                //        bOk = false;
//                //        break;
//                //    }
//                //}
//                if (bOk)
//                {
//                    break;
//                }
//                //Thread.SpinWait(1);
//                Thread.Sleep(0);
//#if UNITY_EDITOR
//                if (!UnityEngine.Application.isPlaying)
//                {
//                    for (int i = 0; i < taskCount; i++)
//                    {
//                        if (i < taskCount)
//                        {
//                            _tasks[i].terminate=true;
//                        }
//                    }
//                    break;
//                }
//#endif
//            }
//        }
        public bool ContainSpecPos(int idx)
        {
            return _hsSpecPos.Contains(idx);
        }
        public void AddSpecPos(int idx)
        {
            _hsSpecPos.Add(idx);
        }
        public void RemoveSpecPos(int idx)
        {
            _hsSpecPos.Remove(idx);
        }
        public void TerminateThread()
        {
            int taskCount = _tasks.Count;
              for (int j = 0; j < taskCount; j++)
            {
                _tasks[j].Terminate();                  
            }
        }

        public void OnDestroy()
        {
#if !USING_DYNAMIC_TREE
            _dicQdTree.Clear();
            _agents.Clear();
#else
            if (_dynamicTree != null)
            {
                _dynamicTree.Destroy();
            }
            _dynamicTree = null;
#endif

            TerminateThread();
           // _destroyed = true;
            // _allAgents.Clear();
            _hsSpecPos.Clear();
            _flowManager = null;
            _allAgentCount = 0;
            //  _mSIdx = 0;
            // _mEIdx = 0;
            _deltaTime = CurTime = 0;
            _pathManagers = null;
            //  _taskEvents = null;
            if (_tasks != null)
            {
                int count = _tasks.Count;
                for (int i = 0; i < count; i++)
                {
                    _tasks[i].terminate = true;
                }
                _tasks.Clear();
            }
            _updateIdx = 0;
            _eCurrentAgentLoopStep = EAgentLoopStep.max;

        }
    }
    public enum EAgentLoopStep
    {
      //  resetBuffData = 0,//multi threads
                          // calcualteNeighbourAgents,
#if SEARCH_ENEMY
        calcualteEnemyAgents,//multi threads
#endif
        checkAction=0,//main thread
#if SEARCH_ENEMY
        findTarget,//multi threads
#endif
       // checkAvailabelRevs, //main thread
        findPath,//multi threads
      //  postFindPath,//multi threads
     //   setDesiredPos,//multit threads
        CalculateVelocity,//multi threads
        FinalMoveHandle,////main thread
        max,
    }
    public class AgentsTask
    {
        // int _startIdx;
        //  int _count;
        int _pathIdx;
        public ManualResetEvent _finished;
        public ManualResetEvent _runFlag;
        PathFindingManager _pfm;
        public int _threadHanldeStep;
        Thread thread;
        public bool terminate;
        internal static AgentsTask Create(int idx, PathFindingManager pfm)
        {
            AgentsTask task = new AgentsTask();
            task._finished =  new ManualResetEvent(true);
            task._runFlag = new ManualResetEvent(false); // new AutoResetEvent(false);
            task._pathIdx = idx;
            task._pfm = pfm;
            task._threadHanldeStep = -1;
            task.terminate = false;
            task.thread = new Thread(new ThreadStart(task.ThreadHandle));
            // task.thread.Priority = ThreadPriority.Highest;
            task.thread.Start(); 
            // task.context = new WorkerContext();
            return task;
        }
        public void SetIdx(int sIdx, int count)
        {
            // _startIdx = sIdx;
            //  _count = count;
        }
        internal void Execute(int step)
        {
            _threadHanldeStep = step;
            _finished.Reset();
            _runFlag.Set();
        }
        public void Terminate()
        {
            //if (!terminate) _finished.WaitOne();
            terminate = true;
            Execute(-1);
        }
        internal void ThreadHandle()
        {
            //while (!_runFlag)
            //{
            //     Thread.Sleep(0);
            //   // Thread.SpinWait(1);
            //}
            _runFlag.WaitOne();
            _runFlag.Reset();
            while (!terminate)
            {
                //try
                //{
                if (_threadHanldeStep == 0)
                {
                    AgentLoop();
                }
                else if (_threadHanldeStep == 1)
                {
                    PathMangerUpdate();
                }
                //  }
                //                catch (System.Exception e)
                //                {
                ////#if UNITY_EDITOR
                //                    UnityEngine.Debug.LogError(e);
                ////#endif
                //                }
                _finished.Set();           
                //lock (_pfm)
                //{
                //    _pfm._threadFinishedCount++;
                //}
                _runFlag.WaitOne();
                _runFlag.Reset();// = false;
//#if UNITY_EDITOR
//                DateTime preTime = System.DateTime.Now.AddSeconds(10);
//#endif
//                while (!_runFlag && !terminate)
//                {
//                    Thread.Sleep(0);
//                    //Thread.SpinWait(1);
//#if UNITY_EDITOR
//                    if(System.DateTime.Now.CompareTo(preTime) >0)
//                    {
//                        break;
//                    }
//#endif
//                }
            }
        }
        internal void AgentLoop()
        {

            //  int eIdx = _startIdx + _count;
            PathManager pm = _pfm._pathManagers[_pathIdx];
            EAgentLoopStep eStep = _pfm._eCurrentAgentLoopStep;
            // for (int i = _startIdx; i < eIdx; i++)
            {

                {

                    int aCount = pm._agentIdx.Count;
                    for (int j = 0; j < aCount; j++)
                    {
                        pm._agentIdx[j].MultiThreadLoop(_pfm.DeltaTime, false, eStep);
                    }
                }
            }
        }
        internal void PathMangerUpdate()
        {
            {
                PathManager[] pathMangers = _pfm._pathManagers;
                pathMangers[_pathIdx].Loop(_pfm.DeltaTime);
            }
        }
    }
}
