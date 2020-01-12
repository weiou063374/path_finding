using System;
using System.Collections.Generic;
using UnityEngine;
//using UnityEditor;
using PathFinding;
using System.Collections;
using TrueSync;


    public class TestPathFinding : MonoBehaviour
    {
        public static SingleObjectPool<PathFindingAgentBehaviour> s_AstarAgent = null;
        public TestAgent agent;

        //public Vector3 destination = new Vector3(48, 0, 48);
        public GameObject[] destObj = null;
        public GameObject[] startObj = null;

        public Transform PathValues;
        public Transform Units;
        public int maxSpeed = 3;
        PathManager _pm = null;
        public static PathFindingManager _pfm = null;
        GridMap _map = null;
        public int enabledTags = -1;
        public int randomSeed = 0;
        public int frameIdx = 0;

        [Tooltip("frame sync can't use coroutine")]
        public bool FrameSyncTest = false;
        public WTWGAME.GridGraph _gridGraph;
        private void Awake()
        {
            GridMap.SetNodeSize(1,1);
            TSRandom.Init();
            _pfm = new PathFindingManager(1);
            //_pm = PathManager.Instance;
            _gridGraph._mapSizeX = 50;
            _gridGraph._mapSizeZ = 50;
            _gridGraph._startPos = TSVector.zero;
            _gridGraph.SetUpMap();
   
            _map = _gridGraph._map;//GridMapManager.Instance.CreateGridMap(50, 50, TSVector.zero, 0, this.IsTileUnWalkable);
            PathFindingAgentBehaviour.C_DESIRED_DELTATIME =18;
            s_AstarAgent = new SingleObjectPool<PathFindingAgentBehaviour>(10); 
        }
        //public bool IsTileUnWalkable(IInt2 pos)
        //{
        //    int nodeIdx = _map.GetIdx(pos.x,pos.y);
        //    Pathfinding.GridGraph gg = Pathfinding.GridNode.GetGridGraph(0);
        //    Pathfinding.GridNode node = gg.nodes[nodeIdx];
        //    return !(node.Walkable && (enabledTags >> (int)node.Tag & 0x1) != 0);
        //}
        private void Start()
        {

            //GameObject unit=AssetDatabase.LoadAssetAtPath<GameObject>("Assets/unit.prefab");          
            // StartCoroutine(SpawnUnit());
            Framework.CoroutineScheduler.instance.StartCoroutine(SpawnUnit());
        }
        void OnTap(TapGesture gesture)
        {
        return;
            Ray ray= Camera.main.ScreenPointToRay(gesture.Position);

            int mask =1<< LayerMask.NameToLayer("terrain");
             RaycastHit hit;
            if(UnityEngine.Physics.Raycast(ray,out hit, 1000, mask))
            {
                TSVector pos= TSVector.zero;
                pos.Set(CustomMath.FloatToFP(hit.point.x), CustomMath.FloatToFP(hit.point.y), CustomMath.FloatToFP(hit.point.z));
                TSVector targetPos= pos;
                foreach (TestAgent ab in _listAgents)
                {
                    if(!ab.gameObject.activeSelf)
                    {
                        continue;
                    }
                   // if (ab.AgentType==EAgentType.astar)
                    {
                        ab.unit.ChangeAgentType(EAgentType.astar);
                        ab.unit._baseData.defaultTargetPos = targetPos;
                        ab.unit.agent.StartPos = ab.unit.position;
                        ab.unit.agent.TargetPos = targetPos;
                        //if(ab.unit.group==null ||ab.unit.group.leader==ab)
                        //{
                        //    .FindQuickPath(ab.unit.agent,10, ab.unit.agent.StartPos, ab.unit.agent.TargetPos,ab.unit._map,false);
                        //}
                   
                    }
                    //else
                    //{
                    //  (ab.agent).TargetPos=targetPos;
                    //}
                }
            }
           
        }
       public int countLimit = 500;
       const int campCount = 2;
        Color[] _campColor = new Color[campCount]
        {
            Color.white,
            Color.blue,
          //  Color.red
        };
        const int groupCount = 0;
        List<TestAgent> _listAgents = new List<TestAgent>();
        FP[] _atkRanges = new FP[]
        {
            1,
            
            2,
            3,
        };
        Framework.WaitForMS _wait = new Framework.WaitForMS(500);
        IEnumerator SpawnUnit()
        {
            if(randomSeed==0)
            {
                randomSeed = (int)System.DateTime.Now.Ticks;
            }
            UnityEngine.Random.InitState(randomSeed);
            TSRandom random = TSRandom.instance;
            while (true && countLimit>0)
                {               
                    for(int i=0;i<10 && countLimit>0;i++)
                    {
                        TestAgent  tagent = GameObject.Instantiate<TestAgent>(agent);
                        PathFindingAgentBehaviour unit = s_AstarAgent.New();
                        if (unit != null)
                        {
                            tagent._testPathFinding = this;
                            tagent.unit = unit;
                            int camp = countLimit % campCount;
                        //start pos


                            Vector3 vPos = startObj[camp].transform.position
                                    + new Vector3(UnityEngine.Random.Range(0, 2.0f), 0, UnityEngine.Random.Range(0,2.0f));
                            TSVector pos = TSVector.zero;
                            pos.Set(CustomMath.FloatToFP(vPos.x), CustomMath.FloatToFP(vPos.y), CustomMath.FloatToFP(vPos.z));
                            TSVector sPos = pos;
                            tagent.gameObject.transform.position = CustomMath.TsVecToVector3(sPos);
                          //  Debug.Log(pos);
                            //target pos
                            Vector3 tpos = destObj[camp].transform.position;// new Vector3(48.0f, 0.0f, 46.8f);
                            pos.Set(CustomMath.FloatToFP(tpos.x), CustomMath.FloatToFP(tpos.y), CustomMath.FloatToFP(tpos.z));
                
                            //get center
                            int idx = _map.GetGridNodeId(pos);
                            pos = _map.GetWorldPosition(idx);

                            TSVector targetPos = pos;

                            FP attackRange = _atkRanges[countLimit % _atkRanges.Length];
                            FP range = TSMath.Max(attackRange + GridMap.GetNodeSize() * CustomMath.FPHalf, FP.One * 3*GridMap.GetNodeSize());
                            AgentBaseData data = new AgentBaseData();
                            data.id = countLimit;
                            data.mapId = 0;
                            data.playerId = camp;
                            data.eAgentType = EAgentType.none;//data.id%5== 0? EAgentType.ingoreObstacle:
    #if !USING_FLOW_FIELD
                            data.defaultTargetPos = TSVector.zero;//astar
    #else
                            data.defaultTargetPos = targetPos;
    #endif
                            data.loopCallback = tagent.Loop;
                            data.boidsType = (byte)EBoidsActiveType.all;
                            data.maxSpeed = FP.One * maxSpeed;
                            data.position = sPos;
                            data.collidesWithLayer = 1;
                            data.viewRadius = FP.One * 6 * GridMap.GetNodeSize();
                            data.neighbourRadius = range;
                            data.random = random;
                            data.colliderRadius =0.5f;//test
                            data.pfm =_pfm;
                            data.groupId =(byte)( data.eAgentType == EAgentType.ingoreObstacle ? 1 : 0);
                            data.targetFailCallback = tagent.FailFindPathCallback;

                            unit.enabled = false;
                            unit.Init(data);
                        EAgentType agentType = EAgentType.astar;
    #if USING_FLOW_FIELD
                        agentType = EAgentType.flowFiled;
    #endif
                        unit.ChangeAgentType(data.eAgentType==EAgentType.ingoreObstacle? data.eAgentType
                                : agentType);
                            unit.agent.TargetPos = targetPos;
                            // unit.OnEnable();//?????????
                            tagent.attackRange = attackRange;// FP.One * UnityEngine.Random.Range(0.8f, 5);
                            tagent._attackRange = attackRange.AsFloat();
                        // unit.AgentType = EAgentType.flowFiled;    
    #if !USING_FLOW_FIELD
                        unit.ChangeAgentType(EAgentType.astar);//astar
    #endif

                        if (groupCount>0)
                            {
                                AgentGroupManager.instance.UpdateGroup(unit, countLimit % groupCount);
                            }                       
                       
                            tagent.transform.GetChild(0).GetComponent<SpriteRenderer>().color = _campColor[camp];

                            // unit.agent.StartPos = unit.position;
                            // unit._agent.TargetPos = (TSVector)destination;
                            tagent.gameObject.name = "unit" + countLimit;
                            tagent.transform.SetParent(Units);
                           // unit.agent.TargetPos = targetPos;
                      
                           // unit.agent._activeBoids = (byte)EBoidsActiveType.all;

                            _listAgents.Add(tagent);
                            // PathFindingManager.Instance.AddAgent(this);
                            //_pm.FindFastPath(unit._agent, unit._agent.StartPos, unit._agent.TargetPos);//, unit._agent._vecPath
                            // break;
                            if(unit.group!=null && (unit.group.leader as PathFindingAgentBehaviour)==unit && unit.AgentType==EAgentType.astar)
                            {
                                _pm.FindQuickPath(unit.agent,10, unit.agent.StartPos, unit.agent.TargetPos,unit.map,false);
                            }    
                            //if(unit.group!=null)
                            //{
                            //    unit.agent._activeBoids = (byte)EBoidsActiveType.alignment& (byte)EBoidsActiveType.cohesion & (byte)EBoidsActiveType.terrainSeperation
                            //}
                            countLimit--;
                            if(countLimit%5==0)
                            {
                                yield return _wait;
                             }
                        }
                    }
              
                }
                yield return null;
        }
        private void OnDestroy()
        {
            PathFindingAgentBehaviour.ClearAgentPool();
            if(s_AstarAgent!=null)
            {
                s_AstarAgent.Clear();
            }
            _pfm.OnDestroy();
        }
        public TestAgent GetAgent(int agentId)
        {
            int count = _listAgents.Count;
            for (int i = 0; i < count; i++)
            {   
                if(_listAgents[i].unit.Id==agentId)
                {
                    return _listAgents[i];//.enabled;
                }
            }
            return null;
        }
        public void Update()
        {
            int count = _listAgents.Count;
            for(int i=0;i<count;i++)
            {
                _listAgents[i].Loop();
            }
            int desiredDeltaTime = (PathFindingAgentBehaviour.C_DESIRED_DELTATIME);//.AsInt();
            int frameTime = desiredDeltaTime;// Math.Min((int)(UnityEngine.Time.deltaTime * 1000), desiredDeltaTime); // 30;
            _pfm.Update(frameTime);
            Framework.CoroutineScheduler.instance.UpdateAllCoroutines(_pfm.CurTime);
            frameIdx++;
            //if(frameIdx%10==0)
            //{
            //    Debug.LogError(frameIdx);
            //}
        }
    }

