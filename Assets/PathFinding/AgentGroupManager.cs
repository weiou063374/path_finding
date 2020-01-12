///author:huwei
///date:2018.4.26
using System;
using System.Collections.Generic;

using TrueSync;
namespace PathFinding
{
    public class AgentGroupManager : Singleton<AgentGroupManager>
    {
        Dictionary<int, AgentGroup> _group = new Dictionary<int, AgentGroup>();
        static SingleObjectPool<AgentGroup> s_groupPool = new SingleObjectPool<AgentGroup>(5);
        public void UpdateGroup(IAgentBehaviour agent,int groupId)
        {
            if (agent.group != null)
            {
                RemoveAgent(agent);
            }
            AgentGroup group;
            if(!_group.TryGetValue(groupId,out group))
            {
                group = s_groupPool.New();
                group.groupId = groupId;
                _group[groupId] = group;
            }          
            group.AddAgent(agent);
        }
        public void RemoveAgent(IAgentBehaviour agent)
        {
            AgentGroup aGroup = agent.group;
            if(aGroup!=null)
            {
                aGroup.RemoveAgent(agent);
                if (aGroup._agents.Count == 0)//store group
                {
                    s_groupPool.Store(aGroup);
                    _group.Remove(aGroup.groupId);
                }
            }           
        }
        public void Reset()
        {
            _group.Clear();
        }
        public static void OnDestroy()
        {
            s_groupPool.Clear();
        }
    }
}
