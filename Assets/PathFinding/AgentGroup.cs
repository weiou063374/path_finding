///author:huwei
///date:2018.4.26
using System;
using System.Collections.Generic;
using TrueSync;
namespace PathFinding
{
    public class AgentGroup:IResetable
    {
        public int groupId;
        public IAgentBehaviour leader;
        public List<IAgentBehaviour> _agents = new List<IAgentBehaviour>();
        public void AddAgent(IAgentBehaviour agent)
        {
            if(_agents.Count==0)
            {
                leader = agent;
            }
            _agents.Add(agent);
            agent.group = this;
        }
        public void RemoveAgent(IAgentBehaviour agent)
        {
            if (agent == leader)
            {
                if (_agents.Count > 0)
                {
                    leader = _agents[0];
                }
                else
                {
                    leader = null;
                }
            }
            _agents.Remove(agent);
            agent.group = null;
        }
        public void Reset()
        {
            groupId = -1;
            _agents.Clear();
            leader = null;
        }
        //public void Loop()
        //{

        //}
    }
}
