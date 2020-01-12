///author:huwei
///date:2018.4.5
using System;
using System.Collections.Generic;

using TrueSync;
namespace PathFinding
{
    public class FlowFieldGroup 
    {
        public List<FlowFieldAgent> _agents = new List<FlowFieldAgent>();
        public void AddAgent(FlowFieldAgent agent)
        {
            _agents.Add(agent);
            agent._group = this;
        }
        public void RemoveAgent(FlowFieldAgent agent)
        {
            _agents.Remove(agent);
            agent._group = null ;
        }
    }
}
