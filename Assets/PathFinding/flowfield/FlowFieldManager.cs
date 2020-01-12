///author:huwei
///date:2018.4.5
using System;
using System.Collections.Generic;

using TrueSync;
namespace PathFinding
{
   public class FlowFieldManager//: Singleton<FlowFieldManager>
    {
        Dictionary<int, FlowField> _dicFlowFields = new Dictionary<int, FlowField>();//key:distination
  
        public FlowField GetFlowField(FlowFieldAgent agent)
        {
            FlowField ff;
            int idx = agent._behaviour.map.GetGridNodeId(agent.TargetPos);
            if (!_dicFlowFields.TryGetValue(idx, out ff))
            {
                
            }
            return ff;
        }
        internal void AddAgent(IAgentBehaviour agent,TSVector dest,GridMap map)
        {
            FlowField ff;
            int idx = map.GetGridNodeId(dest);
            if (!_dicFlowFields.TryGetValue(idx,out ff))
            {
                ff = new FlowField(dest, map);
                _dicFlowFields[idx] =ff;
            }
            ff.AddAgent(agent);        
        }
        internal void RemoveAgent(IAgentBehaviour agent, TSVector dest, GridMap map)
        {
            FlowField ff;
            int idx = map.GetGridNodeId(dest);
            if (_dicFlowFields.TryGetValue(idx, out ff))
            {
                ff.RemoveAgent(agent);
            }        
        }
        internal FlowField UpdateAgent(IAgentBehaviour agent,TSVector preDest, TSVector dest, GridMap map)
        {
            if(preDest!=dest)
            {
                FlowField ff;
                bool hasPre = preDest != TSVector.MinValue;
                int idx = map.GetGridNodeId(preDest);
                if (hasPre && _dicFlowFields.TryGetValue(idx, out ff))
                {
                    ff.RemoveAgent(agent);
                }
                idx = map.GetGridNodeId(dest);
                if (!_dicFlowFields.TryGetValue(idx, out ff))
                {
                    ff = new FlowField(dest,map);
                    _dicFlowFields[idx] = ff;
                }
        
                ff.AddAgent(agent);
                return ff;
            }
            return null;
        }
      
        public void Loop(int frameTime)//test
        {
            int count = _dicFlowFields.Count;
            foreach (KeyValuePair<int,FlowField> kv in _dicFlowFields)
            {                     
                kv.Value.Loop(frameTime);
            }
        }
        public void UpdateFlowFields()
        {
            foreach (KeyValuePair<int, FlowField> kv in _dicFlowFields)
            {
                kv.Value.UpdateFlowField(true);
            }
        }
        public void OnDestroy()
        {
            _dicFlowFields.Clear();
        }
    }
}
