
using System;
using UnityEngine;
using System.Collections.Generic;
using PathFinding;
using System.Collections;
using TrueSync;

namespace WTWGAME
{
    //[Serializable]
    //public struct GridGraphSphereCollider
    //{
    //    int x;
    //    int y;
    //    int z;
    //    int radius;
    //}
    public class GridGraph : MonoBehaviour
    {
        //[SerializeField]
        //public List<GridGraphSphereCollider> _sphereColliders=new List<GridGraphSphereCollider>();
        public List<int> _listColliderNodes = new List<int>();
        public int _mapSizeX = 40;
        public int _mapSizeZ = 90;//64 - 594 * FP.EN1 * CustomMath.FPHalf;
        public TSVector _startPos = new TSVector(64 - 264 * FP.EN1 * CustomMath.FPHalf, 0, 64 - 594 * FP.EN1 * CustomMath.FPHalf);
        // public FP _nodeSize =66*FP.EN2;
        Dictionary<int, int> _hsColliderNodes = new Dictionary<int, int>();
        public GridMap _map = null;
        public void SetUpMap()
        {
            //_nodeSize = PathFinding.GridMap.nodeSize;
            // _mapSizeX=BattleManager.con_terrainW*
            if (_map == null)
            {
                _map = GridMapManager.instance.CreateGridMap(_mapSizeX, _mapSizeZ, _startPos, 0, this.IsTileUnWalkable);
                int count = _listColliderNodes.Count;
                for (int i = 0; i < count; i++)
                {
                    int gidx = _listColliderNodes[i];
                    int proxyId = _map.AddObstacle(gidx);
                    _hsColliderNodes.Add(gidx, proxyId);
                }
            }
        }
        //warning 
        public void AddObstacle(TSVector pos, FP size)
        {
            FP nodeSize = GridMap.GetNodeSize();
            for (FP x = -size * CustomMath.FPHalf; x < size * CustomMath.FPHalf + nodeSize; x += nodeSize)
            {
                if (x > size * CustomMath.FPHalf)
                {
                    x = size * CustomMath.FPHalf;
                }
                for (FP z = -size * CustomMath.FPHalf; z < size * CustomMath.FPHalf + nodeSize; z += nodeSize)
                {
                    if (z > size * CustomMath.FPHalf)
                    {
                        z = size * CustomMath.FPHalf;
                    }
                    TSVector tPos = TSVector.zero;
                    tPos.x = pos.x + x;
                    tPos.z = pos.z + z;

                    int idx = _map.GetGridNodeId(tPos);
                    if (_hsColliderNodes.ContainsKey(idx))
                    {
                        break;
                    }
                    int pid = _map.AddObstacle(idx);
                    _hsColliderNodes.Add(idx, pid);
                }
            }

        }
        //warning obstacle can't be overlap
        public void RemoveObstacle(TSVector pos, FP size)
        {
            for (FP x = -size * CustomMath.FPHalf; x < size * CustomMath.FPHalf + GridMap.GetNodeSize(); x += GridMap.GetNodeSize())
            {
                if (x > size * CustomMath.FPHalf)
                {
                    x = size * CustomMath.FPHalf;
                }
                for (FP z = -size * CustomMath.FPHalf; z < size * CustomMath.FPHalf + GridMap.GetNodeSize(); z += GridMap.GetNodeSize())
                {
                    if (z > size * CustomMath.FPHalf)
                    {
                        z = size * CustomMath.FPHalf;
                    }
                    TSVector tPos = TSVector.zero;
                    tPos.x = pos.x + x;
                    tPos.z = pos.z + z;

                    int idx = _map.GetGridNodeId(tPos);
                    int pid = 0;
                    if (_hsColliderNodes.TryGetValue(idx, out pid))
                    {
                        _hsColliderNodes.Remove(idx);
                        _map.RemoveObstacle(pid);
                    }
                    else
                    {
                        Debug.LogError("obstacle may be overlap");
                    }
                }
            }

        }
        public bool IsTileUnWalkable(int gidx)
        {
            return _hsColliderNodes.ContainsKey(gidx);
        }
        public bool IsTileUnWalkable(IInt2 iPos)
        {
            int gidx = iPos.y * _mapSizeX + iPos.x;
            return _hsColliderNodes.ContainsKey(gidx);
        }
        private void OnEnable()
        {
#if UNITY_EDITOR
            if (drawObstalceCollider && _map != null)
            {
                int count = _listColliderNodes.Count;
                for (int i = 0; i < count; i++)
                {
                    TSVector lPos = _map._startPos + _map.GetWorldPosition(_listColliderNodes[i]);
                    Vector3 pos = CustomMath.TsVecToVector3(lPos);
                    Debug.DrawLine(pos, pos + Vector3.up * 3, Color.red, 60);
                }
            }
#endif
        }
#if UNITY_EDITOR
        public bool drawObstalceCollider = false;

        public TSVector GetGridFloatCoord(TSVector position)
        {
            TSVector vec = TSVector.zero;
            vec.x = (position.x - _startPos.x) / PathFinding.GridMap.GetNodeSize();
            vec.z = (position.z - _startPos.z) / PathFinding.GridMap.GetNodeSize();
            return vec;
        }
        public bool GetGridCoord(TSVector position, ref IInt2 coord)
        {
            TSVector vec = GetGridFloatCoord(position);
            int ix = TSMath.Floor(vec.x).AsInt();
            int iz = TSMath.Floor(vec.z).AsInt();
            int x = CustomMath.Clamp(ix, 0, _mapSizeX - 1);
            int z = CustomMath.Clamp(iz, 0, _mapSizeZ - 1);
            //TSVector2 coord;
            coord.x = x;
            coord.y = z;
            return x == ix && z == iz;
        }
        public void CalculateColliders(Transform tr)
        {
            PathFinding.GridMap.SetNodeSize(1, 1);
            //_nodeSize = PathFinding.GridMap.nodeSize;
            float size = PathFinding.GridMap.GetNodeSize().AsFloat();
            int childCount = tr.childCount;
            HashSet<int> dicNodes = new HashSet<int>();
            _listColliderNodes.Clear();
            for (int i = 0; i < childCount; i++)
            {
                if(!tr.GetChild(i).gameObject.activeSelf)
                {
                    continue;
                }
                SphereCollider sc = tr.GetChild(i).GetComponent<SphereCollider>();
                BoxCollider bc = tr.GetChild(i).GetComponent<BoxCollider>();
                Vector3 minPos = Vector3.zero;
                Vector3 maxPos = Vector3.zero;
                IInt2 iPos = IInt2.zero;
                TSVector vec = TSVector.zero;
                int gidx = 0;
                if (sc)
                {
                    Vector3 center = sc.center;//sc.transform.position+
                    minPos =center -  (sc.radius)*sc.transform.localScale;
                    maxPos = center +  sc.radius*sc.transform.localScale;
                    for (float x = minPos.x; x < maxPos.x + size - CustomMath.EPSILON; x = x + size)
                    {
                        if (x > maxPos.x)
                        {
                            x = maxPos.x;
                        }
                        for (float z = minPos.z; z < maxPos.z + size - CustomMath.EPSILON; z = z + size)
                        {
                            if (z > maxPos.z)
                            {
                                z = maxPos.z;
                            }
                            iPos = IInt2.zero;
                            vec = new TSVector(x, 0, z);
                            GetGridCoord(vec, ref iPos);
                            if ((CustomMath.TsVecToVector3(vec) - center).magnitude <= sc.radius)
                            {
                                gidx = iPos.y * _mapSizeX + iPos.x;
                                if (!dicNodes.Contains(gidx))
                                {
                                    _listColliderNodes.Add(gidx);
                                    dicNodes.Add(gidx);
                                }
                            }
                        }
                    }
                    vec = new TSVector(maxPos.x, 0, maxPos.z);
                    GetGridCoord(vec, ref iPos);
                    if ((CustomMath.TsVecToVector3(vec) - center).magnitude <= sc.radius)
                    {
                        gidx = iPos.y * _mapSizeX + iPos.x;
                        if (!dicNodes.Contains(gidx))
                        {
                            _listColliderNodes.Add(gidx);
                            dicNodes.Add(gidx);
                        }
                    }

                }
                else if (bc != null)
                {
                    iPos = IInt2.zero;
                    vec = TSVector.zero;
                   
                    minPos = bc.bounds.min;// * bc.transform.localScale.x;//bc.transform.position +
                    maxPos = bc.bounds.max;// * bc.transform.localScale.z;// bc.transform.position +
                    for (float x = minPos.x; x < maxPos.x + size - CustomMath.EPSILON; x = x + size)
                    {
                        if (x > maxPos.x)
                        {
                            x = maxPos.x;
                        }
                        for (float z = minPos.z; z < maxPos.z + size - CustomMath.EPSILON; z = z + size)
                        {
                            if (z > maxPos.z)
                            {
                                z = maxPos.z;
                            }
                            iPos = IInt2.zero;
                            vec = new TSVector(x, 0, z);
                            GetGridCoord(vec, ref iPos);
                            gidx = iPos.y * _mapSizeX + iPos.x;
                            if (!dicNodes.Contains(gidx))
                            {
                                dicNodes.Add(gidx);
                                _listColliderNodes.Add(gidx);
                            }
                        }
                    }

                }

            }
        }
#endif
    }
}



