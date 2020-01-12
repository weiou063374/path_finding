///author:huwei
///date:2018.4.17
using System;
using System.Collections.Generic;
using TrueSync;
using AStarMachine;
namespace PathFinding
{
   public class GridMapManager : Singleton<GridMapManager>
    {
       // static SingleObjectPool<GridMap> _mapPool = new SingleObjectPool<GridMap>(10);

        Dictionary<int, GridMap> _dicMap = new Dictionary<int, GridMap>();
        public GridMap CreateGridMap(int w,int h,TSVector startPos,int id, IsTileUnWalkable isTileWalkable)
        {
            GridMap map;
            if(!_dicMap.TryGetValue(id, out map))
            {
                map = new GridMap();//_mapPool.New();
                map.Ini(w, h, startPos, id, isTileWalkable);//todo
                _dicMap[id] = map;
            }           
            return map;
        }
        public GridMap GetMap(int id)
        {
            GridMap map;
            _dicMap.TryGetValue(id, out map);
            return map;
        }
        public void RemoveMap(int id)
        {
            if(_dicMap.ContainsKey(id))
            {
                _dicMap.Remove(id);
            }
        }
    }
}
