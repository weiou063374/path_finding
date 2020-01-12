using UnityEngine;
using System.Collections;

using System.Collections.Generic;


namespace WTWGAME
{
    public enum ESceneObj
    { 
        camp1Home,
        camp2Home,
    }
	public class SceneObjects : MonoBehaviour
    {
        public List<IInt3> ObjPoses = new List<IInt3>();
#if UNITY_EDITOR
        public void HandleObjs()
        {
            int count = transform.childCount;
            ObjPoses.Clear();
            for(int i=0;i<count;i++)
            {
                Transform tr = transform.GetChild(i);
                if (!tr.name.Contains("home"))
                {
                    continue;
                }
                Vector3 pos = tr.position;
                IInt3 iPos= new IInt3(pos);
                ObjPoses.Add(iPos);
            }
        }
        public void HandleObstacles()
        {
        
            Transform obstacleTr = transform.Find("obstacles");
            int count = obstacleTr.childCount;;
            GridGraph gg = GetComponent<GridGraph>();
            gg._startPos = TrueSync.TSVector.zero;
            if (gg!=null && obstacleTr!=null)
            {
                gg.CalculateColliders(obstacleTr);
            }
            else
            {
                Debug.LogError("HandleObstacles  error!");
            }
        }
#endif
    }

}