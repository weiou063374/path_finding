using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using WTWGAME;
public class GenRandomObstacles
{
    [MenuItem("Tool/GenRandomObstacles")]
    public static void GenObstacles()
    {
       
        var cube = GameObject.Find("Cube");
        var obstacles = GameObject.Find("scene/obstacles");
        int count = obstacles.transform.childCount;
        int size = 50;
        int newCount = Random.Range(20, 30);
        int maxCount =Mathf.Max(count, newCount);
        for (int i=0;i<maxCount;i++)
        {
            Transform tr = null;
            if(i<count)
            {
                tr = obstacles.transform.GetChild(i);
                tr.gameObject.SetActive(true);
                if(i>= newCount)
                {
                    tr.gameObject.SetActive(false);
                }
            }
            else if(i< maxCount)
            {
                var go = GameObject.Instantiate(cube);
                tr = go.transform;
                go.layer = LayerMask.NameToLayer("obstacles");
                tr.parent = obstacles.transform;
            }
           
            if(tr!=null)
            {
                tr.localScale = new Vector3(Random.Range(0.1f,3),1,Random.Range(0.1f,3));
                tr.localPosition = new Vector3(Random.Range(tr.localScale.x*0.5f+3,size), 0, Random.Range(tr.localScale.z * 0.5f +3, size));
            }
        }
    }
    [MenuItem("Tool/SaveSceneObjPosesAsInt and handle obstacles")]
    public static void SaveSceneObjPosesAsInt()
    {
        GameObject go = GameObject.Find("scene");
        if (go == null)
        {
            Debug.LogError("SaveSceneObjPosesAsInt scene null");
        }
        SceneObjects so = go.GetComponent<SceneObjects>();
        so.HandleObjs();
        so.HandleObstacles();
        UnityEditor.SceneManagement.EditorSceneManager.SaveOpenScenes();
    }
}
