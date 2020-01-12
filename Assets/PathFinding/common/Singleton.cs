///author:huwei
///date:2018.4.5
#if UNITY_EDITOR
using UnityEngine;
#endif
using System.Collections;

namespace PathFinding
{
	public class Singleton<T>: System.Object
        where T:class,new()
	{
		protected static T _instance=null;
		public static T instance
        {
			get{
				// 
				if(System.Object.ReferenceEquals(_instance,null) ){
					_instance = new T();
				}
				return _instance; 
			}
		}
    }
#if UNITY_EDITOR
    public class SingletonMono<T> : MonoBehaviour
        where T : MonoBehaviour, new()
    {
        protected static T _instance = null;
        public static T instance
        {
            get
            {
                if (applicationIsQuitting)
                {
                    Debug.LogWarning("[Singleton] Instance '" + typeof(T) +
                        "' already destroyed on application quit.returning null.");
                    return null;
                }
                // 
                if (System.Object.ReferenceEquals(_instance, null))
                {
                    GameObject singleton = new GameObject();
                    _instance = singleton.AddComponent<T>();
                }
                return _instance;
            }
        }

        public static bool applicationIsQuitting = false;
        public void OnDestroy()
        {
            applicationIsQuitting = true;
        }
    }
#endif
}

