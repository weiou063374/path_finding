using UnityEngine;
using System.Collections;

namespace Framework
{
    public class Singleton<T> : System.Object
        where T : class,new()
    {
        protected Singleton()
        {

        }
        protected static T _instance = null;
        public static T instance {
            get {
                // if the instance hasn't been assigned then search for it
                if (System.Object.ReferenceEquals(_instance, null)) {
                    _instance = new T();
                }
                return _instance;
            }
        }
     //   protected Singleton(){}
    }
    public class SingletonMono<T> : MonoBehaviour
        where T : MonoBehaviour, new()
    {
      //  protected SingletonMono() { }
		protected virtual void Awake()
        {
            _instance = GetComponent<T>();
        }
        protected virtual void OnDestroy()
        {
            _instance = null;
        }
        protected static T _instance = null;
        public static T instance
        {
            get
            {
                //
                if (System.Object.ReferenceEquals(_instance, null))
                {
                    GameObject singleton = new GameObject();
                    _instance = singleton.AddComponent<T>();
#if UNITY_EDITOR
                    singleton.name = typeof(T).ToString();
#endif
                }
                return _instance;
            }
        }
    }
}

