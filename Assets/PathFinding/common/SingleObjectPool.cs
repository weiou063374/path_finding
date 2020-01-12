///author:huwei
///date:2018.4.5
//#define POOLTEST
using System;
using System.Collections.Generic;
using System.Collections;

//public interface ObjectReset
//{
//    void Reset();
//}
public interface IResetable
{
  //   bool IsPooleed { get; set; }
     void Reset();
}
public class SingleObjectPool<T> where T : class, IResetable,new()
{
    private Stack<T> m_objectStack;
    private readonly Object lock_Obj=new Object();
    private Action<T> m_resetAction;
    private Action<T> m_onetimeInitAction;
#if POOLTEST && UNITY_EDITOR
    Hashtable _hs=new Hashtable();
#endif
    public SingleObjectPool(int initialBufferSize, Action<T>
        ResetAction = null, Action<T> OnetimeInitAction = null)
    {
        m_objectStack = new Stack<T>(initialBufferSize);
        for(int i=0;i<initialBufferSize;i++)
        {
            T t = new T();
            t.Reset();
#if POOLTEST 
            _hs.Add(t,true);
#endif
            // t.IsPooleed = true;
            m_objectStack.Push(t);
        }
        m_resetAction = ResetAction;
        m_onetimeInitAction = OnetimeInitAction;
    }
    public void Clear()
    {
        lock(lock_Obj)
        {
            if (m_objectStack != null)
            {
                m_objectStack.Clear();
            }
        }
       
    }
  
    public T New()
    {
      
        lock (lock_Obj)
        {
            int count = m_objectStack.Count;
            if (count > 0)
            {
                T t = m_objectStack.Pop();
#if POOLTEST
            _hs.Remove(t);
#endif
                // t.Reset();
                // t.IsPooleed = false;
                if (m_resetAction != null)
                    m_resetAction(t);

                return t;
            }
        }
        T t1 = new T();
        t1.Reset();
        //t.IsPooleed = true;
        //   m_objectStack.Push(t);
        if (m_onetimeInitAction != null)
            m_onetimeInitAction(t1);

        return t1;
    }

    public void Store(T obj)//warning，will cause issue if store the same obj multi times
    {
        if(obj!=null)//&& !obj.IsPooleed
        {
            obj.Reset();
            //obj.IsPooleed = true;
            lock(lock_Obj)
            {
                m_objectStack.Push(obj);
#if POOLTEST
            if(_hs.ContainsKey(obj))
            {
              //UnityEngine.Debug.LogError("pool store duplicate!");
                return;
            }
            _hs.Add(obj, true);
#endif
            }

        }
    }
}