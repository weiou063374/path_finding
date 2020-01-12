//using System.Collections.Generic;
///// <summary>
///// data for UpdateHandle
///// </summary>
//public class UpdateHandleData : IResetable
//{
//    internal bool IsFinished;
//    internal int nextRunTime;
//    public virtual void Reset()
//    {
//        IsFinished = false;
//        nextRunTime = 0;
//    }
//    public virtual void Finish() { IsFinished = true; }
//    //public virtual void Loop(int frameTime, UpdateHandleData handle) { }
//}

//public delegate void RunUpdateHandle<T>(int frameTime, T handle) where T: UpdateHandleData;
///// <summary>
/////simple update handle as a replacer for coroutine
///// </summary>
///// <typeparam name="T"></typeparam>
//public class UpdateListHandle<T> where T : UpdateHandleData
//{
//    List<T> list = null;
//    RunUpdateHandle<T> runHandle = null;
//    public void AddUpdateHandle(T u, RunUpdateHandle<T> rh)
//    {
//        if(list==null)
//        {
//            list = new List<T>();
//        }
//        list.Add(u);
//        runHandle = rh;
//    }
//    public void Clear()
//    {
//        if(list!=null)
//        {
//            //list.Clear();
//            int icount = list.Count;
//            for (int i = icount - 1; i >= 0; i--)
//            {
//                T sh = list[i];
//                //if (sh.bFinish)
//                {
//                    list.RemoveAt(i);
//                    sh.Finish();
//                    sh.Reset();
//                 //   sh.Reset();
//                }
//            }
//        }
//    }
//    public void Loop(int frameTime)
//    {
//        if (list != null && list.Count > 0 && runHandle!=null)
//        {
//            int icount = list.Count;
//            for (int i = icount - 1; i >= 0; i--)
//            {
//                T sh = list[i];
//                if(frameTime<sh.nextRunTime)
//                {
//                    continue;
//                }
//                runHandle(frameTime,sh);
//                if (sh.IsFinished)
//                {
//                    list.RemoveAt(i);
//                    sh.Finish();
//                    sh.Reset();
//                    // sh.Reset();
//                }
//            }
//        }
//    }
  
//}
