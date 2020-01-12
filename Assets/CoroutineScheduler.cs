using System.Collections;
using UnityEngine;
using System.Collections.Generic;

/// <summary>
/// CoroutineScheduler.cs
/// 
/// Port of the Javascript version from 
/// http://www.unifycommunity.com/wiki/index.php?title=CoroutineScheduler
/// 
/// Linked list node type used by coroutine scheduler to track scheduling of coroutines.
/// 
/// 
/// BMBF Researchproject http://playfm.htw-berlin.de
/// PlayFM - Serious Games für den IT-gestützten Wissenstransfer im Facility Management 
///	Gefördert durch das bmb+f - Programm Forschung an Fachhochschulen profUntFH
///	
///	<author>Frank.Otto@htw-berlin.de</author>
///
/// 
/// A simple coroutine scheduler. Coroutines can yield until the next update
/// "yield;", until a given number of updates "yield anInt", until a given
/// amount of seconds "yield aFloat;", or until another coroutine has finished
/// "yield scheduler.StartCoroutine(Coroutine())".
/// 
/// Multiple scheduler instances are supported and can be very useful. A
/// coroutine running under one scheduler can yield (wait) for a coroutine
/// running under a completely different scheduler instance.
/// 
/// Unity's YieldInstruction classes are not used because I cannot
/// access their internal data needed for scheduling. Semantics are slightly
/// different from Unity's scheduler. For example, in Unity if you start a
/// coroutine it will run up to its first yield immediately, while in this
/// scheduler it will not run until the next time UpdateAllCoroutines is called.
/// This feature allows any code to start coroutines at any time, while
/// making sure the started coroutines only run at specific times.
/// 
/// You should not depend on update order between coroutines running on the same
/// update. For example, StartCoroutine(A), StartCoroutine(B), StartCoroutine(C)
/// where A, B, C => while(true) { print(A|B|C); yield; }, do not expect "ABC" or
/// "CBA" or any other specific ordering.
/// </summary>
namespace Framework
{

    public interface IYieldWrapper
    {
        bool finished { get; }
    }
    public class WaitForMS
    {
        int _ms;//million second

        public WaitForMS(int ms)
        {
            _ms = ms;
        }

        public int ms
        {
            get
            {
                return _ms;
            }

            set
            {
                _ms = value;
            }
        }
    }
    public class WaitForFrames
    {
        int _frames;//million second

        public WaitForFrames(int frames)
        {
            _frames = frames;
        }

        public int frames
        {
            get
            {
                return _frames;
            }

            set
            {
                _frames = value;
            }
        }
    }
    public class CoroutineNode
    {
        int _id;
        public IEnumerator fiber;
        public bool finished = false;
        public int waitForFrame = -1;
        public int waitForTime = -1;
        public CoroutineNode waitForCoroutine;
        public IYieldWrapper waitForUnityObject; //lonewolfwilliams

        public CoroutineNode(IEnumerator _fiber,int id)
        {
            this.fiber = _fiber;
            _id = id;
        }
        public int Id
        {
            get { return _id; }
        }
    }
    public class CoroutineScheduler : Singleton<CoroutineScheduler>
    {
        int _idx=0;
        Dictionary<int, CoroutineNode> _dicCoroutines = new Dictionary<int, CoroutineNode>();
        int currentFrame;
        int currentTime;
        List<int> _tobeRemovedNodes = new List<int>();
        /**
       * Starts a coroutine, the coroutine does not run immediately but on the
       * next call to UpdateAllCoroutines. The execution of a coroutine can
       * be paused at any point using the yield statement. The yield return value
       * specifies when the coroutine is resumed.
       */

        public CoroutineNode StartCoroutine(IEnumerator fiber)
        {
            // if function does not have a yield, fiber will be null and we no-op
            if (fiber == null)
            {
                return null;
            }
            // create coroutine node and run until we reach first yield
            CoroutineNode coroutine = new CoroutineNode(fiber,_idx++);
            AddCoroutine(coroutine);
            return coroutine;
        }
        public void StopCoroutine(CoroutineNode node)
        {
            if (_dicCoroutines.ContainsKey(node.Id))
            {
                _dicCoroutines.Remove(node.Id);
            }
        }
        public void StopCoroutine(int  id)
        {
            if(_dicCoroutines.ContainsKey(id))
            {
                _dicCoroutines.Remove(id);
            }
        }
        /**
       * Stops all coroutines running on this behaviour. Use of this method is
       * discouraged, think of a natural way for your coroutines to finish
       * on their own instead of being forcefully stopped before they finish.
       * If you need finer control over stopping coroutines you can use multiple
       * schedulers.
       */
        public void StopAllCoroutines()
        {
            _dicCoroutines.Clear();
            _idx=0;
            currentFrame = 0;
            currentTime = 0;
        }

        /**
       * Returns true if this scheduler has any coroutines. You can use this to
       * check if all coroutines have finished or been stopped.
       */
        public bool HasCoroutines()
        {
            return _dicCoroutines.Count > 0;
        }

        /**
       * Runs all active coroutines until their next yield. Caller must provide
       * the current frame and time. This allows for schedulers to run under
       * frame and time regimes other than the Unity's main game loop.
       */
        public void UpdateAllCoroutines(int time)
        {
            if(currentTime!=time)
            {
                currentFrame++;
            }      
            currentTime = time;
            foreach(KeyValuePair<int,CoroutineNode> kv in _dicCoroutines)
            {                
                // store listNext before coroutine finishes and is removed from the list
                CoroutineNode coroutine = kv.Value;

                if (coroutine.waitForFrame > 0 && currentFrame >= coroutine.waitForFrame)
                {
                    coroutine.waitForFrame = -1;
                    UpdateCoroutine(coroutine);
                }
                else if (coroutine.waitForTime >0 && time >= coroutine.waitForTime)
                {
                    coroutine.waitForTime = -1;
                    UpdateCoroutine(coroutine);
                }
                else if (coroutine.waitForCoroutine != null && coroutine.waitForCoroutine.finished)
                {
                    coroutine.waitForCoroutine = null;
                    UpdateCoroutine(coroutine);
                }
                else if (coroutine.waitForUnityObject != null && coroutine.waitForUnityObject.finished)//lonewolfwilliams
                {
                    coroutine.waitForUnityObject = null;
                    UpdateCoroutine(coroutine);
                }
                else if (coroutine.waitForFrame == -1 && coroutine.waitForTime == -1
                         && coroutine.waitForCoroutine == null && coroutine.waitForUnityObject == null)
                {
                    // initial update
                    UpdateCoroutine(coroutine);
                }
            }
            foreach(int node in _tobeRemovedNodes)
            {
                _dicCoroutines.Remove(node);
            }
            _tobeRemovedNodes.Clear();
        }

        /**
       * Executes coroutine until next yield. If coroutine has finished, flags
       * it as finished and removes it from scheduler list.
       */
        private void UpdateCoroutine(CoroutineNode coroutine)
        {
            IEnumerator fiber = coroutine.fiber;
            if (coroutine.fiber.MoveNext())
            {
                if(fiber.Current==null)//next frame
                {
                    coroutine.waitForFrame =currentFrame+1;
                    return;
                }
                System.Object yieldCommand = fiber.Current;

                if (yieldCommand.GetType() == typeof(WaitForFrames))
                {
                    WaitForFrames wff = yieldCommand as WaitForFrames;
                    coroutine.waitForFrame = currentFrame + wff.frames;
                }
                else if (yieldCommand.GetType() == typeof(WaitForMS))
                {
                    WaitForMS wfms = yieldCommand as WaitForMS;
                    coroutine.waitForTime = currentTime + wfms.ms;
                }
                else if (yieldCommand.GetType() == typeof(CoroutineNode))
                {
                    coroutine.waitForCoroutine = (CoroutineNode)yieldCommand;
                }
                else
                {
                    throw new System.ArgumentException("CoroutineScheduler: Unexpected coroutine yield type: " + yieldCommand.GetType());
                }
            }
            else
            {
                // coroutine finished
                coroutine.finished = true;
                RemoveCoroutine(coroutine);
            }
        }

        private void AddCoroutine(CoroutineNode coroutine)
        {
            _dicCoroutines[coroutine.Id] = coroutine;   
        }

        private void RemoveCoroutine(CoroutineNode coroutine)
        {
            IEnumerator fiber = coroutine.fiber;
            if (coroutine.fiber.MoveNext())
            {
                if (fiber.Current == null)//next frame
                {
                    coroutine.waitForFrame = currentFrame + 1;
                    return;
                }
                System.Object yieldCommand = fiber.Current;

                if (yieldCommand.GetType() == typeof(WaitForFrames))
                {
                    WaitForFrames wff = yieldCommand as WaitForFrames;
                    coroutine.waitForFrame = currentFrame + wff.frames;
                }
                else if (yieldCommand.GetType() == typeof(WaitForMS))
                {
                    WaitForMS wfms = yieldCommand as WaitForMS;
                    coroutine.waitForTime = currentTime + wfms.ms;
                }
                else if (yieldCommand.GetType() == typeof(CoroutineNode))
                {
                    coroutine.waitForCoroutine = (CoroutineNode)yieldCommand;
                }
                else if (yieldCommand is IYieldWrapper) //lonewolfwilliams
                {
                    coroutine.waitForUnityObject = yieldCommand as IYieldWrapper;
                }
                else
                {
                    throw new System.ArgumentException("CoroutineScheduler: Unexpected coroutine yield type: " + yieldCommand.GetType());
                }
            }
            else
            {
                // coroutine finished
                coroutine.finished = true;
                // StopCoroutine(coroutine);
                _tobeRemovedNodes.Add(coroutine.Id);
            }
        }

    }
}