///author:huwei
///date:2018.4.13
//#define HEAPDEBUG

public interface IBinaryHeapNode
    {
        int HeapIndex { get; set; }
    }
    public delegate bool BinaryHeapMCompare<T>(T a, T b);
    public class BinaryHeapM<T> where T : IBinaryHeapNode
    {
        public struct Item
        {
            public T value;
          //  public int index;
        }

        public BinaryHeapMCompare<T> BinaryHeapMCompareFun = null;
        public int numberOfItems;
        public int growthFactor = 2;

        int D = 2;

        /** Internal backing array*/
        private Item[] binaryHeap;

        public BinaryHeapM(int numberOfElements, BinaryHeapMCompare<T> Compare, int di = 2)
        {
            BinaryHeapMCompareFun = Compare;
            binaryHeap = new Item[numberOfElements];
            numberOfItems = 0;
            D = di;
        }

        public void Clear()
        {
            numberOfItems = 0;
        }
        public void Copy(BinaryHeapM<T> m,int count)
        {
            numberOfItems = System.Math.Min(count, binaryHeap.Length);
            System.Array.Copy(m.binaryHeap, binaryHeap, numberOfItems);
            
        }
       
        public int Add(T node)
        {
            if (node == null) throw new System.ArgumentNullException("node");

            if (numberOfItems == binaryHeap.Length)
            {
                int newSize = System.Math.Max(binaryHeap.Length + 4, (binaryHeap.Length * growthFactor));
                if (newSize > 1 << 18)
                {
                    throw new System.Exception("Binary Heap Size really large (2^18)");
                }

                var tmp = new Item[newSize];

                for (int i = 0; i < binaryHeap.Length; i++)
                {
                    tmp[i] = binaryHeap[i];
                }
            //System.Array.Copy(binaryHeap, tmp, binaryHeap.Length);
#if ASTARDEBUG && UNITY_EDITOR
				UnityEngine.Debug.Log("Resizing binary heap to "+newSize);
#endif
                binaryHeap = tmp;
            }

            binaryHeap[numberOfItems].value = node;
            Item obj = binaryHeap[numberOfItems];
            obj.value.HeapIndex= numberOfItems;

            int bubbleIndex = numberOfItems;


            while (bubbleIndex != 0)
            {
                int parentIndex = (bubbleIndex - 1) / D;

                if (BinaryHeapMCompareFun(binaryHeap[parentIndex].value, obj.value))
                {
                    binaryHeap[bubbleIndex] = binaryHeap[parentIndex];
                    binaryHeap[bubbleIndex].value.HeapIndex=bubbleIndex;

                    binaryHeap[parentIndex] = obj;
                    binaryHeap[parentIndex].value.HeapIndex=parentIndex;
                    bubbleIndex = parentIndex;
                }
                else
                {
                    break;
                }
            }
            numberOfItems++;

#if HEAPDEBUG
            Validate();
#endif
            return obj.value.HeapIndex;
        }

        public int Ajust(int idx = 0)
        {
            int swapItem = idx, parent;
            int newIdx=idx;
            do
            {
                if (D == 0)
                {
                    parent = swapItem;
                    int p2 = parent * D;
                    if (p2 + 1 <= numberOfItems)
                    {
                        // Both children exist
                        if (BinaryHeapMCompareFun(binaryHeap[parent].value, binaryHeap[p2].value))
                        {
                            swapItem = p2;//2 * parent;
                        }
                        if (BinaryHeapMCompareFun(binaryHeap[swapItem].value, binaryHeap[p2 + 1].value))
                        {
                            swapItem = p2 + 1;
                        }
                    }
                    else if ((p2) <= numberOfItems)
                    {
                        // Only one child exists
                        if (BinaryHeapMCompareFun(binaryHeap[parent].value, binaryHeap[p2].value))
                        {
                            swapItem = p2;
                        }
                    }
                }
                else
                {
                    parent = swapItem;
                    int pd = parent * D + 1;

                    if (D >= 1 && pd + 0 < numberOfItems && (BinaryHeapMCompareFun(binaryHeap[swapItem].value, binaryHeap[pd + 0].value)
                    ))
                    {
                        swapItem = pd + 0;
                    }

                    if (D >= 2 && pd + 1 < numberOfItems && (BinaryHeapMCompareFun(binaryHeap[swapItem].value, binaryHeap[pd + 1].value)
                    ))
                    {
                        swapItem = pd + 1;
                    }
                }

                // One if the parent's children are smaller or equal, swap them
                if (parent != swapItem)
                {
                    var tmpIndex = binaryHeap[parent];
                    binaryHeap[parent] = binaryHeap[swapItem];
                    binaryHeap[parent].value.HeapIndex=parent;

                    binaryHeap[swapItem] = tmpIndex;
                    binaryHeap[swapItem].value.HeapIndex=swapItem;
                    newIdx = swapItem;
                }
                else
                {
                    binaryHeap[parent].value.HeapIndex = parent;
                    binaryHeap[swapItem].value.HeapIndex = swapItem;
                    break;
                }
            } while (true);

#if HEAPDEBUG
            Validate();
#endif
            return newIdx;
        }
        public bool NeedValidate = true;

        public void Ajust2(int root,int count)
        {
            int parent = root;
            int child = parent * 2 + 1;
            while (parent < count)
            {
                if (child + 1 < count && BinaryHeapMCompareFun(binaryHeap[child].value, binaryHeap[child + 1].value))
                {
                    ++child;
                }
                if (child < count && BinaryHeapMCompareFun(binaryHeap[parent].value, binaryHeap[child].value))
                {
                    var tmpIndex = binaryHeap[parent];
                    binaryHeap[parent] = binaryHeap[child];
                    binaryHeap[parent].value.HeapIndex=parent;

                    binaryHeap[child] = tmpIndex;
                    binaryHeap[child].value.HeapIndex=child;
                    parent = child;
                    child = parent * 2 + 1;
                }
                else
                {
                    break;
                }
            }
        }
        public void sort2()
        {
            int end = numberOfItems - 1;
            while (end > 0)
            {
                var tmpIndex = binaryHeap[end];
                binaryHeap[end] = binaryHeap[0];
                binaryHeap[end].value.HeapIndex=end;

                binaryHeap[0] = tmpIndex;
                binaryHeap[0].value.HeapIndex=0;
                Ajust2(0, end);
                end--;
            }
        }
        public int UpdateNode(int idx)
        {
            //switch above
            int parent = (idx - 1) / D, swapItem = idx;
            bool bSorted = false;
            int newIdx = idx;
            while (parent != swapItem && parent >= 0)
            {
                if (BinaryHeapMCompareFun(binaryHeap[parent].value, binaryHeap[swapItem].value))
                {
                    var tmpIndex = binaryHeap[parent];
                    binaryHeap[parent] = binaryHeap[swapItem];
                    binaryHeap[parent].value.HeapIndex=parent;

                    binaryHeap[swapItem] = tmpIndex;
                    binaryHeap[swapItem].value.HeapIndex=swapItem;
                    bSorted = true;
                    newIdx = parent;
                    swapItem = parent;
                }
                if (parent == 0)
                {
                    break;
                }
                parent = (parent - 1) / D;
            }
            if (!bSorted)
            {
                newIdx= Ajust(idx);
            }
            return newIdx;
        }
        public T Remove(int idx = 0)
        {
            if (idx >= binaryHeap.Length || numberOfItems == 0)
            {
                return default(T);
            }
            numberOfItems--;
            T returnItem = binaryHeap[idx].value;
            returnItem.HeapIndex = -1;

            binaryHeap[idx] = binaryHeap[numberOfItems];
            binaryHeap[idx].value.HeapIndex=idx;

            UpdateNode(idx);
            return returnItem;
        }

        public void Validate()
        {
            if(!NeedValidate)
            {
                return;
            }
            for (int i = 1; i < numberOfItems; i++)
            {
                int parentIndex = (i - 1) / D;

                if (BinaryHeapMCompareFun(binaryHeap[parentIndex].value, binaryHeap[i].value))
                {
                    throw new System.Exception("Invalid state at " + i + ":" + parentIndex + "," + i);
                }
            }
        }
        public void Rebuild()
        {
#if ASTARDEBUG
			int changes = 0;
#endif
            for (int i = 1; i < numberOfItems; i++)
            {
                int bubbleIndex = i;
                var node = binaryHeap[i].value;
                //uint nodeF = node.F;
                while (bubbleIndex != 0)
                {
                    int parentIndex = (bubbleIndex - 1) / D;

                    if (BinaryHeapMCompareFun(binaryHeap[parentIndex].value, node))
                    {//nodeF < binaryHeap[parentIndex].F  
                        binaryHeap[bubbleIndex] = binaryHeap[parentIndex];
                        binaryHeap[bubbleIndex].value.HeapIndex=bubbleIndex;

                        binaryHeap[parentIndex].value = node;
                        binaryHeap[parentIndex].value.HeapIndex=parentIndex;

                        bubbleIndex = parentIndex;
#if ASTARDEBUG
						changes++;
#endif
                    }
                    else
                    {
                        break;
                    }
                }
            }
#if HEAPDEBUG
            Validate();
#endif

#if ASTARDEBUG && UNITY_EDITOR
			UnityEngine.Debug.Log("+++ Rebuilt Heap - "+changes+" changes +++");
#endif
          
        }
        public T GetItem(int i)
        {            
           if(i>=0 && i<numberOfItems)
           {
             return binaryHeap[i].value;
            }

           return default(T);
        }
    }

//}
