//#define DYNAMICTREE_DEBUG
using System;
using System.Collections.Generic;
using TrueSync;
namespace PathFinding
{
    ///custom data interface for dynamic tree
    public interface DynamicTreeData
    {
        TSVector position { get; set; }
        AABB aabb { get; set; }
        FP neighbourRadius { get; set; }
        int proxyId { get; set; }
    }
    /// <summary>
    /// node for dynamic tree
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class TreeNode<T>:IResetable
    {
        public const int nullNode = -1;
        /// Enlarged AABB
        public AABB aabb;

        public T userData;//
        public int parentOrNext;
        // int next;
        public int child1;
        public int child2;

        // leaf = 0, free node = -1
        public int height;
        public bool IsLeaf()
        {
            return child1 == nullNode;
        }
        public void Reset()
        {
            child1 = -1;
            child2 = -1;
            userData = default(T);
            aabb.lowerBound = TSVector2.zero;
            aabb.upperBound = TSVector2.zero;
        }
    }
    public delegate bool DynamicTreeQueryCallback(int id);
    public delegate FP DynamicTreeRayCastCallback(ref RayCastInput i, int id);
    public class DynamicTree<T> //where T: DynamicTreeData
    {
        /// This is used to fatten AABBs in the dynamic tree. This allows proxies
        /// to move by a small amount without triggering a tree adjustment.
        public  FP c_aabbExtension = FP.One*15 / 100;
        /// This is used to fatten AABBs in the dynamic tree. This is used to predict
        /// the future position based on the current displacement.
        /// This is a dimensionless multiplier.
        public  FP c_aabbMultiplier = FP.One * 2;
        public int _root;

        public TreeNode<T>[] _nodes = null;
        public int _nodeCount;
        public int _nodeCapacity;

        public int _freeList;

        /// This is used to incrementally traverse the tree for re-balancing.
        public uint _path;
        public int _insertionCount;
        /// ructing the tree initializes the node pool.
        public DynamicTree()
        {
            _root = TreeNode<T>.nullNode;

            _nodeCapacity = 16;
            _nodeCount = 0;
            _nodes = new TreeNode<T>[_nodeCapacity];

          
            // Build a linked list for the free list.
            for (int i = 0; i < _nodeCapacity - 1; ++i)
            {
                TreeNode<T> node = new TreeNode<T>();
                node.parentOrNext = i + 1;
                node.height = -1;
                _nodes[i] = node;
            }
            _nodes[_nodeCapacity - 1] = new TreeNode<T>();
            _nodes[_nodeCapacity - 1].parentOrNext = TreeNode<T>.nullNode;
            _nodes[_nodeCapacity - 1].height = -1;

            _freeList = 0;

            _path = 0;

            _insertionCount = 0;
        }

        /// Destroy the tree, freeing the node pool.
        public void Destroy()
        {
           // _nodeCapacity = 0;
        }

        int AllocateNode()
        {
            // Expand the node pool as needed.
            if (_freeList == TreeNode<T>.nullNode)
            {
                //Assert(_nodeCount == _nodeCapacity);

                // The free list is empty. Rebuild a bigger pool.
                _nodeCapacity *= 2;

                int newSize = _nodeCapacity;
                var tmp = new TreeNode<T>[newSize];
                int len = _nodes.Length;

                if(len> newSize)
                {
                    len = newSize;
                }
                for (int i = 0; i < len; i++)
                {
                    tmp[i] = _nodes[i];
                }
                _nodes = tmp;

                // Build a linked list for the free list. The parent
                // pointer becomes the "next" pointer.
                for (int i = _nodeCount; i < _nodeCapacity - 1; ++i)
                {
                    _nodes[i] = new TreeNode<T>();
                    _nodes[i].parentOrNext = i + 1;
                    _nodes[i].height = -1;
                }
                _nodes[_nodeCapacity - 1] = new TreeNode<T>();
                _nodes[_nodeCapacity - 1].parentOrNext = TreeNode<T>.nullNode;
                _nodes[_nodeCapacity - 1].height = -1;
                _freeList = _nodeCount;
            }

            // Peel a node off the free list.
            int nodeId = _freeList;
            _freeList = _nodes[nodeId].parentOrNext;
            _nodes[nodeId].parentOrNext = TreeNode<T>.nullNode;
            _nodes[nodeId].child1 = TreeNode<T>.nullNode;
            _nodes[nodeId].child2 = TreeNode<T>.nullNode;
            _nodes[nodeId].height = 0;
            _nodes[nodeId].userData = default(T);
            ++_nodeCount;
            return nodeId;
        }
        void Assert(bool bTrue)
        {
#if UNITY_EDITOR && !MULTI_THREAD
            if (PathFinding.PathFindingManager.DEBUG && !bTrue)
            {
                UnityEngine.Debug.LogError("Assert");
            }
#endif
        }
        int Balance(int iA)
        {
            Assert(iA != TreeNode<T>.nullNode);

            TreeNode<T> A = _nodes[iA];
            if (A.IsLeaf() || A.height < 2)
            {
                return iA;
            }

            int iB = A.child1;
            int iC = A.child2;
            Assert(0 <= iB && iB < _nodeCapacity);
            Assert(0 <= iC && iC < _nodeCapacity);

            TreeNode<T> B = _nodes[iB];
            TreeNode<T> C = _nodes[iC];

            int balance = C.height - B.height;

            // Rotate C up
            if (balance > 1)
            {
                int iF = C.child1;
                int iG = C.child2;
                TreeNode<T> F = _nodes[iF];
                TreeNode<T> G = _nodes[iG];
                Assert(0 <= iF && iF < _nodeCapacity);
                Assert(0 <= iG && iG < _nodeCapacity);

                // Swap A and C
                C.child1 = iA;
                C.parentOrNext = A.parentOrNext;
                A.parentOrNext = iC;

                // A's old parent should point to C
                if (C.parentOrNext != TreeNode<T>.nullNode)
                {
                    if (_nodes[C.parentOrNext].child1 == iA)
                    {
                        _nodes[C.parentOrNext].child1 = iC;
                    }
                    else
                    {
                        Assert(_nodes[C.parentOrNext].child2 == iA);
                        _nodes[C.parentOrNext].child2 = iC;
                    }
                }
                else
                {
                    _root = iC;
                }

                // Rotate
                if (F.height > G.height)
                {
                    C.child2 = iF;
                    A.child2 = iG;
                    G.parentOrNext = iA;
                    A.aabb.Combine(ref B.aabb, ref G.aabb);
                    C.aabb.Combine(ref A.aabb, ref F.aabb);

                    A.height = 1 + System.Math.Max(B.height, G.height);
                    C.height = 1 + System.Math.Max(A.height, F.height);
                }
                else
                {
                    C.child2 = iG;
                    A.child2 = iF;
                    F.parentOrNext = iA;
                    A.aabb.Combine(ref B.aabb, ref F.aabb);
                    C.aabb.Combine(ref A.aabb, ref G.aabb);

                    A.height = 1 + System.Math.Max(B.height, F.height);
                    C.height = 1 + System.Math.Max(A.height, G.height);
                }

                return iC;
            }

            // Rotate B up
            if (balance < -1)
            {
                int iD = B.child1;
                int iE = B.child2;
                TreeNode<T> D = _nodes[iD];
                TreeNode<T> E = _nodes[iE];
                Assert(0 <= iD && iD < _nodeCapacity);
                Assert(0 <= iE && iE < _nodeCapacity);

                // Swap A and B
                B.child1 = iA;
                B.parentOrNext = A.parentOrNext;
                A.parentOrNext = iB;

                // A's old parent should point to B
                if (B.parentOrNext != TreeNode<T>.nullNode)
                {
                    if (_nodes[B.parentOrNext].child1 == iA)
                    {
                        _nodes[B.parentOrNext].child1 = iB;
                    }
                    else
                    {
                        Assert(_nodes[B.parentOrNext].child2 == iA);
                        _nodes[B.parentOrNext].child2 = iB;
                    }
                }
                else
                {
                    _root = iB;
                }

                // Rotate
                if (D.height > E.height)
                {
                    B.child2 = iD;
                    A.child1 = iE;
                    E.parentOrNext = iA;
                    A.aabb.Combine(ref C.aabb, ref E.aabb);
                    B.aabb.Combine(ref A.aabb, ref D.aabb);

                    A.height = 1 + System.Math.Max(C.height, E.height);
                    B.height = 1 + System.Math.Max(A.height, D.height);
                }
                else
                {
                    B.child2 = iE;
                    A.child1 = iD;
                    D.parentOrNext = iA;
                    A.aabb.Combine(ref C.aabb, ref D.aabb);
                    B.aabb.Combine(ref A.aabb, ref E.aabb);

                    A.height = 1 + System.Math.Max(C.height, D.height);
                    B.height = 1 + System.Math.Max(A.height, E.height);
                }

                return iB;
            }

            return iA;
        }

        void FreeNode(int nodeId)
        {
            Assert(0 <= nodeId && nodeId < _nodeCapacity);
            Assert(0 < _nodeCount);
            _nodes[nodeId].parentOrNext = _freeList;
            _nodes[nodeId].height = -1;
            _freeList = nodeId;
            --_nodeCount;
        }

        void RemoveLeaf(int leaf)
        {
            if (leaf == _root)
            {
                _root = TreeNode<T>.nullNode;
                return;
            }

            int parent = _nodes[leaf].parentOrNext;
            int grandParent = _nodes[parent].parentOrNext;
            int sibling;
            if (_nodes[parent].child1 == leaf)
            {
                sibling = _nodes[parent].child2;
            }
            else
            {
                sibling = _nodes[parent].child1;
            }

            if (grandParent != TreeNode<T>.nullNode)
            {
                // Destroy parent and connect sibling to grandParent.
                if (_nodes[grandParent].child1 == parent)
                {
                    _nodes[grandParent].child1 = sibling;
                }
                else
                {
                    _nodes[grandParent].child2 = sibling;
                }
                _nodes[sibling].parentOrNext = grandParent;
                FreeNode(parent);

                // Adjust ancestor bounds.
                int index = grandParent;
                while (index != TreeNode<T>.nullNode)
                {
                    index = Balance(index);

                    int child1 = _nodes[index].child1;
                    int child2 = _nodes[index].child2;

                    _nodes[index].aabb.Combine(ref _nodes[child1].aabb, ref _nodes[child2].aabb);
                    _nodes[index].height = 1 + System.Math.Max(_nodes[child1].height, _nodes[child2].height);

                    index = _nodes[index].parentOrNext;
                }
            }
            else
            {
                _root = sibling;
                _nodes[sibling].parentOrNext = TreeNode<T>.nullNode;
                FreeNode(parent);
            }

            //Validate();
        }

        void InsertLeaf(int leaf)
        {

            ++_insertionCount;

            if (_root == TreeNode<T>.nullNode)
            {
                _root = leaf;
                _nodes[_root].parentOrNext = TreeNode<T>.nullNode;
                return;
            }

            // Find the best sibling for this node
            AABB leafAABB = _nodes[leaf].aabb;
            int index = _root;
            while (_nodes[index].IsLeaf() == false)
            {
                int child1 = _nodes[index].child1;
                int child2 = _nodes[index].child2;

                FP area = _nodes[index].aabb.GetPerimeter();

                AABB combinedAABB = new AABB();
                combinedAABB.Combine(ref _nodes[index].aabb, ref leafAABB);
                FP combinedArea = combinedAABB.GetPerimeter();

                // Cost of creating a new parent for this node and the new leaf
                FP cost = FP.One * 2 * combinedArea;

                // Minimum cost of pushing the leaf further down the tree
                FP inheritanceCost = FP.One * 2 * (combinedArea - area);

                // Cost of descending into child1
                FP cost1;
                if (_nodes[child1].IsLeaf())
                {
                    AABB aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child1].aabb);
                    cost1 = aabb.GetPerimeter() + inheritanceCost;
                }
                else
                {
                    AABB aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child1].aabb);
                    FP oldArea = _nodes[child1].aabb.GetPerimeter();
                    FP newArea = aabb.GetPerimeter();
                    cost1 = (newArea - oldArea) + inheritanceCost;
                }

                // Cost of descending into child2
                FP cost2;
                if (_nodes[child2].IsLeaf())
                {
                    AABB aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child2].aabb);
                    cost2 = aabb.GetPerimeter() + inheritanceCost;
                }
                else
                {
                    AABB aabb = new AABB();
                    aabb.Combine(ref leafAABB, ref _nodes[child2].aabb);
                    FP oldArea = _nodes[child2].aabb.GetPerimeter();
                    FP newArea = aabb.GetPerimeter();
                    cost2 = newArea - oldArea + inheritanceCost;
                }

                // Descend according to the minimum cost.
                if (cost < cost1 && cost < cost2)
                {
                    break;
                }

                // Descend
                if (cost1 < cost2)
                {
                    index = child1;
                }
                else
                {
                    index = child2;
                }
            }

            int sibling = index;

            // Create a new parent.
            int oldParent = _nodes[sibling].parentOrNext;
            int newParent = AllocateNode();
            _nodes[newParent].parentOrNext = oldParent;
            _nodes[newParent].userData = default(T);
            _nodes[newParent].aabb.Combine(ref leafAABB, ref _nodes[sibling].aabb);
            _nodes[newParent].height = _nodes[sibling].height + 1;

            if (oldParent != TreeNode<T>.nullNode)
            {
                // The sibling was not the root.
                if (_nodes[oldParent].child1 == sibling)
                {
                    _nodes[oldParent].child1 = newParent;
                }
                else
                {
                    _nodes[oldParent].child2 = newParent;
                }

                _nodes[newParent].child1 = sibling;
                _nodes[newParent].child2 = leaf;
                _nodes[sibling].parentOrNext = newParent;
                _nodes[leaf].parentOrNext = newParent;
            }
            else
            {
                // The sibling was the root.
                _nodes[newParent].child1 = sibling;
                _nodes[newParent].child2 = leaf;
                _nodes[sibling].parentOrNext = newParent;
                _nodes[leaf].parentOrNext = newParent;
                _root = newParent;
            }

            // Walk back up the tree fixing heights and AABBs
            index = _nodes[leaf].parentOrNext;
            while (index != TreeNode<T>.nullNode)
            {
                index = Balance(index);

                int child1 = _nodes[index].child1;
                int child2 = _nodes[index].child2;

                //Assert(child1 != TreeNode<T>.nullNode);
                //Assert(child2 != TreeNode<T>.nullNode);

                _nodes[index].height = 1 + System.Math.Max(_nodes[child1].height, _nodes[child2].height);
                _nodes[index].aabb.Combine(ref _nodes[child1].aabb, ref _nodes[child2].aabb);

                index = _nodes[index].parentOrNext;
            }

            //Validate();
        }
        /// Create a proxy. Provide a tight fitting AABB and a userData pointer.
        public int CreateProxy(ref AABB aabb, T userData)
        {
            int proxyId = AllocateNode();

            // Fatten the aabb.
            TSVector2 r = new TSVector2(c_aabbExtension, c_aabbExtension);
            _nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
            _nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
            _nodes[proxyId].userData = userData;
            _nodes[proxyId].height = 0;

            InsertLeaf(proxyId);

            return proxyId;
        }

        /// Destroy a proxy. This asserts if the id is invalid.
        public void DestroyProxy(int proxyId)
        {
            if(proxyId<0)
            {
                return;
            }
            Assert(0 <= proxyId && proxyId < _nodeCapacity);
            Assert(_nodes[proxyId].IsLeaf());

            RemoveLeaf(proxyId);
            FreeNode(proxyId);
        }

        /// Move a proxy with a swepted AABB. If the proxy has moved outside of its fattened AABB,
        /// then the proxy is removed from the tree and re-inserted. Otherwise
        /// the function returns immediately.
        /// @return true if the proxy was re-inserted.
        public bool MoveProxy(int proxyId, ref AABB aabb, ref TSVector2 displacement)
        {
            Assert(0 <= proxyId && proxyId < _nodeCapacity);

            Assert(_nodes[proxyId].IsLeaf());

            if (_nodes[proxyId].aabb.Contains(ref aabb))
            {
                return false;
            }

            RemoveLeaf(proxyId);

            // Extend AABB.
            AABB b = aabb;
            TSVector2 r = new TSVector2(c_aabbExtension, c_aabbExtension);
            b.lowerBound = b.lowerBound - r;
            b.upperBound = b.upperBound + r;

            // Predict AABB displacement.
            TSVector2 d = c_aabbMultiplier * displacement;

            if (d.x < FP.Zero)
            {
                b.lowerBound.x += d.x;
            }
            else
            {
                b.upperBound.x += d.x;
            }

            if (d.y < FP.Zero)
            {
                b.lowerBound.y += d.y;
            }
            else
            {
                b.upperBound.y += d.y;
            }

            _nodes[proxyId].aabb = b;

            InsertLeaf(proxyId);
            return true;
        }

        /// Get proxy user data.
        /// @return the proxy user data or 0 if the id is invalid.
        public T GetUserData(int proxyId)
        {
            Assert(0 <= proxyId && proxyId < _nodeCapacity);
            return _nodes[proxyId].userData;
        }

        /// Get the fat AABB for a proxy.
        public AABB GetFatAABB(int proxyId)
        {
            Assert(0 <= proxyId && proxyId < _nodeCapacity);
            return _nodes[proxyId].aabb;
        }
        public static bool TestOverlap(ref AABB a, ref AABB b)
        {
            TSVector2 d1, d2;
            d1 = b.lowerBound - a.upperBound;
            d2 = a.lowerBound - b.upperBound;

            if (d1.x > FP.Zero || d1.y > FP.Zero)
                return false;

            if (d2.x > FP.Zero || d2.y > FP.Zero)
                return false;

            return true;
        }


       //  = new Stack<int>();
        /// Query an AABB for overlapping proxies. The callback class
        /// is called for each proxy that overlaps the supplied AABB.
        public bool Query(DynamicTreeQueryCallback callback, ref AABB aabb, Stack<int> _tmpStack, bool bTestHasNode)
        {
            _tmpStack.Clear();
            _tmpStack.Push(_root);

            while (_tmpStack.Count > 0)
            {
                int nodeId = _tmpStack.Pop();
                if (nodeId == TreeNode<T>.nullNode)
                {
                    continue;
                }

                TreeNode<T> node = _nodes[nodeId];

                if (TestOverlap(ref node.aabb, ref aabb))
                {
                    if (node.IsLeaf())
                    {
 						if (bTestHasNode)
                        {
                            return true;
                        }
                        bool proceed = callback(nodeId);
                        if (proceed == false)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        _tmpStack.Push(node.child1);
                        _tmpStack.Push(node.child2);
                    }
                }
            }
			return false;
        }


        public int RayCast(DynamicTreeRayCastCallback callback, ref RayCastInput input, Stack<int> _tmpStack,bool bTestHasNode,int excludeProxyId=-1)
        {
            TSVector2 p1 = input.p1;
            TSVector2 p2 = input.p2;
            TSVector2 r = p2 - p1;
            Assert(r.LengthSquared() >= FP.Zero);
            r.Normalize();

            // v is perpendicular to the segment.
            TSVector2 v = CustomMath.Cross(FP.One, r);
            TSVector2 abs_v = CustomMath.Abs(v);

            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)

            FP maxFraction = input.maxFraction;
            // Build a bounding box for the segment.
            AABB segmentAABB = new AABB();
            {
                TSVector2 t = p1 + maxFraction * (p2 - p1);
                segmentAABB.lowerBound = TSVector2.Min(p1, t);
                segmentAABB.upperBound = TSVector2.Max(p1, t);
            }

            Stack<int> stack = _tmpStack;
            stack.Clear();
            stack.Push(_root);

            while (stack.Count > 0)
            {
                int nodeId = stack.Pop();
                if (nodeId == TreeNode<T>.nullNode)
                {
                    continue;
                }

                TreeNode<T> node = _nodes[nodeId];

                if (TestOverlap(ref node.aabb, ref segmentAABB) == false)
                {
                    continue;
                }

                // Separating axis for segment (Gino, p80).
                // |dot(v, p1 - c)| > dot(|v|, h)
                TSVector2 c = node.aabb.GetCenter();
                TSVector2 h = node.aabb.GetExtents();
                FP separation = TSMath.Abs(TSVector2.Dot(v, p1 - c)) - TSVector2.Dot(abs_v, h);
                if (separation > FP.Zero)
                {
                    continue;
                }
               
                if (node.IsLeaf())
                {
                    if (bTestHasNode && nodeId!=excludeProxyId)
                    {
                        return nodeId;
                    }
                    RayCastInput subInput;
                    subInput.p1 = input.p1;
                    subInput.p2 = input.p2;
                    subInput.maxFraction = maxFraction;
                    FP value = FP.One;
                    if (callback!=null)
                    {
                        value = callback(ref subInput, nodeId);
                    }                

                    if (value == FP.Zero)
                    {
                        // The client has terminated the ray cast.
                        return -1;
                    }

                    if (value > FP.Zero)
                    {
                        // Update segment bounding box.
                        maxFraction = value;
                        TSVector2 t = p1 + maxFraction * (p2 - p1);
                        segmentAABB.lowerBound = TSVector2.Min(p1, t);
                        segmentAABB.upperBound = TSVector2.Max(p1, t);
                    }
                }
                else
                {
                    stack.Push(node.child1);
                    stack.Push(node.child2);
                }
            }
            return -1;
        }

        /// Validate this tree. For testing.
        public void Validate()
        {
#if DYNAMICTREE_DEBUG && UNITY_EDITOR
            ValidateStructure(_root);
            ValidateMetrics(_root);

            int freeCount = 0;
            int freeIndex = _freeList;
            while (freeIndex != TreeNode<T>.nullNode)
            {
                Assert(0 <= freeIndex && freeIndex < _nodeCapacity);
                freeIndex = _nodes[freeIndex].parentOrNext;
                ++freeCount;
            }

            Assert(GetHeight() == ComputeHeight());

            Assert(_nodeCount + freeCount == _nodeCapacity);
#endif
        }

        /// Compute the height of the binary tree in O(N) time. Should not be
        /// called often.
        public int GetHeight()
        {
            if (_root == TreeNode<T>.nullNode)
            {
                return 0;
            }

            return _nodes[_root].height;
        }

        /// Get the maximum balance of an node in the tree. The balance is the difference
        /// in height of the two children of a node.
        public int GetMaxBalance()
        {
            int maxBalance = 0;
            for (int i = 0; i < _nodeCapacity; ++i)
            {
                TreeNode<T> node = _nodes[i];
                if (node.height <= 1)
                {
                    continue;
                }

                Assert(node.IsLeaf() == false);

                int child1 = node.child1;
                int child2 = node.child2;
                int balance = System.Math.Abs(_nodes[child2].height - _nodes[child1].height);
                maxBalance = System.Math.Max(maxBalance, balance);
            }

            return maxBalance;
        }

        /// Get the ratio of the sum of the node areas to the root area.
        public FP GetAreaRatio()
        {
            if (_root == TreeNode<T>.nullNode)
            {
                return FP.Zero;
            }

            TreeNode<T> root = _nodes[_root];
            FP rootArea = root.aabb.GetPerimeter();

            FP totalArea = FP.Zero;
            for (int i = 0; i < _nodeCapacity; ++i)
            {
                TreeNode<T> node = _nodes[i];
                if (node.height < 0)
                {
                    // Free node in pool
                    continue;
                }

                totalArea += node.aabb.GetPerimeter();
            }

            return totalArea / rootArea;
        }

        /// Build an optimal tree. Very expensive. For testing.
        public void RebuildBottomUp()
        {
            int[] nodes = new int[_nodeCount];

            int count = 0;

            // Build array of leaves. Free the rest.
            for (int i = 0; i < _nodeCapacity; ++i)
            {
                if (_nodes[i].height < 0)
                {
                    // free node in pool
                    continue;
                }

                if (_nodes[i].IsLeaf())
                {
                    _nodes[i].parentOrNext = TreeNode<T>.nullNode;
                    nodes[count] = i;
                    ++count;
                }
                else
                {
                    FreeNode(i);
                }
            }

            while (count > 1)
            {
                FP minCost = FP.MaxValue;
                int iMin = -1, jMin = -1;
                for (int i = 0; i < count; ++i)
                {
                    AABB aabbi = _nodes[nodes[i]].aabb;

                    for (int j = i + 1; j < count; ++j)
                    {
                        AABB aabbj = _nodes[nodes[j]].aabb;
                        AABB b = new AABB();
                        b.Combine(ref aabbi, ref aabbj);
                        FP cost = b.GetPerimeter();
                        if (cost < minCost)
                        {
                            iMin = i;
                            jMin = j;
                            minCost = cost;
                        }
                    }
                }

                int index1 = nodes[iMin];
                int index2 = nodes[jMin];
                TreeNode<T> child1 = _nodes[index1];
                TreeNode<T> child2 = _nodes[index2];

                int parentIndex = AllocateNode();
                TreeNode<T> parent = _nodes[parentIndex];
                parent.child1 = index1;
                parent.child2 = index2;
                parent.height = 1 + System.Math.Max(child1.height, child2.height);
                parent.aabb.Combine(ref child1.aabb, ref child2.aabb);
                parent.parentOrNext = TreeNode<T>.nullNode;

                child1.parentOrNext = parentIndex;
                child2.parentOrNext = parentIndex;

                nodes[iMin] = parentIndex;
                --count;
                nodes[jMin] = nodes[count - 1];
            }

            _root = nodes[0];
            Validate();
        }

        public void ShiftOrigin(ref TSVector2 newOrigin)
        {
            // Build array of leaves. Free the rest.
            for (int i = 0; i < _nodeCapacity; ++i)
            {
                _nodes[i].aabb.lowerBound -= newOrigin;
                _nodes[i].aabb.upperBound -= newOrigin;
            }
        }


        int ComputeHeight()
        {
            int height = ComputeHeight(_root);
            return height;
        }
        int ComputeHeight(int nodeId)
        {
            Assert(0 <= nodeId && nodeId < _nodeCapacity);
            TreeNode<T> node = _nodes[nodeId];

            if (node.IsLeaf())
            {
                return 0;
            }

            int height1 = ComputeHeight(node.child1);
            int height2 = ComputeHeight(node.child2);
            return 1 + System.Math.Max(height1, height2);
        }

        void ValidateStructure(int index)
        {
            if (index == TreeNode<T>.nullNode)
            {
                return;
            }

            if (index == _root)
            {
                Assert(_nodes[index].parentOrNext == TreeNode<T>.nullNode);
            }

            TreeNode<T> node = _nodes[index];

            int child1 = node.child1;
            int child2 = node.child2;

            if (node.IsLeaf())
            {
                Assert(child1 == TreeNode<T>.nullNode);
                Assert(child2 == TreeNode<T>.nullNode);
                Assert(node.height == 0);
                return;
            }

            Assert(0 <= child1 && child1 < _nodeCapacity);
            Assert(0 <= child2 && child2 < _nodeCapacity);

            Assert(_nodes[child1].parentOrNext == index);
            Assert(_nodes[child2].parentOrNext == index);

            ValidateStructure(child1);
            ValidateStructure(child2);
        }
        void ValidateMetrics(int index)
        {
            if (index == TreeNode<T>.nullNode)
            {
                return;
            }

            TreeNode<T> node = _nodes[index];

            int child1 = node.child1;
            int child2 = node.child2;

            if (node.IsLeaf())
            {
                Assert(child1 == TreeNode<T>.nullNode);
                Assert(child2 == TreeNode<T>.nullNode);
                Assert(node.height == 0);
                return;
            }

            Assert(0 <= child1 && child1 < _nodeCapacity);
            Assert(0 <= child2 && child2 < _nodeCapacity);

            int height1 = _nodes[child1].height;
            int height2 = _nodes[child2].height;
            int height;
            height = 1 + System.Math.Max(height1, height2);
            Assert(node.height == height);

            AABB aabb = new AABB();
            aabb.Combine(ref _nodes[child1].aabb, ref _nodes[child2].aabb);

            Assert(aabb.lowerBound == node.aabb.lowerBound);
            Assert(aabb.upperBound == node.aabb.upperBound);

            ValidateMetrics(child1);
            ValidateMetrics(child2);
        }
    }
}
