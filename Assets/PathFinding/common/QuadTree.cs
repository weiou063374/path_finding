///author:huwei
///date:2018.4.5
//using UnityEngine;
//using System.Collections.Generic;
using System;
using System.Collections.Generic;
using TrueSync;
namespace PathFinding
{
    public interface QTData
    {
        QTData next { get; set; }
        TSVector position { get; set; }
        FP speed { get; set; }
        FP neighbourRadius { get; set; }
        FP InsertNeighbour(QTData data, FP rangeSqr);
        void CalculateNeighbours(int camp, Quadtree<QTData> tree);
    }
    public class Quadtree<T> where T: QTData
    {
       static SingleObjectPool<QTNode> _qtNodePool = new SingleObjectPool<QTNode>(50);
        class QTNode:IResetable
        {
            public T nextData;
            public int childNode1;
            public int childNode2;
            public int childNode3;
            public int childNode4;      
            public byte count;
            public FP maxSpeed;//     /** Maxspeed of all data  inside this node */
            public void Reset()
            {
                childNode1=0;
                childNode2=0;
                childNode3=0;
                childNode4 = 0;
                count = 0;
                nextData = default(T);
                maxSpeed = FP.Zero;
            }       
          
            public void Add(T data)
            {
                data.next = nextData;
                nextData = data;
                count++;
            }
   
            public void Distribute(QTNode[] nodes, QTBound r)
            {
                TSVector c = r.center;

                while (nextData != null)
                {
                    T nx =(T) nextData.next;
                    if (nextData.position.x > c.x)
                    {
                        if (nextData.position.z > c.z)
                        {
                            nodes[childNode4].Add(nextData);
                        }
                        else
                        {
                            nodes[childNode3].Add(nextData);
                        }
                    }
                    else
                    {
                        if (nextData.position.z > c.z)
                        {
                            nodes[childNode2].Add(nextData);
                        }
                        else
                        {
                            nodes[childNode1].Add(nextData);
                        }
                    }
                    nextData = (T)nx;
                }
                count = 0;
            }
            
            public FP CalculateMaxSpeed(QTNode[] nodes, int index)
            {
                if (childNode1 == index)
                {
                    // Leaf node
                    for (var data = nextData; data != null; data = (T)data.next)
                    {
                        maxSpeed = TSMath.Max(maxSpeed,data.speed);
                    }
                }
                else
                {
                    maxSpeed = TSMath.Max(nodes[childNode1].CalculateMaxSpeed(nodes, childNode1), nodes[childNode2].CalculateMaxSpeed(nodes, childNode2));
                    maxSpeed = TSMath.Max(maxSpeed, nodes[childNode3].CalculateMaxSpeed(nodes, childNode3));
                    maxSpeed = TSMath.Max(maxSpeed, nodes[childNode4].CalculateMaxSpeed(nodes, childNode4));
                }
                return maxSpeed;
            }
        }

        const int LeafSize = 15;
        FP maxRadius =FP.Zero;
        QTNode[] nodes = new QTNode[42];
        //List<QTNode> nodes = new List<QTNode>();
        int filledNodes = 1;

        QTBound bounds;

 
        public void Clear()
        {
            int count = nodes.Length;
            for (int i = 0; i < filledNodes; i++)
            {
                if(nodes[i]==null)
                {
                    break;
                }
                _qtNodePool.Store(nodes[i]);
            }
            //_qtNodePool.Clear();
            //nodes[0].Reset();// = new QTNode();
            nodes[0] = _qtNodePool.New();
            filledNodes = 1;
            maxRadius = FP.Zero;
        }

        public void SetBounds(QTBound r)
        {
            bounds = r;
        }

        int GetNodeIndex()
        {
            if (filledNodes == nodes.Length)
            {
                var nds = new QTNode[nodes.Length * 2];
                for (int i = 0; i < nodes.Length; i++) nds[i] = nodes[i];
                nodes = nds;
            }
            nodes[filledNodes] = _qtNodePool.New();// new QTNode();
            nodes[filledNodes].childNode1 = filledNodes;
            filledNodes++;
            return filledNodes - 1;
        }
        public void Insert(T T)
        {
            int i = 0;
            QTBound r = bounds;
            TSVector p = T.position;

            T.next = null;

            maxRadius = TSMath.Max(T.neighbourRadius, maxRadius);

            int depth = 0;
            TSVector min = TSVector.zero;
            TSVector max = TSVector.zero;
            while (true)
            {
                depth++;

                if (nodes[i].childNode1 == i)
                {
                    // Leaf node.
                    if (nodes[i].count < LeafSize || depth > 10)
                    {
                        nodes[i].Add(T);
                      //  nodes[i].count++;
                        break;
                    }
                    else
                    {
                        // Split
                        QTNode node = nodes[i];
                        node.childNode1 = GetNodeIndex();
                        node.childNode2 = GetNodeIndex();
                        node.childNode3 = GetNodeIndex();
                        node.childNode4 = GetNodeIndex();
                        nodes[i] = node;

                        nodes[i].Distribute(nodes, r);
                    }
                }
                // Note, no else
                if (nodes[i].childNode1 != i)
                {
                    // Not a leaf node
                    TSVector c = r.center;
                    if (p.x > c.x)
                    {
                        if (p.z > c.z)
                        {
                            i = nodes[i].childNode4;
                            r = QTBound.MinMaxQTBound(c, r.max);
                        }
                        else
                        {
                            i = nodes[i].childNode3;
                            min.Set(c.x,0, r.min.z);
                            max.Set(r.max.x, 0, c.z);
                            r = QTBound.MinMaxQTBound(min, max);
                        }
                    }
                    else
                    {
                        if (p.z > c.z)
                        {
                            i = nodes[i].childNode2;
                            min.Set(r.min.x, 0, c.z);
                            max.Set(c.x, 0, r.max.z);
                            r = QTBound.MinMaxQTBound(min ,max);
                        }
                        else
                        {
                            i = nodes[i].childNode1;
                            r = QTBound.MinMaxQTBound(r.min, c);
                        }
                    }
                }
            }
        }

        public void CalculateSpeeds()
        {
           // nodes[0].CalculateMaxSpeed(nodes, 0);
        }
      
        public void Query(TSVector p, FP speed, FP timeHorizon, FP TRadius, T T)
        {
            QuadtreeQuery tmp = new QuadtreeQuery();
            tmp.p = p;
            tmp.speed = speed;
            tmp.timeHorizon = timeHorizon;
            tmp.maxRadius = FP.One*1000000;//avoid overflow
            tmp.TRadius = TRadius;
            tmp.T = T;
            tmp.nodes = nodes;
            tmp.QueryRec(0, bounds);
        }

        struct QuadtreeQuery
        {
            public TSVector p;
            public FP speed, timeHorizon, TRadius, maxRadius;
            public T T;
            public QTNode[] nodes;

            public void QueryRec(int i, QTBound r)
            {
               
                var radius = TSMath.Min(TSMath.Max((nodes[i].maxSpeed + speed) * timeHorizon, TRadius) , maxRadius);//+ TRadius,warning

                if (nodes[i].childNode1 == i)
                {
                    // Leaf node
                    for (T a = nodes[i].nextData; a != null; a = (T)a.next)
                    {
                        FP v = T.InsertNeighbour(a, radius * radius);
                        //
                        if (v < maxRadius * maxRadius)
                        {
                            maxRadius=TSMath.Sqrt(v);
                        }
                    }
                }
                else
                {
                    TSVector min = TSVector.zero, max = TSVector.zero;
                    // Not a leaf node
                    TSVector c = r.center;
                    if (p.x - radius < c.x)
                    {
                        if (p.z - radius < c.z)
                        {
                            QueryRec(nodes[i].childNode1, QTBound.MinMaxQTBound(r.min, c));
                            radius = TSMath.Min(radius, maxRadius);
                        }
                        if (p.z + radius > c.z)
                        {
                            min.Set(r.min.x, 0, c.z);
                            max.Set(c.x, 0, r.max.z);
                            QueryRec(nodes[i].childNode2, QTBound.MinMaxQTBound(min,max));
                            radius = TSMath.Min(radius, maxRadius);
                        }
                    }

                    if (p.x + radius > c.x)
                    {
                        if (p.z - radius < c.z)
                        {
                            max.Set(r.max.x, 0, c.z);
                            min.Set(c.x, 0, r.min.z);
                            QueryRec(nodes[i].childNode3, QTBound.MinMaxQTBound(min, max));
                            radius = TSMath.Min(radius, maxRadius);
                        }
                        if (p.z + radius > c.z)
                        {
                            QueryRec(nodes[i].childNode4, QTBound.MinMaxQTBound(c, r.max));
                        }
                    }
                }
            }
        }
       
    }
}
