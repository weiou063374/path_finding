#if UNITY_EDITOR
//using UnityEngine;
#endif
using System.Collections.Generic;
using System;
using TrueSync;

namespace PathFinding {
	
	public class SmoothPath
    { 
		public static int subdivisions = 2;
		public static int iterations = 2;
		public static FP strength = CustomMath.FPHalf;
		public static bool uniformLength = true;
		public static int maxSegmentLength = 2;

		public static List<TSVector> SmoothSimple (List<TSVector> path, List<TSVector> subdivided) {
			if (path.Count < 2) return path;

			//List<TSVector> subdivided;

			if (uniformLength) {
				maxSegmentLength = Math.Max(maxSegmentLength, 1);

				FP pathLength = 0;
				for (int i = 0; i < path.Count-1; i++) {
					pathLength += TSVector.Distance(path[i], path[i+1]);
				}
				int estimatedNumberOfSegments =(pathLength / maxSegmentLength).AsInt();
                // Get a list with an initial capacity high enough so that we can add all points
                //subdivided = ListPool<TSVector>.Claim(estimatedNumberOfSegments+2);
                subdivided.Capacity = estimatedNumberOfSegments + 2;

                FP distanceAlong = FP.Zero;

                // Sample points every [maxSegmentLength] world units along the path
                int count = path.Count - 1;
                for (int i = 0; i < count; i++) {
					var start = path[i];
					var end = path[i+1];

					FP length = TSVector.Distance(start, end);

					while (distanceAlong < length) {
						subdivided.Add(TSVector.Lerp(start, end, distanceAlong / length));
						distanceAlong += maxSegmentLength;
					}

					distanceAlong -= length;
				}
			} else {
				subdivisions = Math.Max(subdivisions, 0);

				if (subdivisions > 10) {
#if UNITY_EDITOR && PATHMANAGER_DEBUG
                    
                    UnityEngine.Debug.LogWarning("Very large number of subdivisions. Cowardly refusing to subdivide every segment into more than " + (1 << subdivisions) + " subsegments");
#endif
                    subdivisions = 10;
				}

				int steps = 1 << subdivisions;
				//subdivided = ListPool<TSVector>.Claim();
                subdivided.Capacity = (path.Count - 1) * steps + 1;//

                for (int i = 0; i < path.Count-1; i++)
					for (int j = 0; j < steps; j++)
						subdivided.Add(TSVector.Lerp(path[i], path[i+1], j*FP.One / steps));
			}

			// Make sure we get the exact position of the last point
			// (none of the branches above will add it)
			subdivided.Add(path[path.Count-1]);

			if (strength > 0) {
				for (int it = 0; it < iterations; it++) {
					TSVector prev = subdivided[0];

					for (int i = 1; i < subdivided.Count-1; i++) {
						TSVector tmp = subdivided[i];

						// prev is at this point set to the value that subdivided[i-1] had before this loop started
						// Move the point closer to the average of the adjacent points
						subdivided[i] = TSVector.Lerp(tmp, (prev+subdivided[i+1])/2, strength);
						prev = tmp;
					}
				}
			}
			return subdivided;
		}


        public static void SliceSimple(List<TSVector> path, List<TSVector> subdivided)
        {
            int count = path.Count;
            TSVector diff = TSVector.MinValue;
            for (int i = 0; i < count - 1; i++)
            {
                TSVector curDiff = path[i + 1] - path[i];
                if (curDiff != diff)
                {
                    subdivided.Add(path[i]);
                }
                diff = curDiff;
            }
            if(count-1>=0)
            {
                subdivided.Add(path[count - 1]);
            }         
        }
       
    }
}
