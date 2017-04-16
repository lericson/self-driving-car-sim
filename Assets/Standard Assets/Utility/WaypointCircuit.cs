using System;
using System.Collections;
using UnityEngine;
using System.Collections.Generic;
using UnityStandardAssets  ;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace UnityStandardAssets.Utility
{
    public class WaypointCircuit : MonoBehaviour
    {
        public WaypointList waypointList = new WaypointList();
        [SerializeField] private bool smoothRoute = true;
        private int numPoints;
        public Vector3[] points;
        public float[] distances;
        private System.Random rnd = new System.Random(DateTime.Now.Millisecond + DateTime.Now.Second);

        public float editorVisualisationSubsteps = 100;
        public float Length { get; private set; }

        public Transform[] Waypoints
        {
            get { return waypointList.items; }
        }

        //this being here will save GC allocs
        private int p0n;
        private int p1n;
        private int p2n;
        private int p3n;

        private float i;
        private Vector3 P0;
        private Vector3 P1;
        private Vector3 P2;
        private Vector3 P3;

        // Use this for initialization
        private void Awake()
        {
            if (Waypoints.Length > 1)
            {
                CachePositionsAndDistances();
            }
            numPoints = Waypoints.Length;
        }


        public RoutePoint GetRoutePoint(float dist)
        {
            // position and direction
            Vector3 p1 = GetRoutePosition(dist);
            Vector3 p2 = GetRoutePosition(dist + 0.1f);
            Vector3 delta = p2 - p1;
            return new RoutePoint(p1, delta.normalized);
        }


        public Vector3 GetRoutePosition(float dist)
        {
            int point = 0;

            if (Length == 0)
            {
                Length = distances[distances.Length - 1];
            }
            dist = Mathf.Repeat(dist, Length);

            while (distances[point] < dist)
            {
                ++point;
            }


            // get nearest two points, ensuring points wrap-around start & end of circuit
            p1n = ((point - 1) + numPoints)%numPoints;
            p2n = point;

            // found point numbers, now find interpolation value between the two middle points
            i = Mathf.InverseLerp(distances[p1n], distances[p2n], dist);

            if (smoothRoute)
            {
                // smooth catmull-rom calculation between the two relevant points


                // get indices for the surrounding 2 points, because
                // four points are required by the catmull-rom function
                p0n = ((point - 2) + numPoints)%numPoints;
                p3n = (point + 1)%numPoints;

                // 2nd point may have been the 'last' point - a dupe of the first,
                // (to give a value of max track distance instead of zero)
                // but now it must be wrapped back to zero if that was the case.
                p2n = p2n%numPoints;

                P0 = points[p0n];
                P1 = points[p1n];
                P2 = points[p2n];
                P3 = points[p3n];

                return CatmullRom(P0, P1, P2, P3, i);
            }
            else
            {
                // simple linear lerp between the two points:

                p1n = ((point - 1) + numPoints)%numPoints;
                p2n = point;

                return Vector3.Lerp(points[p1n], points[p2n], i);
            }
        }
        private Vector3 CatmullRom(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float i)
        {
            // comments are no use here... it's the catmull-rom equation.
            // Un-magic this, lord vector!
            return 0.5f *
                   ((2 * p1) + (-p0 + p2) * i + (2 * p0 - 5 * p1 + 4 * p2 - p3) * i * i +
                    (-p0 + 3 * p1 - 3 * p2 + p3) * i * i * i);
        }

        public IEnumerable<CatmulRomSpline> GetCurrentSplines(Vector3 pos)
        {
            int closestI = GetClosestWaypointIndex(pos);
            Vector3 closestWaypoint = points[closestI];

            Vector3 nextWaypoint = points[(closestI + 1 + numPoints) % numPoints];
            Vector3 previousWaypoint = points[(closestI - 1 + numPoints) % numPoints];
            
            //TODO: Use planes intead of point projections
            Vector3 closestOnLineAhead = Math3d.ProjectPointOnLineSegment(nextWaypoint, closestWaypoint, pos);
            Vector3 closestOnLineBehind = Math3d.ProjectPointOnLineSegment(closestWaypoint, previousWaypoint, pos);

            int p0Idx, p1Idx, p2Idx, p3Idx;
            int p0Idx_, p1Idx_, p2Idx_, p3Idx_;

            bool closerToLineBehind = Vector3.Distance(closestOnLineAhead, pos) < Vector3.Distance(closestOnLineBehind, pos);

            if (closerToLineBehind)
            {
                p0Idx_ = (closestI - 1 + numPoints) % numPoints;
                p0Idx = (p0Idx_ - 1 + numPoints) % numPoints;
            }
            else
            {
                p0Idx = (closestI - 2 + numPoints) % numPoints;
                p0Idx_ = (p0Idx + 1 + numPoints) % numPoints;
            }
            p1Idx = (p0Idx + 1 + numPoints) % numPoints;
            p2Idx = (p0Idx + 2 + numPoints) % numPoints;
            p3Idx = (p0Idx + 3 + numPoints) % numPoints;

            //Debug.Log(string.Format("p0Idx = {0}, p1Idx = {1}, p2Idx = {2}, p3Idx = {3}, numPoints = {4}, closestI = {5}",
            //    p0Idx,p1Idx,p2Idx,p3Idx,numPoints,closestI));

            p1Idx_ = (p0Idx_ + 1 + numPoints) % numPoints;
            p2Idx_ = (p0Idx_ + 2 + numPoints) % numPoints;
            p3Idx_ = (p0Idx_ + 3 + numPoints) % numPoints;

            return new List<CatmulRomSpline> {
                new CatmulRomSpline(points[p0Idx], points[p1Idx], points[p2Idx], points[p3Idx]),
                new CatmulRomSpline(points[p0Idx_], points[p1Idx_], points[p2Idx_], points[p3Idx_])
            };
        }

        public int GetClosestWaypointIndex(Vector3 pos)
        {
            int closestI = -1;
            float closest = float.MaxValue;
            for (int i = 0; i < points.Length; i++)
            {
                float dist = Vector3.Distance(pos, points[i]);
                if (dist < closest)
                {
                    closest = dist;
                    closestI = i;
                }
            }
            return closestI;
        }

        public void putAtRandomPoint(Transform pos)
        {
            int p0Idx = rnd.Next(numPoints);
            Debug.Log(string.Format("p0Idx = {0}", p0Idx));
            Debug.Log(string.Format("numPoints = {0}", numPoints));
            int p1Idx = (p0Idx + 1 + numPoints) % numPoints;
            int p2Idx = (p0Idx + 2 + numPoints) % numPoints;
            int p3Idx = (p0Idx + 3 + numPoints) % numPoints;
            var spline = new CatmulRomSpline(points[p0Idx], points[p1Idx], points[p2Idx],points[p3Idx]);
            float t = (float)rnd.NextDouble();
            Debug.Log(string.Format("t = {0}",t));
            Vector3 x = spline.getPointAtT(t);
            Vector3 x_ = spline.getPointAtT(t + 1e-2f);
            Vector3 newHeadding= Vector3.Normalize(x_ - x);
            Vector3 initialHeadding = pos.rotation * Vector3.forward;
            Debug.Log(string.Format("newHeadding = {0}",newHeadding));
            //pos.rotation.SetFromToRotation(newHeadding, initialHeadding);
            pos.position = x;
            pos.rotation = Quaternion.LookRotation(newHeadding);
        }




        private void CachePositionsAndDistances()
        {
            // transfer the position of each point and distances between points to arrays for
            // speed of lookup at runtime
            points = new Vector3[Waypoints.Length + 1];
            distances = new float[Waypoints.Length + 1];

            float accumulateDistance = 0;
            for (int i = 0; i < points.Length; ++i)
            {
                var t1 = Waypoints[(i)%Waypoints.Length];
                var t2 = Waypoints[(i + 1)%Waypoints.Length];
                if (t1 != null && t2 != null)
                {
                    Vector3 p1 = t1.position;
                    Vector3 p2 = t2.position;
                    points[i] = Waypoints[i%Waypoints.Length].position;
                    distances[i] = accumulateDistance;
                    accumulateDistance += (p1 - p2).magnitude;
                }
            }
        }


        private void OnDrawGizmos()
        {
            DrawGizmos(false);
        }


        private void OnDrawGizmosSelected()
        {
            DrawGizmos(true);
        }


        private void DrawGizmos(bool selected)
        {
            waypointList.circuit = this;
            if (Waypoints.Length > 1)
            {
                numPoints = Waypoints.Length;

                CachePositionsAndDistances();
                Length = distances[distances.Length - 1];

                Gizmos.color = selected ? Color.yellow : new Color(1, 1, 0, 0.5f);
                Vector3 prev = Waypoints[0].position;
                if (smoothRoute)
                {
                    for (float dist = 0; dist < Length; dist += Length/editorVisualisationSubsteps)
                    {
                        Vector3 next = GetRoutePosition(dist + 1);
                        Gizmos.DrawLine(prev, next);
                        prev = next;
                    }
                    Gizmos.DrawLine(prev, Waypoints[0].position);
                }
                else
                {
                    for (int n = 0; n < Waypoints.Length; ++n)
                    {
                        Vector3 next = Waypoints[(n + 1)%Waypoints.Length].position;
                        Gizmos.DrawLine(prev, next);
                        prev = next;
                    }
                }
            }
        }


        [Serializable]
        public class WaypointList
        {
            public WaypointCircuit circuit;
            public Transform[] items = new Transform[0];
        }

        public struct RoutePoint
        {
            public Vector3 position;
            public Vector3 direction;


            public RoutePoint(Vector3 position, Vector3 direction)
            {
                this.position = position;
                this.direction = direction;
            }
        }
    }

    public class CatmulRomSpline
    {
        public Vector3 p0 { get; private set; }
        public Vector3 p1 { get; private set; }
        public Vector3 p2 { get; private set; }
        public Vector3 p3 { get; private set; }

        private GameObject p0Marker;
        private GameObject p1Marker;
        private GameObject p2Marker;
        private GameObject p3Marker;

        private bool m_visualised = false;
        public bool visualised {
            get
            {
                return m_visualised;
            }
            set
            {
                if(value == m_visualised)
                {
                    return;
                }
                m_visualised = value;
                if (m_visualised)
                {

                    p0Marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    p0Marker.transform.localScale = new Vector3(1f, 1f, 1f);
                    p0Marker.transform.position = p0;
                    p0Marker.GetComponent<Collider>().enabled = false;

                    p1Marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    p1Marker.transform.localScale = new Vector3(1f, 1f, 1f);
                    p1Marker.transform.position = p1;
                    p1Marker.GetComponent<Collider>().enabled = false;

                    p2Marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    p2Marker.transform.localScale = new Vector3(1f, 1f, 1f);
                    p2Marker.transform.position = p2;
                    p2Marker.GetComponent<Collider>().enabled = false;

                    p3Marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    p3Marker.transform.localScale = new Vector3(1f, 1f, 1f);
                    p3Marker.transform.position = p3;
                    p3Marker.GetComponent<Collider>().enabled = false;
                }
                else
                {
                    GameObject.Destroy(p0Marker);
                    GameObject.Destroy(p1Marker);
                    GameObject.Destroy(p2Marker);
                    GameObject.Destroy(p3Marker);
                }

            }
        }

        public CatmulRomSpline() : this(Vector3.zero, Vector3.zero, Vector3.zero, Vector3.zero)
        {}

        public CatmulRomSpline(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            this.p0 = p0;
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
        }

        public Vector3 GetClosestPointOnSpline(Vector3 pos)
        {
            Vector3 _;
            return GetClosestPointOnSpline(pos, out _);
        }

        public static Vector3 GetClosestPointOnSeveralSplines(IEnumerable<CatmulRomSpline> splines,Vector3 pos, out Vector3 roadHeadding)
        {
            float closestDist = float.MaxValue;
            Vector3 closestPoint = Vector3.zero;
            roadHeadding = Vector3.zero;
            foreach (CatmulRomSpline spline in splines)
            {
                float dist;
                Vector3 roadHeaddingCand;
                Vector3 closestPointCand = spline.GetClosestPointOnSpline(pos, out roadHeaddingCand, out dist);
                if(dist < closestDist)
                {
                    closestDist = dist;
                    roadHeadding = roadHeaddingCand;
                    closestPoint = closestPointCand;
                }
            }
            return closestPoint;
        }

        public Vector3 GetClosestPointOnSpline(Vector3 pos, out Vector3 roadHeadding)
        {
            float _;
            return GetClosestPointOnSpline(pos, out roadHeadding,out _);
        }
        public Vector3 GetClosestPointOnSpline(Vector3 pos,out Vector3 roadHeadding,out float smallestDistance)
        {
            smallestDistance = float.MaxValue;
            float tAtSmallestDistance = 0;
            Vector3 x_t = getPointAtT(0);
            Vector3 x_t_ = x_t;
            Vector3 closestPoint = x_t;
            const int n = 10;
            for (int i = 1; i <= n; i++)
            {
                float t = ((float)i) / n;
                x_t = getPointAtT(t);
                Vector3 closestPointCandidate = Math3d.ProjectPointOnLineSegment(x_t, x_t_, pos);
                float dist = Vector3.Distance(closestPointCandidate, pos);
                if (dist < smallestDistance)
                {
                    smallestDistance = dist;
                    closestPoint = closestPointCandidate;
                    tAtSmallestDistance = t;
                }
                x_t_ = x_t;
            }
            roadHeadding = Vector3.Normalize(getPointAtT(tAtSmallestDistance + 1e-5f) - closestPoint);
            //Debug.DrawRay(closestPoint, roadHeadding * 100, Color.blue, 1);
            return closestPoint;
        }

        public Vector3 getPointAtT(float t)
        {
            // comments are no use here... it's the catmull-rom equation.
            // Un-magic this, lord vector!
            return 0.5f *
                   ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                    (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t);
        }

        public override bool Equals(object obj)
        {
            if (!typeof(CatmulRomSpline).IsInstanceOfType(obj))
            {
                return false;
            }

            CatmulRomSpline other = obj as CatmulRomSpline;
            if(other.p0 == p0 && other.p1 == p1 && other.p2 == p2 && other.p3 == p3)
            {
                return true;
            }
            return false;
        }

        public override string ToString()
        {
            return string.Format("p0={0}, p1={1}, p2={2}, p3={3}",p0,p1,p2,p3);
        }
    }
}

namespace UnityStandardAssets.Utility.Inspector
{
#if UNITY_EDITOR
    [CustomPropertyDrawer(typeof (WaypointCircuit.WaypointList))]
    public class WaypointListDrawer : PropertyDrawer
    {
        private float lineHeight = 18;
        private float spacing = 4;


        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            EditorGUI.BeginProperty(position, label, property);

            float x = position.x;
            float y = position.y;
            float inspectorWidth = position.width;

            // Draw label


            // Don't make child fields be indented
            var indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;

            var items = property.FindPropertyRelative("items");
            var titles = new string[] {"Transform", "", "", ""};
            var props = new string[] {"transform", "^", "v", "-"};
            var widths = new float[] {.7f, .1f, .1f, .1f};
            float lineHeight = 18;
            bool changedLength = false;
            if (items.arraySize > 0)
            {
                for (int i = -1; i < items.arraySize; ++i)
                {
                    var item = items.GetArrayElementAtIndex(i);

                    float rowX = x;
                    for (int n = 0; n < props.Length; ++n)
                    {
                        float w = widths[n]*inspectorWidth;

                        // Calculate rects
                        Rect rect = new Rect(rowX, y, w, lineHeight);
                        rowX += w;

                        if (i == -1)
                        {
                            EditorGUI.LabelField(rect, titles[n]);
                        }
                        else
                        {
                            if (n == 0)
                            {
                                EditorGUI.ObjectField(rect, item.objectReferenceValue, typeof (Transform), true);
                            }
                            else
                            {
                                if (GUI.Button(rect, props[n]))
                                {
                                    switch (props[n])
                                    {
                                        case "-":
                                            items.DeleteArrayElementAtIndex(i);
                                            items.DeleteArrayElementAtIndex(i);
                                            changedLength = true;
                                            break;
                                        case "v":
                                            if (i > 0)
                                            {
                                                items.MoveArrayElement(i, i + 1);
                                            }
                                            break;
                                        case "^":
                                            if (i < items.arraySize - 1)
                                            {
                                                items.MoveArrayElement(i, i - 1);
                                            }
                                            break;
                                    }
                                }
                            }
                        }
                    }

                    y += lineHeight + spacing;
                    if (changedLength)
                    {
                        break;
                    }
                }
            }
            else
            {
                // add button
                var addButtonRect = new Rect((x + position.width) - widths[widths.Length - 1]*inspectorWidth, y,
                                             widths[widths.Length - 1]*inspectorWidth, lineHeight);
                if (GUI.Button(addButtonRect, "+"))
                {
                    items.InsertArrayElementAtIndex(items.arraySize);
                }

                y += lineHeight + spacing;
            }

            // add all button
            var addAllButtonRect = new Rect(x, y, inspectorWidth, lineHeight);
            if (GUI.Button(addAllButtonRect, "Assign using all child objects"))
            {
                var circuit = property.FindPropertyRelative("circuit").objectReferenceValue as WaypointCircuit;
                var children = new Transform[circuit.transform.childCount];
                int n = 0;
                foreach (Transform child in circuit.transform)
                {
                    children[n++] = child;
                }
                Array.Sort(children, new TransformNameComparer());
                circuit.waypointList.items = new Transform[children.Length];
                for (n = 0; n < children.Length; ++n)
                {
                    circuit.waypointList.items[n] = children[n];
                }
            }
            y += lineHeight + spacing;

            // rename all button
            var renameButtonRect = new Rect(x, y, inspectorWidth, lineHeight);
            if (GUI.Button(renameButtonRect, "Auto Rename numerically from this order"))
            {
                var circuit = property.FindPropertyRelative("circuit").objectReferenceValue as WaypointCircuit;
                int n = 0;
                foreach (Transform child in circuit.waypointList.items)
                {
                    child.name = "Waypoint " + (n++).ToString("000");
                }
            }
            y += lineHeight + spacing;

            // Set indent back to what it was
            EditorGUI.indentLevel = indent;
            EditorGUI.EndProperty();
        }


        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            SerializedProperty items = property.FindPropertyRelative("items");
            float lineAndSpace = lineHeight + spacing;
            return 40 + (items.arraySize*lineAndSpace) + lineAndSpace;
        }


        // comparer for check distances in ray cast hits
        public class TransformNameComparer : IComparer
        {
            public int Compare(object x, object y)
            {
                return ((Transform) x).name.CompareTo(((Transform) y).name);
            }
        }
    }


#endif
}
