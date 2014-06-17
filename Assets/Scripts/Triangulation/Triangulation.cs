﻿using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System; 
using Vectrosity;

[ExecuteInEditMode]
public class Triangulation : MonoBehaviour 
{
	//Data holder to display and save
	public List<Vector3> points = new List<Vector3>();
	public List<Color> colours = new List<Color>();
	// Use this for initialization

	public List<Triangle> triangles = new List<Triangle>(); 
	public List<Line> lines = new List<Line>(); 

	public List<Line> linesMinSpanTree = new List<Line>(); 
	public List<Geometry> obsLines = new List<Geometry> (); 
	public Geometry debugLines = new Geometry (); 

	public bool drawTriangles = false; 
	public bool drawRoadMap = false; 
	private bool drawMinSpanTree = false;
	public bool stopAll = false;

	public void Start(){
		debugLines.edges.Clear ();
	}

	public void Clear()
	{
		linesMinSpanTree.Clear(); 
		triangles.Clear(); 
		lines.Clear(); 
		points.Clear(); 
		colours.Clear();
		debugLines.edges.Clear ();

		GameObject temp = GameObject.Find("temp"); 
		DestroyImmediate(temp); 


		stopAll = true;
	}
	void OnDrawGizmosSelected() 
	{
		return; 
		//Debug.Log(colours.Count);
		//Debug.Log(points.Count);
		var i = 0;
		foreach(Vector3 v in points)
		{

			Gizmos.color = colours[i];
			//Gizmos.color = Color.red;
			Gizmos.DrawSphere (v, 0.25f);
			i++; 
		}

		//Gizmos.color = Color.red;
		//Gizmos.DrawSphere (new Vector3(0,2,0), 1);
	}
	public void Update()
	{

		//return; 
		//points.Clear(); 
		//colours.Clear();  
		if ( stopAll )
			return;
//		foreach (Geometry g in obsLines) {
//			foreach( Line l in g.edges ){
//				Debug.DrawLine(l.vertex[0],l.vertex[1], Color.blue);
//				debugLines.DrawGeometry(this.gameObject);
//			}
//		}
		
		/***
		if(drawMinSpanTree)
		{
			foreach(Line l in linesMinSpanTree)
			{
				l.DrawLine(Color.blue); 
			}
		}

		foreach(Triangle tt in triangles)
		{
			//triangulation.points.Add(tt.GetCenterTriangle());
			//triangulation.colours.Add(Color.cyan); 
			if(drawTriangles)
			{	

				tt.DrawDebug();
				foreach(Vector3 v in tt.getVertexMiddle())
				{
				//	points.Add(v);
				}

				foreach(Color v in tt.colourVertex)
				{
				//	colours.Add(v);
				}
			}

			if(drawRoadMap)
			{
				Line[] ll = tt.GetSharedLines(); 
			

				if(ll.Length == 1)
				{
					Debug.DrawLine(ll[0].MidPoint(), tt.GetCenterTriangle(),Color.red);
					//Debug.Log("Drawing Red Line at: " + ll[0].MidPoint() + " " + tt.GetCenterTriangle());
				}
				else if(ll.Length > 2)
				{
					for(int i = 0; i<ll.Length; i++)
					{
						Debug.DrawLine(ll[i].MidPoint(), tt.GetCenterTriangle(),Color.red);
						//Debug.Log("Drawing Red Line at: " + ll[i].MidPoint() + " " + tt.GetCenterTriangle());
					}

				}
				
				else
				{
					for(int i = 0; i<ll.Length; i++)
					{
						Debug.DrawLine(ll[i].MidPoint(), ll[(i+1) % ll.Length].MidPoint(),Color.red);
					}
				}
			}
		}
		***/
	}

	public void AddPoint(Vector3 v)
	{
		points.Add(v); 
		colours.Add(Color.cyan); 
	}

	public void AddPoint(Vector3 v,Color c)
	{
		points.Add(v); 
		colours.Add(c); 
	}

	public void TriangulationSpace ()
	{
		//Compute one step of the discritzation
		//Find this is the view
		GameObject floor = (GameObject)GameObject.Find ("Floor");
			
		Vector3 [] vertex = new Vector3[4]; 
		
		//First geometry is the outer one
		List<Geometry> geos = new List<Geometry> ();

		
		//Drawing lines
		//VectorLine.SetCamera3D(Camera.current); 

		//Floor
		Vector3[] f = new Vector3[4];
		MeshFilter mesh = (MeshFilter)(floor.GetComponent ("MeshFilter"));
		Vector3[] t = mesh.sharedMesh.vertices; 
		
		Geometry tempGeometry = new Geometry (); 
		
		
		vertex [0] = mesh.transform.TransformPoint (t [0]);
		vertex [2] = mesh.transform.TransformPoint (t [120]);
		vertex [1] = mesh.transform.TransformPoint (t [110]);
		vertex [3] = mesh.transform.TransformPoint (t [10]);
		
		vertex [0].y = 1; 
		vertex [1].y = 1; 
		vertex [2].y = 1; 
		vertex [3].y = 1; 
		//these were in tempGeometry previously

		//Disabled Temporarily - Find a way to avoid floor when checking for obstacle collision
		//geos.Add (tempGeometry);
		
		
		
		GameObject[] obs = GameObject.FindGameObjectsWithTag ("Obs");
		if(obs == null)
		{
			Debug.Log("Add tag geos to the geometries"); 
			return; 
		}
		//data holder
		Triangulation triangulation = GameObject.Find ("Triangulation").GetComponent<Triangulation> (); 
		triangulation.points.Clear ();
		triangulation.colours.Clear (); 
		
		//Only one geometry for now
		
		foreach (GameObject o in obs) {
			mesh = (MeshFilter)(o.GetComponent ("MeshFilter"));
			t = mesh.sharedMesh.vertices; 
			tempGeometry = new Geometry();

			vertex [0] = mesh.transform.TransformPoint (t [6]);
			vertex [1] = mesh.transform.TransformPoint (t [8]);
			vertex [3] = mesh.transform.TransformPoint (t [7]);
			vertex [2] = mesh.transform.TransformPoint (t [9]);
			
			vertex [0].y = 1;
			vertex [2].y = 1;
			vertex [1].y = 1;
			vertex [3].y = 1;
			for (int i = 0; i< vertex.Length; i+=1) {
				if (i < vertex.Length - 1)
				    tempGeometry.edges.Add (new Line (vertex [i], vertex [i + 1]));
				else 	       
					tempGeometry.edges.Add (new Line (vertex [0], vertex [i]));
			}	
			
			geos.Add (tempGeometry); 
			
		}
		
		//lines are defined by all the points in  obs
		//List<Line> lines = new List<Line> (); 
		lines = new List<Line> ();

		/*foreach (Geometry g in geos) {
			for (int i = 0; i< g.vertex.Length; i+=1) {
				if (i < g.vertex.Length - 1)
					lines.Add (new Line (g.vertex [i], g.vertex [i + 1]));
				else 	       
					lines.Add (new Line (g.vertex [0], g.vertex [i]));
				}
		}*/


		obsLines.Clear ();
		obsLines = geos;


		//Create empty GameObject
		GameObject temp = GameObject.Find("temp");
		DestroyImmediate(temp);
		temp = new GameObject("temp");

		foreach(Geometry g in geos)
			g.DrawGeometry(temp);

		//CODESPACE
		//Find out intersection. Reconstruct.



		//Draw a spere where the collision happenned
		foreach(Geometry g1 in geos)
		{
			foreach(Geometry g2 in geos)
			{
				if(g1 == g2 )
					continue; 
				g1.CollisionDraw(g2,temp);
				
			}
			g1.DrawVertex(temp);
		}



		for (int i = 0; i < geos.Count; i++) {
			for (int j = i + 1; j < geos.Count; j++) {
				//check all line intersections
				//Debug.Log("Inside Check");
				if( GeometryIntersect( i, j ) ){
					//Debug.Log("Obstacles Intersect " + i + " " + j);
					//if intersections
					//resolve to form new geometry at position j --resolve(i,j)
					tempGeometry = GeometryMerge( i, j );
					obsLines.RemoveAt(j);
					obsLines.RemoveAt(i);
					obsLines.Add(tempGeometry);
					//remove item at position i, decrement i since it will be increment in the next step, break
					i--;
				}

			}
		}








		///Uncomment the following later
		/*
		//Lines are also the one added. 

		//Compare each point to every point 


		for (int i = 0; i < geos.Count; i++) {
			
			for (int j = i+1; j < geos.Count; j++) {
				
				for (int w = 0; w<geos[i].vertex.Length; w++) {
					
					for (int z = 0; z<geos[j].vertex.Length; z++) {
						
						List<Line> toAdd = new List<Line> (); 
						
						Boolean foundBreak = false; 
						
						foreach (Line l in lines) {
							
							if (LineIntersection (geos [i].vertex [w], geos [j].vertex [z],
							                      l.vertex [0], l.vertex [1])) {
								
								foundBreak = true; 
								break; 
							}								
							
						}
						if (!foundBreak) {	
							//Debug.DrawLine(geos[i].vertex[w], geos[j].vertex[z], Color.blue);
							lines.Add (new Line (geos [i].vertex [w], geos [j].vertex [z])); 		
						}	
					}
				}
			}
		}
		
		//Find the centers 
		List<Triangle> triangles = new List<Triangle> (); 
		//Well why be efficient when you can be not efficient
		foreach (Line l in lines) {
			Vector3 v1 = l.vertex [0]; 
			Vector3 v2 = l.vertex [1];
			foreach (Line l2 in lines) {
				if (l == l2)
					continue;
				Vector3 v3 = Vector3.zero; 
				
				
				if (l2.vertex [0].Equals (v2)) {
					v3 = l2.vertex [1];
					//have to check if closes
				} else if (l2.vertex [1].Equals (v2)) {
					v3 = l2.vertex [0];
				}
				
				if (v3 != Vector3.zero) {
					foreach (Line l3 in lines) {
						if (l3 == l2 || l3 == l)
							continue; 
						if ((l3.vertex [0].Equals (v1) && l3.vertex [1].Equals (v3))
						    || (l3.vertex [1].Equals (v1) && l3.vertex [0].Equals (v3))) {
							//Debug.DrawLine(v1,v2,Color.red); 
							//Debug.DrawLine(v2,v3,Color.red); 
							//Debug.DrawLine(v3,v1,Color.red); 
							
							//Add the traingle
							Triangle toAddTriangle = new Triangle (
								v1, triangulation.points.IndexOf (v1),
								v2, triangulation.points.IndexOf (v2),
								v3, triangulation.points.IndexOf (v3));
							
							
							Boolean isAlready = false; 
							foreach (Triangle tt in triangles) {
								if (tt.Equals (toAddTriangle)) {
									//Debug.Log(toAddTriangle.refPoints[0]+", "+
									//          toAddTriangle.refPoints[1]+", "+
									//          toAddTriangle.refPoints[2]+", "); 
									isAlready = true; 
									break; 
								}
								
							}
							if (!isAlready) {
								triangles.Add (toAddTriangle);
							}
							
						}
					}
				}
			}
		}
		
		
		//Find shared edge and triangle structure
		
		foreach (Triangle tt in triangles) {
			foreach (Triangle ttt in triangles) {
				if (tt == ttt)
					continue; 
				tt.ShareEdged (ttt);
				
			}
			
		}
		
		triangulation.triangles = triangles; */
		
		
		
	}

	private bool pointInsideGeo( Vector3 point, Geometry G ){
		float minx = float.MaxValue, minz = float.MaxValue;

		foreach (Line l in G.edges) {
			if( minx > l.vertex[0].x ) minx = l.vertex[0].x;
			if( minx > l.vertex[1].x ) minx = l.vertex[1].x;

			if( minz > l.vertex[0].z ) minz = l.vertex[0].z;
			if( minz > l.vertex[1].z ) minz = l.vertex[1].z;
		}
		minx -= 10;
		minz -= 10;
		Vector3 extPoint = new Vector3 (minx, 1.0f, minz);

		int intersects = 0;
		foreach (Line l in G.edges) {
			if( LineIntersect( extPoint, point, l.vertex[0], l.vertex[1] ) > 0 )
			   intersects++;
		}
		if( intersects % 2 == 0 )
			return false;
		return true;
	}

	private Geometry GeometryMerge( int x, int y ){
		Geometry tempGeometry = new Geometry ();
		//Two Geometry objects - G1 and G2
		Geometry G1 = obsLines [x];
		Geometry G2 = obsLines [y];
		//Create new called G3 which starts as an union of G1 and G2
		Geometry G3 = new Geometry ();
		foreach (Line l in G1.edges)
			G3.edges.Add(l);		
		foreach (Line l in G2.edges)
			G3.edges.Add(l);		


		//Check for intersection points among lines in G3
		for (int i = 0; i < G3.edges.Count; i++) {
			for( int j = i + 1; j < G3.edges.Count; j++ ) {
				Vector3 vA = G3.edges[i].vertex[0];
				Vector3 vB = G3.edges[i].vertex[1];
				Vector3 vC = G3.edges[j].vertex[0];
				Vector3 vD = G3.edges[j].vertex[1];
				int caseType = LineIntersect( vA, vB, vC, vD );
				if( caseType == 1 ){//Regular intersections
					Vector3 pt = LineIntersectionPoint( vA, vB, vC, vD );
					if( EndPointIntersecion( pt, vA, vB, vC, vD ) > 0 )
						continue;

					Line lne = new Line( pt, vA );
					G3.edges.Add( lne );
					lne = new Line( pt, vB );
					G3.edges.Add( lne );
					lne = new Line( pt, vC );
					G3.edges.Add( lne );
					lne = new Line( pt, vD );
					G3.edges.Add( lne );
					G3.edges.RemoveAt(j);
					G3.edges.RemoveAt(i);
					i--;
					break;
				}
				/*else if( caseType == 2 ){//Colinear overlapping lines
					//Case 1: no shared endpoints

					bool xCol = false; //colinear on x-axis
					bool yCol = false; //colinear on y-axis
					if( vA.x == vB.x && vB.x == vC.x && vC.x == vD.x )
						yCol = true;
					else if( vA.y == vB.y && vB.y == vC.y && vC.y == vD.y )
						xCol = true;

					Vector3 pt = LineIntersectionPoint( vA, vB, vC, vD );
					Line lne = new Line( vA, vB );
					//Debug.Log("Hello");
					//No need to find intersections. Can determine by minx, maxx checks
					if( yCol ){
						if( vA.y > vB.y )
							Swap( ref vA, ref vB );
					}
					else{
						if( vA.x > vB.x )
							Swap( ref vA, ref vB );
					}

					if( EndPointIntersecion( pt, vA, vB, vC, vD ) > 0 ){////Case 2A: Shared endpoint

						bool removeAtI = false;
						bool removeAtJ = false;

						Vector3 otherEndPt;
						if( pt == vC ) otherEndPt = vD;
						else otherEndPt = vC;

						if( pt == vB ){//common endpoint at vB
							if( yCol ){
								if( otherEndPt.y < vA.y )
									removeAtJ = true;
								else if( otherEndPt.y < vB.y )
									removeAtI = true;
							}
							else{
								if( otherEndPt.x < vA.x )
									removeAtJ = true;
								else if( otherEndPt.x < vB.x )
									removeAtI = true;
							}
						}
						else{//common endpoint at vA
							if( yCol ){
								if( otherEndPt.y > vB.y )
									removeAtJ = true;
								else if( otherEndPt.y > vA.y )
									removeAtI = true;
							}
							else{
								if( otherEndPt.x > vB.x )
									removeAtJ = true;
								else if( otherEndPt.x > vA.x )
									removeAtI = true;
							}
						}

						if( removeAtI ){
							//G3.edges.RemoveAt(i);
							//i--;
							//Debug.Log("REMOVING i");
							//break;
						}
						else if( removeAtJ ){
							//G3.edges.RemoveAt(j);
							//j--;
							//Debug.Log("REMOVING j");
						}
						else continue;
					}
					else{//Case 2B: No shared endpoints
						Debug.DrawRay( vA, Vector3.up, Color.cyan );
						Debug.DrawRay( vB, Vector3.up, Color.cyan );
						Debug.DrawRay( vC, Vector3.up );
						Debug.DrawRay( vD, Vector3.up );
						List<Vector3> pointList = new List<Vector3>();
						pointList.Add( vA );
						pointList.Add( vB );
						pointList.Add( vC );
						pointList.Add( vD );
						sortPointsX sopx = new sortPointsX();
						sortPointsY sopy = new sortPointsY();
						if( yCol )
							pointList.Sort( sopy );
						else
							pointList.Sort( sopx );
						
						for( int k = 0; k < 3; k++ ){
							G3.edges.Add( new Line( pointList[k], pointList[k + 1] ) );
						}
						G3.edges.RemoveAt(j);
						G3.edges.RemoveAt(i);
						i--;
						break;
					}
				}*/
			}
		}

		//Check A: Points inside Polygon
		//Check all points in G1 with original G2. If inside remove all related lines from G3.
		//copying g3 for modification purposes

		for (int i = 0; i < G3.edges.Count; i++) {
			Vector3	pt1 = G3.edges[i].vertex[0];
			Vector3	pt2 = G3.edges[i].vertex[1];
			if( pointInsideGeo( pt1, G1 ) || pointInsideGeo( pt2, G1 ) || pointInsideGeo( pt1, G2 ) || pointInsideGeo( pt2, G2 ) ){
				//Debug.DrawRay( pt1, Vector3.up, Color.red );
				//Debug.DrawLine( pt1, new Vector3( 100, 1, 100 ), Color.red);
				debugLines.edges.Add(new Line( pt1, new Vector3( 20, 1, 20 ) ) );
				debugLines.edges.Add(new Line( pt2, new Vector3( 20, 1, 20 ) ) );
				Debug.Log("point inside " + pt1 + " " + pt2);
				//G3.edges.RemoveAt(i);
				//i--;
			}
		}

			//OTHER Cases
		//Check B: Check for Internal Line Segment Portions that are inside the polygon
		//Take all the lines in G3. Check for intersection with all other lines in G3.
		//If intersection doesn't consist of an endpoint from each line then remove
		/*
		for (int i = 0; i < G3.edges.Count; i++) {
			Vector3	pt = G3.edges[i].vertex[0];
			pt.x = (G3.edges[i].vertex[0].x + G3.edges[i].vertex[1].x) / 2;
			pt.z = (G3.edges[i].vertex[0].z + G3.edges[i].vertex[1].z) / 2;
			if( pointInsideGeo( pt, G1 ) ){
				//G3.edges.RemoveAt(i);
				//i--;
				//Debug.DrawRay( pt, Vector3.up );
				Debug.Log("woah1");
				//debugLines.edges.Add( new Line( pt, new Vector3(100, 1, 100) ) );
			}
			else if( pointInsideGeo( pt, G2 ) ){
				Debug.Log("woah2");
				//debugLines.edges.Add( new Line( pt, new Vector3(100, 1, 100) ) );
				//G3.edges.RemoveAt(i);
				//i--;
			}
		}
		//Copy new G3
		/*G3temp.edges.Clear ();
		foreach (Line LA in G3.edges) 
			G3temp.edges.Add(LA);
		//Trim
		foreach (Line LA in G3temp.edges) {
			foreach (Line LB in G3temp.edges) {
				if( LA == LB ) continue;
				if( LineIntersect( LA.vertex[0], LA.vertex[1], LB.vertex[0], LB.vertex[1] ) ){
					Vector3 pt = LineIntersectionPoint( LA.vertex[0], LA.vertex[1], LB.vertex[0], LB.vertex[1] );
					if( pt 
				}
			}
			Vector3 pt = new Vector3( (LA.vertex[0].x + LA.vertex[1].x)/2f, 1f, (LA.vertex[0].z + LA.vertex[1].z)/2f );
			if( pointInsideGeo( pt, G1 ) || pointInsideGeo( pt, G2 )  )
				G3.edges.Remove(LA);
		}*/

		return G3;
	}

	private bool GeometryIntersect( int x, int y ){
		foreach( Line La in obsLines[x].edges ){
			foreach( Line Lb in obsLines[y].edges ){
				if( LineIntersect( La.vertex[0], La.vertex[1], Lb.vertex[0], Lb.vertex[1] ) > 0 ){
					/*Debug.Log(La.vertex[0] + "  " + La.vertex[1] );
					Debug.Log(Lb.vertex[0] + "  " + Lb.vertex[1] );
					Vector3 vv = LineIntersectionPoint( La.vertex[0], La.vertex[1], Lb.vertex[0], Lb.vertex[1] );
					Debug.Log(vv);
					Debug.Log ("BREAK");*/
					return true;
				}
			}
		}
		return false;
	}

	private int LineIntersect (Vector3 a, Vector3 b, Vector3 c, Vector3 d){
		Vector2 u = new Vector2 (b.x, b.z) - new Vector2 (a.x, a.z);
		Vector2 p0 = new Vector2 (a.x, a.z);
				
		Vector2 v = new Vector2 (d.x, d.z) - new Vector2 (c.x, c.z);
		Vector2 q0 = new Vector2 (c.x, c.z);

		double numerator1 = CrossProduct ((q0 - p0), v);
		double numerator2 = CrossProduct ((q0 - p0), u);
		double denom = CrossProduct (u, v);

		//Case 1 - Colinear
		if ( denom == 0 && numerator2 == 0 ) {
			//Case 2 - Colinear and Overlapping
			if( Vector2.Dot( (q0 - p0), u ) >= 0 && Vector2.Dot( (q0 - p0), u ) <= Vector2.Dot( u, u ) )
				return 2;
			if( Vector2.Dot( (p0 - q0), v ) >= 0 && Vector2.Dot( (p0 - q0), v ) <= Vector2.Dot( v, v ) )
				return 2;
			return 0;
		}
		//Case 3 - Parallel
		if (denom == 0 && numerator2 != 0)
			return 0;

		//Case 4 - Intersects
		double s = numerator1 / denom;
		 	double t = numerator2 / denom;
							
		if ((s >= 0 && s <= 1) && (t >= 0 && t <= 1))
			return 1;
		
		return 0; 
	}

	private Vector3 LineIntersectionPoint (Vector3 a, Vector3 b, Vector3 c, Vector3 d){
		Vector2 u = new Vector2 (b.x, b.z) - new Vector2 (a.x, a.z);
		Vector2 p0 = new Vector2 (a.x, a.z);
		
		Vector2 v = new Vector2 (d.x, d.z) - new Vector2 (c.x, c.z);
		Vector2 q0 = new Vector2 (c.x, c.z);
		
		double numerator1 = CrossProduct ((q0 - p0), v);
		double numerator2 = CrossProduct ((q0 - p0), u);
		double denom = CrossProduct (u, v);
		
		double s = numerator1 / denom;
		double t = numerator2 / denom;
		
		Vector3 r = a + (b-a)*(float)s; 
		return r;
	}

	private double CrossProduct( Vector2 a, Vector2 b ){
		return (a.x * b.y) - (a.y * b.x);
	}

	private Vector3 LineIntersectVect (Vector3 a, Vector3 b, Vector3 c, Vector3 d)
	{
		//Debug.Log(a); 
		//Debug.Log(b); 
		//Debug.Log(c); 
		//Debug.Log(d); 
		
		Vector2 u = new Vector2 (b.x, b.z) - new Vector2 (a.x, a.z);
		Vector2 p0 = new Vector2 (a.x, a.z);
		Vector2 p1 = new Vector2 (b.x, b.z); 
		
		Vector2 v = new Vector2 (d.x, d.z) - new Vector2 (c.x, c.z);
		Vector2 q0 = new Vector2 (c.x, c.z);
		Vector2 q1 = new Vector2 (d.x, d.z);
		
		Vector2 w = new Vector2 (a.x, a.z) - new Vector2 (d.x, d.z);
		
		
		//if (u.x * v.y - u.y*v.y == 0)
		//	return true;
		
		double s = (v.y * w.x - v.x * w.y) / (v.x * u.y - v.y * u.x);
		double t = (u.x * w.y - u.y * w.x) / (u.x * v.y - u.y * v.x); 
		//Debug.Log(s); 
		//Debug.Log(t); 
		
		//if ((s > 0 && s < 1) || (t > 0 && t < 1))
		//{
		//Interpolation
		Vector3 r = a + (b-a)*(float)s; 
		return r; 
		//}
		
		
		
		//return Vector3.zero; 
	}
	/*private Boolean LineIntersection (Vector3 a, Vector3 b, Vector3 c, Vector3 d)
	{
		
		
		
		// a-b
		// c-d
		//if the same lines
		
		//When share a point use the other algo
		if (a.Equals (c) || a.Equals (d) || b.Equals (c) || b.Equals (d))
			return LineIntersect (a, b, c, d); 
		
		
		
		
		return CounterClockWise (a, c, d) != CounterClockWise (b, c, d) && 
			CounterClockWise (a, b, c) != CounterClockWise (a, b, d);
		
		//if( CounterClockWise(a,c,d) == CounterClockWise(b,c,d))
		//	return false;
		//else if (CounterClockWise(a,b,c) == CounterClockWise(a,b,d))
		//	return false; 
		//else 
		//	return true; 
		
		
	}*/

	//Checks if two edges are meeting regularly at a vertex of the polygon or if they are intersecting
	//in other manners
	int EndPointIntersecion( Vector3 pt, Vector3 v1, Vector3 v2, Vector3 v3, Vector3 v4 ){
		Vector3 pt2;
		//Find common endpoint
		if (v1 == v3)
			pt2 = v1;
		else if( v1 == v4 )
			pt2 = v1;
		else if( v2 == v3 )
			pt2 = v2;
		else if( v2 == v4 )
			pt2 = v2;
		else
			return 0;

		return 1;
		/*if (Math.Abs (pt.x - pt2.x) < 0.01 && Math.Abs (pt.z - pt2.z) < 0.01)
			return 1; //Endpoint and intersection point same
		else
			return 2;*/
	}

	private Boolean CounterClockWise (Vector3 v1, Vector3 v2, Vector3 v3)
	{
		//v1 = a,b
		//v2 = c,d
		//v3 = e,f
		
		float a = v1.x, b = v1.z;  
		float c = v2.x, d = v2.z;  
		float e = v3.x, f = v3.z;  
		
		if ((f - b) * (c - a) > (d - b) * (e - a))
			return true;
		else
			return false; 
	}

	public class sortPointsX : IComparer<Vector3>
	{
		public int Compare(Vector3 a, Vector3 b)
		{
			if (a.x > b.x) return 1;
			else if (a.x < b.x) return -1;
			else return 0;
		}
	}

	public class sortPointsY : IComparer<Vector3>
	{
		public int Compare(Vector3 a, Vector3 b)
		{
			if (a.y > b.y) return 1;
			else if (a.y < b.y) return -1;
			else return 0;
		}
	}

	private static void Swap<Vector3>(ref Vector3 lhs, ref Vector3 rhs){
		Vector3 temp;
		temp = lhs;
		lhs = rhs;
		rhs = temp;
	}
}
