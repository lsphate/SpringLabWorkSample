package SpringSolid.SynchronousModeling;

import SpringSolid.CADDesign.*;
import SpringSolid.Mathematics.math_vector3d;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;

public class synchronous_linear_dimension implements SpringSolid.Part42.Define
{

    private step_manifold_solid_brep importBody;
    private step_vertex_point_array selectPoint;
    private double distance;
    private step_cartesian_point_array cartPointArray;
    private math_vector3d direction;

    public synchronous_linear_dimension(part_body pBody, step_vertex_point_array selectPoint, double distance)
    {
        this.importBody = pBody.GetFeatureManager().GetResultBodyArray().Get(0);
        this.distance = distance;
        this.selectPoint = selectPoint;
    }

    public step_manifold_solid_brep GetSynchronousBody()
    {
        return this.importBody;
    }

    public void LinearDimensionAdjust()
    {
        direction = selectPoint.Get(1).GetVertexGeometry().GetCoord().SubtractThenNormalize(selectPoint.Get(0).GetVertexGeometry().GetCoord());
        step_face_array candidateFace = new step_face_array();
        step_advanced_face adjFace = new step_advanced_face();
        step_face_array importBodyFace = new step_face_array();
        importBody.GetAdvancedFaces(importBodyFace);
        math_vector3d tempNormal;
        for (int i = 0; i < importBodyFace.Count(); i++)
        {
            tempNormal = new math_vector3d();
            importBodyFace.Get(i).GetActiveFace().GetFaceGeometry().GetNormalWithNormalize(0, 0, tempNormal);

            if (((step_advanced_face) importBodyFace.Get(i)).FindFaceNormal().Z() < 0.0)
            {
                tempNormal = tempNormal.Multiply(-1.0);
            }

            if (direction.DotProduct(tempNormal) == 1.0)
            {
                candidateFace.Add((step_advanced_face) importBodyFace.Get(i));
            }
        }

        for (int i = 0; i < candidateFace.Count(); i++)
        {
            short on = ((step_advanced_face) candidateFace.Get(i)).On(selectPoint.Get(1).GetVertexGeometry().GetCoord());
            if (on == 3)
            {
                adjFace = (step_advanced_face) candidateFace.Get(i);
            }
        }
        step_vertex_point_array adjFaceVertices = new step_vertex_point_array();
        adjFace.GetVertices(adjFaceVertices);
        
        step_cartesian_point_array adjFaceLocations = new step_cartesian_point_array();
        step_edge_curve_array adjFaceECArray = new step_edge_curve_array();
        adjFace.GetEdgeCurves(adjFaceECArray);
        for (int i = 0; i < adjFaceECArray.Count(); i++)
        {
            if (adjFaceECArray.Get(i).GetEdgeGeometry().IsCircle())
            {
                adjFaceLocations.Add(((step_circle)adjFaceECArray.Get(i).GetEdgeGeometry()).GetRefpt());
            }
        }


        adjFaceLocations.Add(((step_plane) adjFace.GetFaceGeometry()).GetRefpt());
        this.DimensionAdjust(adjFaceVertices, adjFaceLocations);
//        importBody.WriteToSolidModel("D:\\Graduate\\Paper\\SynMo\\20121022\\final.stp");
    }

    public void DimensionAdjust(step_vertex_point_array adjFaceVertices, step_cartesian_point_array adjFaceLocations)
    {
        for (int i = 0; i < adjFaceVertices.Count(); i++)
        {
            adjFaceVertices.Get(i).GetVertexGeometry().SetCoord(adjFaceVertices.Get(i).GetVertexGeometry().GetCoord().Add(direction.Multiply(distance)));

        }
        for (int i = 0; i < adjFaceLocations.Count(); i++)
        {
            adjFaceLocations.Get(i).SetCoord(adjFaceLocations.Get(i).GetCoord().Add(direction.Multiply(distance)));
        }
        
//        math_vector3d controler = new math_vector3d(1.0, 2.0, 3.0);
//        String selectcase = String.valueOf(direction.DotProduct(controler));
//        switch (selectcase)
//        {
//            case "1.0":
//                adjFaceLocation.SetX(adjFaceLocation.GetX() + distance);
//                for (int i = 0; i < adjFaceVertices.Count(); i++)
//                {
//                    adjFaceVertices.Get(i).GetVertexGeometry().SetX(adjFaceVertices.Get(i).GetVertexGeometry().GetX() + distance);
//                }
//                break;
//            case "-1.0":
//                adjFaceLocation.SetX(adjFaceLocation.GetX() - distance);
//                for (int i = 0; i < adjFaceVertices.Count(); i++)
//                {
//                    adjFaceVertices.Get(i).GetVertexGeometry().SetX(adjFaceVertices.Get(i).GetVertexGeometry().GetX() - distance);
//                }
//                break;
//            case "2.0":
//                adjFaceLocation.SetY(adjFaceLocation.GetY() + distance);
//                for (int i = 0; i < adjFaceVertices.Count(); i++)
//                {
//                    adjFaceVertices.Get(i).GetVertexGeometry().SetY(adjFaceVertices.Get(i).GetVertexGeometry().GetY() + distance);
//                }
//                break;
//            case "-2.0":
//                adjFaceLocation.SetY(adjFaceLocation.GetY() - distance);
//                for (int i = 0; i < adjFaceVertices.Count(); i++)
//                {
//                    adjFaceVertices.Get(i).GetVertexGeometry().SetY(adjFaceVertices.Get(i).GetVertexGeometry().GetY() - distance);
//                }
//                break;
//            case "3.0":
//                adjFaceLocation.SetZ(adjFaceLocation.GetZ() + distance);
//                for (int i = 0; i < adjFaceVertices.Count(); i++)
//                {
//                    adjFaceVertices.Get(i).GetVertexGeometry().SetZ(adjFaceVertices.Get(i).GetVertexGeometry().GetZ() + distance);
//                }
//                break;
//            case "-3.0":
//                adjFaceLocation.SetZ(adjFaceLocation.GetZ() - distance);
//                for (int i = 0; i < adjFaceVertices.Count(); i++)
//                {
//                    adjFaceVertices.Get(i).GetVertexGeometry().SetZ(adjFaceVertices.Get(i).GetVertexGeometry().GetZ() - distance);
//                }
//                break;
//        }
    }
}
