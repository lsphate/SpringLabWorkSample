package SpringSolid.SynchronousModeling;

import SpringSolid.CADDesign.*;
import SpringSolid.Mathematics.math_vector3d;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;

public class synchronous_angular_dimension implements SpringSolid.Part42.Define
{

    private step_manifold_solid_brep importBody;
    private step_face_array selectFace;
    private double adjAngle;
    private double currentAngle;
    private step_cartesian_point_array cartPointArray;
    private math_vector3d direction;
    private step_edge_curve_array commonEdge;

    public synchronous_angular_dimension(part_body pBody, step_face_array selectFace, double angle)
    {
        this.importBody = pBody.GetFeatureManager().GetResultBodyArray().Get(0);
        this.adjAngle = angle;
        this.selectFace = selectFace;
        importBody.CheckCorrectness();
    }

    public step_manifold_solid_brep GetSynchronousBody()
    {
        return this.importBody;
    }

    public void AngularDimensionAdjust()
    {
        commonEdge = new step_edge_curve_array();
        selectFace.Get(0).GetActiveFace().FindCommonEdges(selectFace.Get(1).GetActiveFace(), commonEdge);
        step_vertex_point_array commonEdgeVertex = new step_vertex_point_array();
        commonEdge.Get(0).GetVertices(commonEdgeVertex);
        step_vertex_point_array adjFaceVertex = new step_vertex_point_array();
        selectFace.Get(1).GetActiveFace().GetVertices(adjFaceVertex);
        step_vertex_point_array adjPoint = new step_vertex_point_array();

        step_vertex_point tempVertex = new step_vertex_point();
        for (int i = 0; i < adjFaceVertex.Count(); i++)
        {
            if (adjFaceVertex.Get(i) != commonEdgeVertex.Get(0) && adjFaceVertex.Get(i) != commonEdgeVertex.Get(1))
            {
                adjPoint.Add(adjFaceVertex.Get(i));
            }
        }

        step_edge_curve sideEdge = new step_edge_curve();
        step_edge_curve_array adjFaceEdge = new step_edge_curve_array();
        selectFace.Get(1).GetActiveFace().GetEdgeCurves(adjFaceEdge);
        for (int i = 0; i < adjFaceEdge.Count(); i++)
        {
            boolean isSideEdge = adjFaceEdge.Get(i).IsTwoVerticesAttached(adjPoint.Get(0), adjPoint.Get(1));
            if (isSideEdge)
            {
                sideEdge = adjFaceEdge.Get(i);
            }
        }

        step_face_array tempSideFaceArray = new step_face_array();
        selectFace.Get(1).GetActiveFace().GetBody().GetAdjacentFaces(sideEdge, tempSideFaceArray);
        step_advanced_face sideFace = new step_advanced_face();
        for (int i = 0; i < tempSideFaceArray.Count(); i++)
        {
            if (tempSideFaceArray.Get(i).IDNumber() != selectFace.Get(1).IDNumber())
            {
                sideFace = tempSideFaceArray.Get(i).GetActiveFace();
            }
        }


        math_vector3d adjFaceRefptCoord = new math_vector3d();
        math_vector3d adjFaceNorDir = new math_vector3d();
        math_vector3d adjFaceRefDirVector = new math_vector3d();
        

        this.DimensionAdjust(sideFace, adjPoint, adjFaceRefptCoord, adjFaceNorDir, adjFaceRefDirVector);

        ((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).SetRefptCoord(adjFaceRefptCoord);
        ((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).SetAxisVect(adjFaceNorDir);
        step_direction adjFaceRefDir = new step_direction(adjFaceRefDirVector);
        ((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).GetPosition().SetRefDirection(adjFaceRefDir);

    }

    public void DimensionAdjust(step_advanced_face sideFace, step_vertex_point_array adjPoint,math_vector3d adjFaceRefptCoord, math_vector3d adjFaceNorDir, math_vector3d adjFaceRefDirVector)
    {
        math_vector3d checkRotation = ((step_plane) selectFace.Get(0).GetActiveFace().GetFaceGeometry()).AxisVector().CrossProduct(((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).AxisVector());
        if (checkRotation.Z() > 0)
        {
            adjAngle = adjAngle * -1.0;
        }
        
        if (sideFace.GetFaceGeometry().IsCylindricalSurface())
        {
            adjAngle = adjAngle * -1.0;
        }
        
        adjAngle = adjAngle / 180 * Math.PI;
        
        math_vector3d tempFaceRefptCoord = ((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).RefptCoord().RotationAboutAxis(commonEdge.Get(0).StartCoord(), commonEdge.Get(0).GetEdgeGeometry().DirectVector(), adjAngle);
        math_vector3d tempFaceNorDir = ((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).AxisVector().RotationAboutAxis(commonEdge.Get(0).StartCoord(), commonEdge.Get(0).GetEdgeGeometry().DirectVector(), adjAngle);
        math_vector3d tempFaceRefDirVector = ((step_plane) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).RefDirectionVector().RotationAboutAxis(commonEdge.Get(0).StartCoord(), commonEdge.Get(0).GetEdgeGeometry().DirectVector(), adjAngle);
        adjFaceRefptCoord.Set(tempFaceRefptCoord);
        adjFaceNorDir.Set(tempFaceNorDir);
        adjFaceRefDirVector.Set(tempFaceRefDirVector);

        if (sideFace.GetFaceGeometry().IsCylindricalSurface())
        {
            for (int i = 0; i < adjPoint.Count(); i++)
            {
                adjPoint.Get(i).RotationAboutAxis(commonEdge.Get(0).StartCoord(), commonEdge.Get(0).GetEdgeGeometry().DirectVector(), adjAngle);
            }
        }
        else
        {
            step_vertex_point_array rotateVertexArray = new step_vertex_point_array();
            for (int i = 0; i < adjPoint.Count(); i++)
            {
                rotateVertexArray.Set(i, adjPoint.Get(i).Copy());
                rotateVertexArray.Get(i).RotationAboutAxis(commonEdge.Get(0).StartCoord(), commonEdge.Get(0).GetEdgeGeometry().DirectVector(), adjAngle);
            }
            math_vector3d projectDir = new math_vector3d();
            projectDir = rotateVertexArray.Get(0).GetVertexGeometry().GetCoord().Subtract(commonEdge.Get(0).StartCoord()).Normalize();
            for (int i = 1; i < rotateVertexArray.Count(); i++)
            {
                double length = projectDir.Length();
                if (rotateVertexArray.Get(i).Distance(commonEdge.Get(0).GetEdgeStart()) <= length)
                {
                    projectDir = rotateVertexArray.Get(i).GetVertexGeometry().GetCoord().Subtract(commonEdge.Get(0).StartCoord()).Normalize();
                }
            }

            math_vector3d_array intersectPoint1 = new math_vector3d_array();
            math_vector3d_array intersectPoint2 = new math_vector3d_array();
            short isIntersect1 = sideFace.RayIntersect(rotateVertexArray.Get(0).GetVertexGeometry().GetCoord(), projectDir, intersectPoint1);
            short isIntersect2 = sideFace.RayIntersect(rotateVertexArray.Get(1).GetVertexGeometry().GetCoord(), projectDir, intersectPoint2);
            if (isIntersect1 == 1 && isIntersect2 == 1)
            {
                adjPoint.Get(0).GetVertexGeometry().SetCoord(intersectPoint1.Get(0));
                adjPoint.Get(1).GetVertexGeometry().SetCoord(intersectPoint2.Get(0));
            }
            else if (isIntersect1 != 1 || isIntersect2 != 1)
            {
                math_vector3d moveFaceNormal = selectFace.Get(1).GetActiveFace().GetFaceGeometry().DirectVector();
                do
                {
                    adjPoint.Get(0).GetVertexGeometry().SetCoord(adjPoint.Get(0).GetVertexGeometry().GetCoord().Add(moveFaceNormal.Multiply(50.0)));
                    adjPoint.Get(1).GetVertexGeometry().SetCoord(adjPoint.Get(1).GetVertexGeometry().GetCoord().Add(moveFaceNormal.Multiply(50.0)));
//                    sideFace.WriteToSurfaceModel("D:\\Graduate\\Paper\\SynMo\\20121121\\sideFace.stp");
                    isIntersect1 = sideFace.GetFaceGeometry().RayIntersect(rotateVertexArray.Get(0).GetVertexGeometry().GetCoord(), projectDir, intersectPoint1);
                    isIntersect2 = sideFace.GetFaceGeometry().RayIntersect(rotateVertexArray.Get(1).GetVertexGeometry().GetCoord(), projectDir, intersectPoint2);
                }
                while (isIntersect1 != 1 || isIntersect2 != 1);

                adjPoint.Get(0).GetVertexGeometry().SetCoord(intersectPoint1.Get(0));
                adjPoint.Get(1).GetVertexGeometry().SetCoord(intersectPoint2.Get(0));
            }
        }

    }

    public void GetVectorAndAngle(math_vector3d oriFaceDir, math_vector3d adjFaceDir, double radAngle)
    {
        step_vertex_point_array shareEdgeVertex = new step_vertex_point_array();
        commonEdge.Get(0).GetVertices(shareEdgeVertex);
        step_vertex_point startVertex = shareEdgeVertex.Get(0);

        //Find Origin Face Direction Vector
        step_vertex_point_array oriFaceVertex = new step_vertex_point_array();
        selectFace.Get(0).GetActiveFace().GetVertices(oriFaceVertex);
        step_vertex_point_array oriCandiVertex = new step_vertex_point_array();
        for (int i = 0; i < oriFaceVertex.Count(); i++)
        {
            if (oriFaceVertex.Get(i).IDNumber() != shareEdgeVertex.Get(0).IDNumber() && oriFaceVertex.Get(i).IDNumber() != shareEdgeVertex.Get(1).IDNumber())
            {
                oriCandiVertex.Add(oriFaceVertex.Get(i));
            }
        }
        step_vertex_point oriEndVertex = oriCandiVertex.Get(0);
        double length = oriCandiVertex.Get(0).Distance(startVertex);

        for (int i = 1; i < oriCandiVertex.Count(); i++)
        {
            if (oriCandiVertex.Get(i).Distance(startVertex) < length)
            {
                length = oriCandiVertex.Get(i).Distance(startVertex);
                oriEndVertex = oriCandiVertex.Get(i);
            }
        }
        oriFaceDir = oriEndVertex.GetVertexGeometry().GetCoord().Subtract(startVertex.GetVertexGeometry().GetCoord());

        //Find Adjust Face Direction Vector
        step_vertex_point_array adjFaceVertex = new step_vertex_point_array();
        selectFace.Get(1).GetActiveFace().GetVertices(adjFaceVertex);
        step_vertex_point_array adjCandiVertex = new step_vertex_point_array();
        for (int i = 0; i < adjFaceVertex.Count(); i++)
        {
            if (adjFaceVertex.Get(i).IDNumber() != shareEdgeVertex.Get(0).IDNumber() && adjFaceVertex.Get(i).IDNumber() != shareEdgeVertex.Get(1).IDNumber())
            {
                adjCandiVertex.Add(adjFaceVertex.Get(i));
            }
        }
        step_vertex_point adjEndVertex = adjCandiVertex.Get(0);
        length = adjCandiVertex.Get(0).Distance(startVertex);

        for (int i = 1; i < adjCandiVertex.Count(); i++)
        {
            if (adjCandiVertex.Get(i).Distance(startVertex) < length)
            {
                length = adjCandiVertex.Get(i).Distance(startVertex);
                adjEndVertex = adjCandiVertex.Get(i);
            }
        }
        adjFaceDir = adjEndVertex.GetVertexGeometry().GetCoord().Subtract(startVertex.GetVertexGeometry().GetCoord());

        radAngle = this.LawofCosines(oriFaceDir, adjFaceDir);
    }

    public double LawofCosines(math_vector3d v1, math_vector3d v2)
    {
        double dotProduct = v1.DotProduct(v2);
        double v1length = v1.Length();
        double v2length = v2.Length();
        double cos = dotProduct / (v1length * v2length);
        double theta = Math.acos(cos);
        return theta;
    }
}
