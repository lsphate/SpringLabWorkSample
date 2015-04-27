package SpringSolid.SynchronousModeling;

import SpringSolid.CADDesign.*;
import SpringSolid.Mathematics.math_vector3d;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;
import SpringSolid.SurfaceDesign.plane_split;

public class synchronous_split_body implements SpringSolid.Part42.Define
{

    private step_manifold_solid_brep importBody;
    private step_advanced_face toolFace;
    private double distance;
    private step_manifold_solid_brep_array finalBody;

    public synchronous_split_body(part_body pBody, step_advanced_face selectFace, double distance)
    {
        this.importBody = pBody.GetFeatureManager().GetResultBodyArray().Get(0);
        this.toolFace = selectFace.Copy();
        this.distance = distance;

        this.finalBody = new step_manifold_solid_brep_array();
    }

    public step_manifold_solid_brep_array GetSynchronousBody()
    {
        return this.finalBody;
    }

    public void SplitBody()
    {
        math_vector3d tempVertex = new math_vector3d();
        step_vertex_point_array toolFaceVertices = new step_vertex_point_array();
        math_vector3d toolFaceNormal = new math_vector3d();

        toolFace.GetCandidateVertex(tempVertex);
        ((step_plane) toolFace.GetFaceGeometry()).GetNormal(tempVertex, toolFaceNormal);
        toolFace.GetVertices(toolFaceVertices);

        if (toolFace.FindFaceNormal().Z() < 0.0)
        {
            distance = distance * -1;
        }

        if (toolFaceNormal.X() != 0.0)
        {
            for (int i = 0; i < toolFaceVertices.Count(); i++)
            {
                toolFaceVertices.Get(i).TranslationValue(distance * -1.0, 0.0, 0.0);
            }
        }
        else if (toolFaceNormal.Y() != 0.0)
        {
            for (int i = 0; i < toolFaceVertices.Count(); i++)
            {
                toolFaceVertices.Get(i).TranslationValue(0.0, distance * -1.0, 0.0);
            }
        }
        else if (toolFaceNormal.Z() != 0.0)
        {
            for (int i = 0; i < toolFaceVertices.Count(); i++)
            {
                toolFaceVertices.Get(i).TranslationValue(0.0, 0.0, distance * -1.0);
            }
        }
//        for (int i = 0; i < toolFaceVertices.Count(); i++)
//        {
//            toolFaceVertices.Get(i).TranslationValue(distance * -1.0, 0.0, 0.0);
//        }

        step_plane toolPlane = new step_plane();
        toolPlane.SetFromThreePoints(toolFaceVertices.Get(0).GetVertexGeometry().GetCoord(),
                toolFaceVertices.Get(1).GetVertexGeometry().GetCoord(),
                toolFaceVertices.Get(2).GetVertexGeometry().GetCoord());
//        toolPlane.WriteToSurfaceModel("D:\\Graduate\\Paper\\SynMo\\20120927\\toolPlane.stp");

        synchronous_plane_split sps = new synchronous_plane_split(importBody, toolPlane);
        finalBody = sps.PlaneCut();

//        finalBody.Get(0).WriteToSolidModelWithRecountID("D:\\Graduate\\Paper\\SynMo\\20120927\\finalBody01.stp");
//        finalBody.Get(1).WriteToSolidModelWithRecountID("D:\\Graduate\\Paper\\SynMo\\20120927\\finalBody02.stp");
    }
}
