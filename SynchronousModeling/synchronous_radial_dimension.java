package SpringSolid.SynchronousModeling;

import SpringSolid.Boolean.boolean_intersect;
import SpringSolid.Boolean.boolean_merge;
import SpringSolid.CADDesign.*;
import SpringSolid.Mathematics.math_vector3d;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;
import SpringSolid.Part42Link.step_curve_link;
import SpringSolid.SurfaceDesign.prim_lamina;

public class synchronous_radial_dimension implements SpringSolid.Part42.Define
{

    private step_manifold_solid_brep importBody;
    private step_manifold_solid_brep finalBody;
    private step_face_array selectFace;
    private double adjustRadius;
    double originRadius;
    boolean isThough;

    public synchronous_radial_dimension(part_body pBody, step_advanced_face selectFace, double radius)
    {
        this.importBody = pBody.GetFeatureManager().GetResultBodyArray().Get(0);
        this.selectFace = new step_face_array();
        this.selectFace.Add(selectFace);
        this.adjustRadius = radius;
        this.originRadius = ((step_cylindrical_surface) selectFace.GetFaceGeometry()).GetRadius();
    }

    public step_manifold_solid_brep GetSynchronousBody()
    {
        return importBody;
    }

    public void RadialDimensionAdjust()
    {
        step_face_array importBodyFaceArray = new step_face_array();
        this.importBody.GetAdvancedFaces(importBodyFaceArray);
//        selectFace.Get(0).GetActiveFace().WriteToSurfaceModel("D:\\Graduate\\Paper\\SynMo\\20121221\\selectFace.stp");
        for (int i = 0; i < importBodyFaceArray.Count(); i++)
        {
            if (importBodyFaceArray.Get(i).GetActiveFace().GetFaceGeometry().Type() == selectFace.Get(0).GetActiveFace().GetFaceGeometry().Type()
                    && importBodyFaceArray.Get(i).GetActiveFace().IDNumber() != this.selectFace.Get(0).IDNumber())
            {
                if (IsTwoFaceAdjacent(selectFace.Get(0).GetActiveFace(), importBodyFaceArray.Get(i).GetActiveFace()))
                {
                    this.selectFace.Add(importBodyFaceArray.Get(i).GetActiveFace());
                }

            }
        }//Collect adjust face

        step_face_array innerBoundFaceArray = new step_face_array();
        for (int i = 0; i < importBodyFaceArray.Count(); i++)
        {
            step_face_bound_array tempFaceBoundArray = new step_face_bound_array();
            importBodyFaceArray.Get(i).GetActiveFace().GetFaceBounds(tempFaceBoundArray);
            if (tempFaceBoundArray.Count() > 1)
            {
                if (IsTwoFaceAdjacent(selectFace.Get(0).GetActiveFace(), importBodyFaceArray.Get(i).GetActiveFace()))
                {
                    innerBoundFaceArray.Add(importBodyFaceArray.Get(i));
                }
            }
        }

        isThough = false;
        if (innerBoundFaceArray.Count() > 1)
        {
            isThough = true;
        }//Judge the feature shape

        if (isThough == false)
        {
            for (int i = 0; i < importBodyFaceArray.Count(); i++)
            {
                step_face_bound_array tempFBArray = new step_face_bound_array();
                importBodyFaceArray.Get(i).GetActiveFace().GetFaceBounds(tempFBArray);
                if (IsTwoFaceAdjacent(selectFace.Get(0).GetActiveFace(), importBodyFaceArray.Get(i).GetActiveFace()) == true
                        && tempFBArray.Count() < 2
                        && importBodyFaceArray.Get(i).IDNumber() != selectFace.Get(0).IDNumber()
                        && importBodyFaceArray.Get(i).IDNumber() != selectFace.Get(1).IDNumber())
                {
                    selectFace.Add(importBodyFaceArray.Get(i).GetActiveFace());
                }
            }//Get the top or bottom face
        }

        this.DimensionAdjust(innerBoundFaceArray);
//        this.BooleanSubtract(innerBoundFaceArray);

    }

    public void DimensionAdjust(step_face_array innerBoundFaceArray)
    {
        step_edge_curve_array tempECArray;
        step_edge_curve_array circleArray;
        step_vertex_point center;
        step_vertex_point_array circleVertice;

        if (isThough == false)
        {
            circleArray = new step_edge_curve_array();
            selectFace.Get(selectFace.Count() - 1).GetActiveFace().GetEdgeCurves(circleArray);

            center = new step_vertex_point(((step_circle) circleArray.Get(0).GetEdgeGeometry()).GetPosition().GetRefpt());
            circleVertice = new step_vertex_point_array();
            circleArray.Get(0).GetVertices(circleVertice);
            for (int i = 0; i < circleVertice.Count(); i++)
            {
                math_vector3d moveDirection = circleVertice.Get(i).GetVertexGeometry().GetCoord().Subtract(center.GetVertexGeometry().GetCoord()).Normalize();
                circleVertice.Get(i).GetVertexGeometry().SetCoord(circleVertice.Get(i).GetVertexGeometry().GetCoord().Add(moveDirection.Multiply(adjustRadius)));
            }//Adjust extraFace circle end vertice

            for (int i = 0; i < circleArray.Count(); i++)
            {
                ((step_circle) circleArray.Get(i).GetEdgeGeometry()).SetRadius(originRadius + adjustRadius);
            }//Adjust extraFace circle radius
        }

        for (int i = 0; i < innerBoundFaceArray.Count(); i++)
        {
            tempECArray = new step_edge_curve_array();
            circleArray = new step_edge_curve_array();
            innerBoundFaceArray.Get(i).GetActiveFace().GetEdgeCurves(tempECArray);
            for (int j = 0; j < tempECArray.Count(); j++)
            {
                if (tempECArray.Get(j).GetEdgeGeometry().IsCircle())
                {
                    if (IsBelongToFace(tempECArray.Get(j), selectFace.Get(0)) || IsBelongToFace(tempECArray.Get(j), selectFace.Get(1)))
                    {
                        circleArray.Add(tempECArray.Get(j));
                    }
                }
            }//Get adjust circle with correct radius from top or bottom face

            center = new step_vertex_point(((step_circle) circleArray.Get(0).GetEdgeGeometry()).GetPosition().GetRefpt());
            circleVertice = new step_vertex_point_array();
            circleArray.Get(0).GetVertices(circleVertice);
            for (int j = 0; j < circleVertice.Count(); j++)
            {
                math_vector3d moveDirection = circleVertice.Get(j).GetVertexGeometry().GetCoord().Subtract(center.GetVertexGeometry().GetCoord()).Normalize();
                circleVertice.Get(j).GetVertexGeometry().SetCoord(circleVertice.Get(j).GetVertexGeometry().GetCoord().Add(moveDirection.Multiply(adjustRadius)));
            }//Adjust circle end vertice

            for (int j = 0; j < circleArray.Count(); j++)
            {
                ((step_circle) circleArray.Get(j).GetEdgeGeometry()).SetRadius(originRadius + adjustRadius);
            }//Adjust circle radius

            for (int j = 0; j < selectFace.Count(); j++)
            {
                if (selectFace.Get(j).GetActiveFace().GetFaceGeometry().IsCylindricalSurface())
                {
                    ((step_cylindrical_surface) selectFace.Get(j).GetActiveFace().GetFaceGeometry()).SetRadius(originRadius + adjustRadius);
                }
            }//Adjust cylinder face radius
        }
    }

    public void BooleanSubtract(step_face_array innerBoundFaceArray)
    {

        step_face_array importBodyFaceArray = new step_face_array();
        this.importBody.GetOuterShell().GetAdvancedFaces(importBodyFaceArray);

        for (int i = 0; i < selectFace.Count(); i++)
        {
            this.importBody.GetOuterShell().RemoveAdvancedFace(selectFace.Get(i).GetActiveFace());
        }

        for (int i = 0; i < innerBoundFaceArray.Count(); i++)
        {
            step_face_bound_array tempFaceBoundArray = new step_face_bound_array();
            innerBoundFaceArray.Get(i).GetActiveFace().GetFaceBounds(tempFaceBoundArray);
            for (int j = 0; j < tempFaceBoundArray.Count(); j++)
            {
                if (IsBelongToFace(tempFaceBoundArray.Get(j), selectFace.Get(0)))
                {
                    innerBoundFaceArray.Get(i).GetActiveFace().RemoveFaceBound(tempFaceBoundArray.Get(j));
                }
            }
        }//Remove selectFace & inner face bound



        step_axis2_placement_3d originCylinderAxis = ((step_cylindrical_surface) selectFace.Get(0).GetActiveFace().GetFaceGeometry()).GetPosition();
        double currentRadius = ((step_cylindrical_surface) selectFace.Get(0).GetActiveFace().GetFaceGeometry()).GetRadius();

        double height = 0.0;
        if (isThough == true)
        {
            height = ((step_plane) innerBoundFaceArray.Get(0).GetActiveFace().GetFaceGeometry()).GetRefpt().Distance(((step_plane) innerBoundFaceArray.Get(1).GetActiveFace().GetFaceGeometry()).GetRefpt());
        }
        else if (isThough == false)
        {
            height = ((step_plane) innerBoundFaceArray.Get(0).GetActiveFace().GetFaceGeometry()).GetRefpt().Distance(((step_plane) selectFace.Get(2).GetActiveFace().GetFaceGeometry()).GetRefpt());
        }

        step_manifold_solid_brep newCylinder = this.BuildNewBooleanCylinder(originCylinderAxis, currentRadius, height);

        importBody.SetBox();
        newCylinder.SetBox();
        boolean_intersect bi = new boolean_intersect(importBody, newCylinder);
        bi.TopoIntersect();
        boolean_merge bm = new boolean_merge();
        bm.Set(importBody, newCylinder);
        bm.SetSplitEdge(bi.GetInOutCurveArray1(), bi.GetInOutCurveArray2());

        int opAsubB = BOOLEAN_SECOND_CUT_FIRST;
        bm.MergeTopo((short) opAsubB);
        bm.GetResultBody().WriteToSolidModel("final.stp");

        finalBody = this.CallBody();
    }

    public boolean IsTwoFaceAdjacent(step_advanced_face selectFace, step_advanced_face otherFace)
    {
        boolean isAdjacent = false;
        step_edge_curve_array tempECArray = new step_edge_curve_array();
        selectFace.HasCommonEdge(otherFace, tempECArray);
        if (tempECArray.Count() != 0)
        {
            isAdjacent = true;
        }
        return isAdjacent;
    }

    public boolean IsBelongToFace(step_edge_curve ec, step_face selectFace)
    {
        boolean isBelongToFace = false;
        step_edge_curve_array tempECArray = new step_edge_curve_array();
        selectFace.GetActiveFace().GetEdgeCurves(tempECArray);
        for (int i = 0; i < tempECArray.Count(); i++)
        {
            if (ec.IDNumber() == tempECArray.Get(i).IDNumber())
            {
                isBelongToFace = true;
            }
        }
        return isBelongToFace;
    }

    public boolean IsBelongToFace(step_face_bound fb, step_face selectFace)
    {
        boolean isBelongToFace = false;
        step_edge_curve_array tempECArray1 = new step_edge_curve_array();
        step_edge_curve_array tempECArray2 = new step_edge_curve_array();
        selectFace.GetActiveFace().GetEdgeCurves(tempECArray1);
        fb.GetEdgeCurves(tempECArray2);

        for (int i = 0; i < tempECArray2.Count(); i++)
        {
            for (int j = 0; j < tempECArray1.Count(); j++)
            {
                if (tempECArray2.Get(i).IDNumber() == tempECArray1.Get(j).IDNumber())
                {
                    isBelongToFace = true;
                }
            }
        }
        return isBelongToFace;
    }

    public extruded_area_solid DoExtrude(step_curve_link clink, step_plane extrudePlane, math_vector3d normal, double upLimit, double bottomLimit)
    {
        prim_lamina pl1 = new prim_lamina(extrudePlane, clink);
        step_face_array fa = new step_face_array();
        pl1.GetAdvancedFaces(fa);
        step_advanced_face adv;
        adv = (step_advanced_face) fa.Get(0);

        extruded_area_solid extrude = new extruded_area_solid();
        extrude.Extrusion(adv, normal, upLimit, bottomLimit);

        return extrude;
    }

    private step_manifold_solid_brep BuildNewBooleanCylinder(step_axis2_placement_3d originCylinderAxis, double currentRadius, double height)
    {
        step_curve_link clink = new step_curve_link();
        double newCylinderRadius = currentRadius + adjustRadius;
        step_circle c = new step_circle(originCylinderAxis, newCylinderRadius);
        step_plane extrudePlane = new step_plane(originCylinderAxis);

        clink.Add(c);
        extruded_area_solid extrude = this.DoExtrude(clink, extrudePlane, originCylinderAxis.GetAxis().GetCoord(), (height / 2.0) + 0.1, height / 2.0);
        extrude.GetFeatureManager();
        step_manifold_solid_brep newCylinder = new step_manifold_solid_brep();
        newCylinder = (step_manifold_solid_brep) extrude;
        newCylinder.WriteToSolidModelWithRecountID("D:\\Graduate\\Paper\\SynMo\\20121221\\cylinder.stp");
        return newCylinder;
    }

    public step_manifold_solid_brep CallBody()
    {
        step_read read1 = new step_read();
        read1.ReadData("final.stp");
        read1.ConversionData(1.0);
        read1.ReEvaluateData();
        step_manifold_solid_brep body = read1.shapeModel.manifoldSolidBrepArray.Get(0);
        return body;
    }
}
