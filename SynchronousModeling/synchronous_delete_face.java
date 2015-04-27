package SpringSolid.SynchronousModeling;

import SpringSolid.CADDesign.*;
import SpringSolid.Mathematics.*;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;
import SpringSolid.Part42Link.*;
import java.math.*;

public class synchronous_delete_face implements SpringSolid.Part42.Define
{

    private step_manifold_solid_brep importBody;
    private step_face_array selectFace;
    private step_face_array finalBodyFaceArray;
    private step_manifold_solid_brep finalBody;

    public synchronous_delete_face(part_body pBody, step_advanced_face selectFace)
    {
        this.importBody = pBody.GetFeatureManager().GetResultBodyArray().Get(0);
        this.selectFace = new step_face_array();
        this.selectFace.Add(selectFace);
//        feature_manager feature = this.importBody.GetFeatureManager();


    }

    public step_manifold_solid_brep GetSynchronousBody()
    {
        return importBody;
    }

    public void FaceSeperatation(step_face_array faceWithInnerBound, step_face_array faceWithoutInnerBound)
    {
        step_face_array importBodyFaceArray = new step_face_array();
        this.importBody.GetAdvancedFaces(importBodyFaceArray);
        for (int i = 0; i < importBodyFaceArray.Count(); i++)
        {
            step_advanced_face tempAdvFace = (step_advanced_face) importBodyFaceArray.Get(i);
            if (tempAdvFace.GetFaceGeometry().Type() == selectFace.Get(0).GetActiveFace().GetFaceGeometry().Type() && tempAdvFace.IDNumber() != this.selectFace.Get(0).IDNumber())
            {
                if (IsTwoFaceAdjacent(selectFace.Get(0).GetActiveFace(), tempAdvFace))
                {
                    this.selectFace.Add(tempAdvFace);
                }

            }
            else if (tempAdvFace.GetFaceGeometry().Type() != selectFace.Get(0).GetActiveFace().GetFaceGeometry().Type() || tempAdvFace.IDNumber() != this.selectFace.Get(0).IDNumber())
            {
                faceWithoutInnerBound.Add(tempAdvFace);
            }
        }
        //Seperate outer face & cylindrical face.

        if (selectFace.Count() <= 1)
        {
        }
        else
        {
            for (int i = 0; i < faceWithoutInnerBound.Count(); i++)
            {
                step_advanced_face tempAdvFace = (step_advanced_face) faceWithoutInnerBound.Get(i);
                step_face_bound_array tempFaceBound = new step_face_bound_array();
                tempAdvFace.GetFaceBounds(tempFaceBound);
                step_edge_curve_array keepComEdge = new step_edge_curve_array();
                step_edge_curve_array deleteComEdge = new step_edge_curve_array();
                selectFace.Get(1).GetActiveFace().HasCommonEdge(tempAdvFace, keepComEdge);
                selectFace.Get(0).GetActiveFace().HasCommonEdge(tempAdvFace, deleteComEdge);
                if (keepComEdge.Count() != 0 || keepComEdge.Count() != 0)
                {
                    if (tempFaceBound.Count() > 1.0)
                    {
                        faceWithInnerBound.Add(tempAdvFace.Copy());
                        faceWithoutInnerBound.Set(i, null);
                    }
                }
            }
            faceWithoutInnerBound.RemoveEmptyData();
            //Seperate top & bottom face.
        }

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

    public int ModelTypeIdentify(step_face_array faceWithInnerBound, step_face_array faceWithoutInnerBound)
    {
        int faceType = 0;
        int featureNumber = 0;
        int featureShape = 0;

//        step_axis2_placement_3d tempAxis = new step_axis2_placement_3d();

        /*  featureShape
         0 = through
         1 = notch & shaft
         2 = edge fillet*/
        if (faceWithInnerBound.Count() == 2)
        {
            featureShape = 0;
        }
        else if (faceWithInnerBound.Count() == 1)
        {
            featureShape = 1;
        }
        else if (faceWithInnerBound.Count() == 0)
        {
            featureShape = 2;
        }

        /*  faceType
         1 = cylinder
         2 = cone
         3 = sphere 
         4 = fillet*/
        if (selectFace.Get(0).GetActiveFace().GetFaceGeometry().IsCylindricalSurface() == true)
        {
            if (selectFace.Count() != 1)
            {
                faceType = 1;
                if (((step_cylindrical_surface) selectFace.Get(0).GetActiveFace().GetFaceGeometry()).GetPosition()
                        == ((step_cylindrical_surface) selectFace.Get(1).GetActiveFace().GetFaceGeometry()).GetPosition())
                {
                    featureNumber = 1; //Single feature.
                }
                else
                {
                    featureNumber = 2; //Double feature.
                }
            }
            else if (selectFace.Count() == 1)
            {
                faceType = 4;
                featureNumber = 1;
            }
        }
        else if (selectFace.Get(0).GetActiveFace().GetFaceGeometry().IsConicalSurface() == true)
        {
            faceType = 2;
            if (selectFace.Count() == 2)
            {
                featureNumber = 1; //Single feature.
            }
            else
            {
                featureNumber = 2; //Double feature.
            }
        }
        else if (selectFace.Get(0).GetActiveFace().GetFaceGeometry().IsCylinderOrSphere() == true)
        {
            faceType = 3;
            if (selectFace.Count() == 2)
            {
                featureNumber = 1; //Single feature.
            }
            else
            {
                featureNumber = 2; //Double feature.
            }
        }

        int type = faceType * 100 + featureNumber * 10 + featureShape;

        return type;
    }

    public void DeleteFace()
    {
        step_face_array faceWithInnerBound = new step_face_array();
        step_face_array faceWithoutInnerBound = new step_face_array();
        step_face_array newCylindericalSurface = new step_face_array();

        this.FaceSeperatation(faceWithInnerBound, faceWithoutInnerBound); //Sepreate faces of import body.
        int type = this.ModelTypeIdentify(faceWithInnerBound, faceWithoutInnerBound);
        switch (type)
        {
            case 110:
                this.DeleteThroughHole();
                break;
            case 120:
                this.DeleteSpecialThroughHole(faceWithInnerBound, faceWithoutInnerBound, newCylindericalSurface);
                break;
            case 111:
                this.DeleteNotchAndShaft(faceWithInnerBound, faceWithoutInnerBound);
                break;
            case 211:
                this.DeleteNotchAndShaft(faceWithInnerBound, faceWithoutInnerBound);
                break;
            case 412:
                this.DeleteFillet();
                break;
        }
    }

    public void DeleteThroughHole()
    {
        for (int i = 0; i < selectFace.Count(); i++)
        {
            this.importBody.GetOuterShell().RemoveAdvancedFace(selectFace.Get(i).GetActiveFace());
        }

        step_face_array importBodyFaceArray = new step_face_array();
        this.importBody.GetOuterShell().GetAdvancedFaces(importBodyFaceArray);
        for (int i = 0; i < importBodyFaceArray.Count(); i++)
        {
            step_face_bound_array tempFaceBoundArray = new step_face_bound_array();
            importBodyFaceArray.Get(i).GetActiveFace().GetFaceBounds(tempFaceBoundArray);
            if (tempFaceBoundArray.Count() > 1)
            {
                if (IsTwoFaceAdjacent(selectFace.Get(0).GetActiveFace(), importBodyFaceArray.Get(i).GetActiveFace()))
                {
                    for (int j = 0; j < tempFaceBoundArray.Count(); j++)
                    {
                        if (IsBelongToFace(tempFaceBoundArray.Get(j), selectFace.Get(0)) || IsBelongToFace(tempFaceBoundArray.Get(j), selectFace.Get(1)))
                        {
                            importBodyFaceArray.Get(i).GetBounds().Remove(tempFaceBoundArray.Get(j));
                        }
                    }
                }

            }
        }
    }

    public void DeleteSpecialThroughHole(step_face_array faceWithInnerBound, step_face_array faceWithoutInnerBound, step_face_array newCylindericalSurface)
    {
        this.RebuildCylindericalSurface(selectFace.Get(1).GetActiveFace(), newCylindericalSurface);
        finalBodyFaceArray = new step_face_array();
        finalBodyFaceArray = faceWithoutInnerBound;
        this.CreateNewBody(faceWithInnerBound, newCylindericalSurface);
    }

    public void DeleteNotchAndShaft(step_face_array faceWithInnerBound, step_face_array faceWithoutInnerBound)
    {
        step_face_array tempFaceArray;
        step_face_array adjacentFaceArray = new step_face_array();
        step_edge_curve_array tempEdgeCurveArray = new step_edge_curve_array();
        selectFace.Get(0).GetActiveFace().GetEdgeCurves(tempEdgeCurveArray);
        for (int i = 0; i < tempEdgeCurveArray.Count(); i++)
        {
            tempFaceArray = new step_face_array();
            importBody.GetAdjacentFaces(tempEdgeCurveArray.Get(i), tempFaceArray);
            for (int j = 0; j < tempFaceArray.Count(); j++)
            {
                adjacentFaceArray.Add(tempFaceArray.Get(j));
                if (adjacentFaceArray.CheckExisted(selectFace.Get(0)))
                {
                    adjacentFaceArray.Remove(selectFace.Get(0).GetActiveFace());
                }
                if (adjacentFaceArray.CheckExisted(selectFace.Get(1)))
                {
                    adjacentFaceArray.Remove(selectFace.Get(1).GetActiveFace());
                }
            }
        }
        for (int i = 0; i < faceWithInnerBound.Count(); i++)
        {
            for (int j = 0; j < adjacentFaceArray.Count(); j++)
            {
                step_face tempFace = adjacentFaceArray.Get(j);
                if (faceWithInnerBound.Get(i).IDNumber() == tempFace.IDNumber())
                {
                    adjacentFaceArray.Remove(tempFace.GetActiveFace());
                }
            }
        }
        if (adjacentFaceArray.Count() == 1)
        {
            selectFace.Add(adjacentFaceArray.Get(0));
        }
        this.DeleteThroughHole();
    }

    public void DeleteFillet()
    {
        step_edge_curve_array selectFaceECArray = new step_edge_curve_array();
        step_edge_curve_array selectFaceLine = new step_edge_curve_array();
        step_edge_curve_array selectFaceArc = new step_edge_curve_array();
        selectFace.Get(0).GetActiveFace().GetEdgeCurves(selectFaceECArray);
        for (int i = 0; i < selectFaceECArray.Count(); i++)
        {
            if (selectFaceECArray.Get(i).GetEdgeGeometry().IsLine())
            {
                selectFaceLine.Add(selectFaceECArray.Get(i));
            }
            else if (selectFaceECArray.Get(i).GetEdgeGeometry().IsCircle())
            {
                selectFaceArc.Add(selectFaceECArray.Get(i));
            }
        }

        step_vertex_point_array expoGroup = new step_vertex_point_array();
        double distance = 0.0;

        expoGroup.Add(selectFaceLine.Get(0).GetEdgeStart());
        distance = expoGroup.Get(0).Distance(selectFaceLine.Get(1).GetEdgeStart());
        if (expoGroup.Get(0).Distance(selectFaceLine.Get(1).GetEdgeEnd()) < distance)
        {
            expoGroup.Add(selectFaceLine.Get(1).GetEdgeEnd());
            expoGroup.Add(selectFaceLine.Get(0).GetEdgeEnd());
            expoGroup.Add(selectFaceLine.Get(1).GetEdgeStart());
        }
        else
        {
            expoGroup.Add(selectFaceLine.Get(1).GetEdgeStart());
            expoGroup.Add(selectFaceLine.Get(0).GetEdgeEnd());
            expoGroup.Add(selectFaceLine.Get(1).GetEdgeEnd());
        }

        step_vertex_point_array newEdgeVertex = new step_vertex_point_array();
        newEdgeVertex.Add(FindExtrapolationPoint(expoGroup.Get(0), expoGroup.Get(1)));
        newEdgeVertex.Add(FindExtrapolationPoint(expoGroup.Get(2), expoGroup.Get(3)));

        step_vertex_point_array tempVPArray = new step_vertex_point_array();
        importBody.GetVertices(tempVPArray);

        for (int i = 0; i < tempVPArray.Count(); i++)
        {
            if (tempVPArray.Get(i).IDNumber() == expoGroup.Get(0).IDNumber())
            {
                tempVPArray.Get(i).SetVertexGeometry(newEdgeVertex.Get(0).GetVertexGeometry());
            }
            if (tempVPArray.Get(i).IDNumber() == expoGroup.Get(1).IDNumber())
            {
                tempVPArray.Get(i).SetVertexGeometry(newEdgeVertex.Get(0).GetVertexGeometry());
            }
            if (tempVPArray.Get(i).IDNumber() == expoGroup.Get(2).IDNumber())
            {
                tempVPArray.Get(i).SetVertexGeometry(newEdgeVertex.Get(1).GetVertexGeometry());
            }
            if (tempVPArray.Get(i).IDNumber() == expoGroup.Get(3).IDNumber())
            {
                tempVPArray.Get(i).SetVertexGeometry(newEdgeVertex.Get(1).GetVertexGeometry());
            }
        }
        this.importBody.GetOuterShell().RemoveAdvancedFace(selectFace.Get(0).GetActiveFace());
        step_face_array tempFaceArray = new step_face_array();
        for (int i = 0; i < selectFaceArc.Count(); i++)
        {
            this.importBody.GetAdjacentFacesExclusiveSpefiedFaces(selectFaceArc.Get(i), selectFace.Get(0).GetActiveFace(), tempFaceArray);
            tempFaceArray.Get(i).GetActiveFace().RemoveEdgeCurve(selectFaceArc.Get(i));
        }
        step_face_array finalFaceArray = new step_face_array();
        this.importBody.GetOuterShell().GetAdvancedFaces(finalFaceArray);
        step_closed_shell finalBodyShell = new step_closed_shell();
        for (int i = 0; i < finalFaceArray.Count(); i++)
        {
            finalBodyShell.AddAdvancedFace(finalFaceArray.Get(i).GetActiveFace());
        }
        importBody.SetOuterShell(finalBodyShell);
        int stop = 0;
    }

    public step_vertex_point FindExtrapolationPoint(step_vertex_point vp1, step_vertex_point vp2)
    {
        step_vertex_point expoPoint = new step_vertex_point();
        step_edge_curve_array tempECArray = new step_edge_curve_array();
        step_edge_curve filletCurve = new step_edge_curve();
        double filletRadius = 0.0;
        double tangentPointDistant = (vp1.Distance(vp2)) / 2.0;
        vp1.GetEdgeCurves(importBody, tempECArray);
        for (int i = 0; i < tempECArray.Count(); i++)
        {
            if (tempECArray.Get(i).GetEdgeGeometry().IsCircle())
            {
                filletCurve = tempECArray.Get(i);
                filletRadius = ((step_circle) filletCurve.GetEdgeGeometry()).GetRadius();
            }
        }
        double expoLength = (tangentPointDistant * filletRadius) / (Math.sqrt((filletRadius * filletRadius) - (tangentPointDistant * tangentPointDistant)));
        math_vector3d expoDirection = new math_vector3d();
        filletCurve.Tangent(vp1.GetVertexGeometry().GetCoord(), expoDirection);
        expoDirection = expoDirection.Normalize();
        step_cartesian_point tempCP = new step_cartesian_point(vp1.GetVertexGeometry().GetX() + expoLength * expoDirection.X(),
                vp1.GetVertexGeometry().GetY() + expoLength * expoDirection.Y(),
                vp1.GetVertexGeometry().GetZ() + expoLength * expoDirection.Z());
        for (int i = 0; i < tempECArray.Count(); i++)
        {
            if (tempECArray.Get(i).On(tempCP.GetCoord()) == 1)
            {
                expoDirection = expoDirection.Minus();
                tempCP = new step_cartesian_point(vp1.GetVertexGeometry().GetX() + expoLength * expoDirection.X(),
                        vp1.GetVertexGeometry().GetY() + expoLength * expoDirection.Y(),
                        vp1.GetVertexGeometry().GetZ() + expoLength * expoDirection.Z());
                i = i + tempECArray.Count() + 1;
            }
        }
        expoPoint.SetVertexGeometry(tempCP);
        int stop = 0;
        return expoPoint;

    }

    public void RebuildCylindericalSurface(step_advanced_face inputAdvFace, step_face_array newCylindericalSurface)
    {
        step_cylindrical_surface newCylindericalFace01;
        step_cylindrical_surface newCylindericalFace02;
        step_cylindrical_surface oriCylindericalFace = (step_cylindrical_surface) inputAdvFace.GetFaceGeometry();
        step_axis2_placement_3d oriCylindericalFaceAxis = oriCylindericalFace.GetPosition();
        double oriCylindericalFaceRadius = oriCylindericalFace.GetRadius();

        step_advanced_face returnFace01 = new step_advanced_face();
        newCylindericalFace01 = new step_cylindrical_surface(oriCylindericalFaceAxis, oriCylindericalFaceRadius);
        newCylindericalFace01.SetUbound(0, 0);
        newCylindericalFace01.SetUbound(1, PI);
        newCylindericalFace01.SetVbound(0, -25.0);
        newCylindericalFace01.SetVbound(1, 25.0);
        returnFace01.SetFromSurface(newCylindericalFace01);

        step_advanced_face returnFace02 = new step_advanced_face();
        newCylindericalFace02 = new step_cylindrical_surface(oriCylindericalFaceAxis, oriCylindericalFaceRadius);
        newCylindericalFace02.SetUbound(0, PI);
        newCylindericalFace02.SetUbound(1, 2.0 * PI);
        newCylindericalFace02.SetVbound(0, -25.0);
        newCylindericalFace02.SetVbound(1, 25.0);
        returnFace02.SetFromSurface(newCylindericalFace02);

        newCylindericalSurface.Add(returnFace01);
        newCylindericalSurface.Add(returnFace02);
    }

    public void CreateNewBody(step_face_array faceWithInnerBound, step_face_array newCylindricalSurface)
    {
        finalBodyFaceArray.Add(newCylindricalSurface.Get(0));
        finalBodyFaceArray.Add(newCylindricalSurface.Get(1));
        for (int i = 0; i < faceWithInnerBound.Count(); i++)
        {
            step_advanced_face tempAdvFace = (step_advanced_face) faceWithInnerBound.Get(i); //Get original top or bottom face.

            step_face_bound_array tempBoundArray = new step_face_bound_array();
            tempAdvFace.GetFaceBounds(tempBoundArray);
            step_face_bound tempFaceBonud = tempBoundArray.Get(0); //Get outer bound.

            step_oriented_edge_array tempOriEdgeArray = new step_oriented_edge_array();
            tempFaceBonud.GetOrientedEdges(tempOriEdgeArray); //Get oriented edges of outer bound.

            step_edge_curve_array newFaceEdgeCurveArray = new step_edge_curve_array();

            for (int j = 0; j < tempOriEdgeArray.Count(); j++)
            {
                if (tempOriEdgeArray.Get(j).GetOrientation() == true)
                {
                    newFaceEdgeCurveArray.Add(tempOriEdgeArray.Get(j).GetEdgeElement());
                }
                else if (tempOriEdgeArray.Get(j).GetOrientation() == false)
                {
                    newFaceEdgeCurveArray.Add(tempOriEdgeArray.Get(j).GetEdgeElement().Reverse());
                }
            }

            for (int j = 0; j < newCylindricalSurface.Count(); j++)
            {
                step_advanced_face tempCylindricalFace = newCylindricalSurface.Get(j).GetActiveFace(); //Get one cylindircal face.

                step_edge_curve_array cylindricalEdgeCurveArray = new step_edge_curve_array();
                tempCylindricalFace.GetEdgeCurves(cylindricalEdgeCurveArray); //Get 4 Edges of the cylindrical face.

                for (int k = 0; k < cylindricalEdgeCurveArray.Count(); k++)
                {
                    step_edge_curve tempEdgeCurve = new step_edge_curve();
                    tempEdgeCurve = cylindricalEdgeCurveArray.Get(k);

                    if (tempAdvFace.On(tempEdgeCurve.GetEdgeEnd().GetVertexGeometry().GetCoord()) != 0
                            && tempAdvFace.On(tempEdgeCurve.GetEdgeStart().GetVertexGeometry().GetCoord()) != 0)
                    {
                        newFaceEdgeCurveArray.Add(tempEdgeCurve);
                    }
                }
            }

            step_advanced_face newAdvFace = new step_advanced_face();

            boolean_array newFaceOuterBoolean = new boolean_array();
            step_curve_link newFaceOuterCurveLink = new step_curve_link();

            boolean_array newFaceInnerBoolean = new boolean_array();
            boolean_array[] newFaceInnerBooleanArray = new boolean_array[1];
            step_curve_link newFaceInnerCurveLink = new step_curve_link();
            step_curve_link[] newFaceInnerCurveLinkArray = new step_curve_link[1];

            for (int j = 0; j < newFaceEdgeCurveArray.Count() - 2; j++)
            {
                newFaceOuterCurveLink.Add(newFaceEdgeCurveArray.Get(j).GetEdgeGeometry());
                newFaceOuterBoolean.Add(true);
                System.out.println("Add " + newFaceEdgeCurveArray.Get(j).IDNumber());
            }

            for (int j = newFaceEdgeCurveArray.Count() - 2; j < newFaceEdgeCurveArray.Count(); j++)
            {
                newFaceInnerCurveLink.Add(newFaceEdgeCurveArray.Get(j).GetEdgeGeometry());
                newFaceInnerBoolean.Add(true);
                System.out.println("Add " + newFaceEdgeCurveArray.Get(j).IDNumber());
            }
            newFaceInnerBooleanArray[0] = newFaceInnerBoolean;
            newFaceInnerCurveLinkArray[0] = newFaceInnerCurveLink;

            newAdvFace.SetFromSurfaceAndProfiles(tempAdvFace.GetFaceGeometry(), newFaceOuterCurveLink, newFaceOuterBoolean, newFaceInnerCurveLinkArray, newFaceInnerBooleanArray);
            finalBodyFaceArray.Add(newAdvFace);
        }
//        finalBodyFaceArray.WriteToSurfaceModel("D:\\Graduate\\Paper\\SynMo\\20120527\\out.stp");


        step_closed_shell finalCShell = new step_closed_shell();
        for (int i = 0; i < finalBodyFaceArray.Count(); i++)
        {
            finalCShell.LinkAdvancedFace((step_advanced_face) finalBodyFaceArray.Get(i));
        }
        importBody.SetOuterShell(finalCShell);
    }
}
