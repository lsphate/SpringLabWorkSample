package SpringSolid.SynchronousModeling;

import SpringSolid.CADDesign.oriented_edge_traversal_array;
import SpringSolid.CADDesign.sewing;
import SpringSolid.Mathematics.*;
import SpringSolid.Part42.*;
import SpringSolid.Part42Array.*;
import SpringSolid.Part42Link.*;
import SpringSolid.SurfaceDesign.plane_split;
import SpringSolid.TrimMesh.trim_loop;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.Serializable;

public class synchronous_plane_split extends plane_split implements SpringSolid.Part42.Define, Serializable
{

    private step_manifold_solid_brep body;
    private step_plane cutPlane;
    private step_advanced_face cutFace;
    private boolean orientation;
    private double stockTolerance;
    private short output;
    private FileOutputStream fileout;

    public step_advanced_face GetCutFace()
    {
        return this.cutFace;
    }

    public synchronous_plane_split(step_manifold_solid_brep importBody, step_plane toolPlane)
    {
        this.body = importBody;
        this.cutPlane = toolPlane;
        this.output = 0;

        step_advanced_face cutface = this.SetCuttingPlaneFace(this.cutPlane);
        this.cutFace = cutface;

        if (this.output == 1)
        {
            File file1 = new File("PlaneCutResult.txt");
            try
            {
                FileOutputStream fout = new FileOutputStream(file1);
                this.fileout = fout;
            }
            catch (IOException ex)
            {
            }
        }
    }  /*
     * end of procedure body_plane_split::step_plane_cut
     */


    public step_manifold_solid_brep_array PlaneCut()
    {
        step_face_array[] finalBodyFacesArray = new step_face_array[2];

        step_manifold_solid_brep oriBody = this.body;

        oriBody.ReEvaluateData();
        oriBody.CheckEdgeCurveOrientation();
        oriBody.CalculateTrimEdgeParamOfOrientedEdge();
        oriBody.SetFaceParam();
        oriBody.NormalizeTrimEdgeParamOfOrientedEdge();
        oriBody.SetBox();
        oriBody.CheckCorrectness();

        this.cutPlane.SetPlaneBox();
        this.cutPlane.SetParamBound();

        this.body.GetAdvancedFaces(this.faceArray);
        int facecount = this.faceArray.Count();
        step_advanced_face fpnt;
        boolean report;
        for (int i = 0; i < facecount; ++i)
        {
            fpnt = (step_advanced_face) this.faceArray.Get(i);
            report = fpnt.CheckFaceBox(this.cuttingPlane);
            if (report == true) // && fpnt.IDNumber() == 6535 )
            {
                this.activeFaceArray.Add(fpnt);
            }
        }

        step_edge_curve_array ecArray = new step_edge_curve_array();
        this.activeFaceArray.GetEdgeCurves(ecArray);

        int count = ecArray.Count();
        step_edge_curve ec;
        for (int i = 0; i < count; ++i)
        {
            ec = ecArray.Get(i);
            long id = ec.IDNumber();
            report = ec.CheckFaceBox(this.cuttingPlane);
            if (report == true)
            {
                this.activeEdgeArray.Add(ec);
            }
        }

        step_vertex_point_array vpArray = new step_vertex_point_array();
        this.activeFaceArray.GetVertices(vpArray);

        int vpcount = vpArray.Count();
        step_vertex_point vp;
        for (int i = 0; i < vpcount; ++i)
        {
            vp = vpArray.Get(i);
            long id = vp.IDNumber();
            report = vp.CheckFaceBox(this.cuttingPlane);
            if (report == true)
            {
                this.activeVertexArray.Add(vp);
            }
        }

        step_shell_based_surface_model splitGenerateBody = this.FindPlaneIntersectionAndProfile(this.cutFace);
        step_face_array splitFaces = new step_face_array();
        splitGenerateBody.GetAdvancedFaces(splitFaces);

        this.FaceClassification(finalBodyFacesArray, cutPlane);

        step_manifold_solid_brep_array finalBodyArray = this.AssembleFaces(finalBodyFacesArray, splitFaces);

        return finalBodyArray;
    }  /*
     * end of procedure body_plane_split::PlaneCut
     */


    public step_manifold_solid_brep_array AssembleFaces(step_face_array[] finalBodyFacesArray, step_face_array splitFaces)
    {
        step_manifold_solid_brep_array finalBodyArray = new step_manifold_solid_brep_array();
        finalBodyFacesArray[0].Add((step_advanced_face) splitFaces.Get(0));
        step_manifold_solid_brep frontBody = new step_manifold_solid_brep();
        sewing frontSew = new sewing();
        frontSew.AddToSurfaceModel(finalBodyFacesArray[0]);
        boolean isFront = frontSew.CheckManifoldEntity();
        if (isFront)
        {
            frontSew.BuildSewingBody();
            frontBody = frontSew.GetBody();
            frontBody.SetFeatureManager(body.GetFeatureManager());
            frontBody.ResetFlagFromSequence();
            frontBody.GetFeatureManager().ResetIDNumberFromSequence(54);
            frontBody.ResetFlagFromSequence();
        }
        finalBodyArray.Add(frontBody);

        finalBodyFacesArray[1].Add((step_advanced_face) splitFaces.Get(1));
        step_manifold_solid_brep rearBody = new step_manifold_solid_brep();
        sewing rearSew = new sewing();
        rearSew.AddToSurfaceModel(finalBodyFacesArray[1]);
        boolean isRear = rearSew.CheckManifoldEntity();
        if (isRear)
        {
            rearSew.BuildSewingBody();
            rearBody = rearSew.GetBody();
            rearBody.SetFeatureManager(body.GetFeatureManager());
            rearBody.ResetFlagFromSequence();
            rearBody.GetFeatureManager().ResetIDNumberFromSequence(300);
            rearBody.ResetFlagFromSequence();
        }
        finalBodyArray.Add(rearBody);

        return finalBodyArray;
    }

    public void FaceClassification(step_face_array[] finalBodyFaceArray, step_plane toolPlane)
    {
        step_face_link tempFaceLink = body.GetOuterShell().GetCfsFaces();
        step_face_iterator tempIt = new step_face_iterator(tempFaceLink);
        double dotProduct;

        step_advanced_face tempAdvFace;
        math_vector3d refPoint = toolPlane.RefptCoord();
        math_vector3d normal = toolPlane.AxisVector();
        math_vector3d tempVertex = new math_vector3d();

        step_face_array frontFaceArray = new step_face_array();
        step_face_array rearFaceArray = new step_face_array();

        while (tempIt.HasNext())
        {
            tempAdvFace = (step_advanced_face) tempIt.GetNext();
            tempAdvFace.GetCandidateVertex(tempVertex);

            dotProduct = (tempVertex.Subtract(refPoint)).DotProduct(normal);

            if (dotProduct > 0.0)
            {
                frontFaceArray.Add(tempAdvFace);
            }
            else if (dotProduct < 0.0)
            {
                rearFaceArray.Add(tempAdvFace);
            }
        }
        finalBodyFaceArray[0] = frontFaceArray;
        finalBodyFaceArray[1] = rearFaceArray;
        finalBodyFaceArray[0].WriteToSurfaceModel("D:\\Graduate\\Paper\\SynMo\\20120927\\frontbox.stp");
        finalBodyFaceArray[1].WriteToSurfaceModel("D:\\Graduate\\Paper\\SynMo\\20120927\\rearbox.stp");
        int stop = 0;
    }

    public step_shell_based_surface_model FindPlaneIntersectionAndProfile(step_advanced_face cutface)
    {
        math_vector3d normal = this.cutPlane.AxisVector();
        step_manifold_solid_brep body1 = this.body;
        step_shell_based_surface_model body2 = this.planeCutBody;

        this.VertexPlaneIntersect(body2, cutface);

        math_intercurve_link intercurves = new math_intercurve_link();
        this.EdgePlaneIntersect(body2, intercurves);

        int count = intercurves.Count();

        intercurves = new math_intercurve_link();
        short report = this.FacePlaneIntersect(body2, intercurves);

        this.output = 1;

        if (this.output == 1)
        {
            body1.ResetFlagFromSequence();
            long maxid = 54;
            body1.ResetIDNumberFromSequence(maxid);
            body1.ResetFlagFromSequence();
            body1.WriteToSolidModel("split01.stp");

            body2.ResetFlagFromSequence();
            maxid = 54;
            body2.ResetIDNumberFromSequence(maxid);
            body2.ResetFlagFromSequence();
            body2.WriteToSurfaceModel("split02.stp");
        }

        // this.FindCuttingProfiles(body2, this.cutPlane, intercurves);

        this.CheckFaceOrientation(body2, normal);

        if (this.output == 1)
        {
            // body1.ResetFlagFromSequence();
            // body1.WriteToSolidModel("split07.stp");
            // body2.ResetFlagFromSequence();
            // body2.ResetIDNumberFromSequence(54);
            // body2.ResetFlagFromSequence();
            // body2.WriteToSurfaceModel("PlaneSplitWithoutSetBox.stp");
        }
        return body2;
    }  /*
     * end of procedure body_plane_split::PlaneSplit
     */


    private void EdgePlaneIntersect(step_shell_based_surface_model body2,
            math_intercurve_link intercurves)
    {
        step_edge_curve_array ecArray = this.activeEdgeArray;
        step_advanced_face fpnt2 = this.cuttingPlane;
        math_vector3d_array coorArray = new math_vector3d_array();
        math_interpoint_link interpoints = new math_interpoint_link();
        int count = ecArray.Count();
        step_edge_curve epnt;
        for (int i = 0; i < count; i++)
        {
            epnt = ecArray.Get(i);
            if (epnt.IDNumber() == 242)
            {
                int y = 1;
            }
            this.EFIntersect(epnt, fpnt2, body2, interpoints, intercurves);

            // Note: all newly split edges (except for epnt) are
            // assigned with type=flag=ENTITY_SPLIT

            this.SplitEdgeByFaces(epnt, fpnt2, body2, coorArray, interpoints);
            interpoints.Empty();
        }
    }  /*
     * end of procedure body_plane_split::EdgePlaneIntersect
     */


    private void SplitEdgeByFaces(step_edge_curve epnt1,
            step_advanced_face fpnt2,
            step_shell_based_surface_model body2,
            math_vector3d_array coorArray,
            math_interpoint_link totalpoints)
    {
        step_vertex_point newvpnt1[] = new step_vertex_point[1];
        step_edge_curve newepnt[] = new step_edge_curve[1];
        step_manifold_solid_brep body;
        math_vector3d chkpoint;
        boolean answer = false;
        while (!totalpoints.IsEmpty())
        {
            math_interpoint interpt = totalpoints.GetLast();
            chkpoint = interpt.GetIntersect();

            this.body.MVE(epnt1, chkpoint, newvpnt1, newepnt);
            step_advanced_face fpnt;

            if (newepnt[0] != null)
            {
                newepnt[0].SetEntityFlag(ENTITY_SPLIT);
            }

            if (newvpnt1[0] != null)
            {
                newvpnt1[0].SetEntityFlag(ENTITY_SPLIT);
            }

            if (interpt.GetFace() != null)
            {
                fpnt = interpt.GetFace();
            }

            step_vertex_point newvpnt2[] = new step_vertex_point[1];
            if (interpt.GetFace() != null)
            {
                answer = coorArray.CheckExisted(chkpoint);
                if (answer == false)
                {
                    coorArray.Add(chkpoint);

                    fpnt = interpt.GetFace();
                    step_face_bound newlpnt[] = new step_face_bound[1];
                    body2.MVL(fpnt, chkpoint, newvpnt2, newlpnt);
                }
                else
                {
                    newvpnt2[0] = body2.FindVertexPoint(chkpoint);
                }
            }
            else
            {
                answer = coorArray.CheckExisted(chkpoint);
                if (answer == false)
                {
                    coorArray.Add(chkpoint);
                    step_face_bound newlpnt[] = new step_face_bound[1];
                    body2.MVL(fpnt2, chkpoint, newvpnt2, newlpnt);
                }
                else
                {
                    newvpnt2[0] = body2.FindVertexPoint(chkpoint);
                }
            }

            if (newvpnt2[0] != null)
            {
                newvpnt2[0].SetEntityFlag(ENTITY_SPLIT);
            }

            if (newvpnt1[0] != null)
            {
                newvpnt1[0].AssignSameVertex(newvpnt2[0]);
            }

            totalpoints.Remove(interpt);
        }
    }  /*
     * end of procedure body_plane_split::SplitEdgeByFaces
     */


    private short FacePlaneIntersect(step_shell_based_surface_model body2,
            math_intercurve_link intercurves)
    {
        step_advanced_face face1;
        step_face_array faceArray = this.activeFaceArray;
        int fnum = faceArray.Count();
        step_advanced_face face2 = this.cuttingPlane;
        for (int i = 0; i < fnum; i++)
        {
            face1 = (step_advanced_face) faceArray.Get(i);

            this.FindFacePlaneIntersect(face1, face2, this.GetIntersectCurve());

            this.PlaneCutSplit(face1, face2, intercurves);
        }

        step_vertex_point pvert = null;
        step_vertex_point nvert = null;
        int count = 0;
        step_advanced_face fpnt1;
        step_advanced_face fpnt2;
        step_advanced_face newfpnt1[] = new step_advanced_face[1];
        step_advanced_face newfpnt2[] = new step_advanced_face[1];
        step_edge_curve newepnt1[] = new step_edge_curve[1];
        step_edge_curve newepnt2[] = new step_edge_curve[1];

        while (!intercurves.IsEmpty())
        {
            ++count;

            math_intercurve crosscurve = intercurves.GetFirst();
            step_curve cpnt1 = crosscurve.GetCurve();
            step_curve cpnt2 = cpnt1.CurveCopy();
            cpnt1.SetName(-1);
            cpnt2.SetName(-1);

            fpnt1 = crosscurve.FindFirstFace();
            fpnt2 = crosscurve.FindSecondFace();

            newfpnt1[0] = null;
            newfpnt2[0] = null;
            newepnt1[0] = null;
            newepnt2[0] = null;

            if (fpnt1 != null)
            {
                pvert = crosscurve.GetPvert1();
                nvert = crosscurve.GetNvert1();

                this.FirstFaceSplit(fpnt1, cpnt1, pvert, nvert, newfpnt1, newepnt1);
                fpnt1.SetIntersectType(true);
            }

            if (fpnt2 != null)
            {
                pvert = crosscurve.GetPvert2();
                nvert = crosscurve.GetNvert2();

                this.SecondFaceSplit(fpnt2, cpnt2, pvert, nvert, newfpnt2, newepnt2);
                fpnt2.SetIntersectType(true);

                this.output = 0;
                if (this.output == 1)
                {
                    String filename1 = "A";
                    filename1 = filename1.concat(String.valueOf(count));
                    filename1 = filename1.concat(".stp");
                    if (fpnt1 != null)
                    {
                        fpnt1.GetBody().ResetFlagFromSequence();;
                        fpnt1.GetBody().WriteToSolidModel(filename1);
                    }

                    String filename2 = "B";
                    filename2 = filename2.concat(String.valueOf(count));
                    String filename3 = filename2.concat(".stp");
                    if (fpnt2 != null)
                    {
                        fpnt2.GetBody().WriteToSolidModel(filename3);
                    }
                }
            }

            if (newepnt1[0] != null && newepnt2[0] != null)
            {
                this.CreateCrossEdge(newepnt1, newepnt2, crosscurve);
            }

            intercurves.Remove(crosscurve);
        }

        return 1;
    }  /*
     * end of procedure step_body_plane_split::FaceFaceIntersect
     */


    private void SplitClassification(step_manifold_solid_brep body1, step_plane supnt,
            boolean sense, step_face_array[] finalBodyFaceArray)
    {
        body1.ResetFaceFlag();

        step_face_array frontFaceArray = new step_face_array();
        step_face_array rearFaceArray = new step_face_array();

        math_vector3d refpt = supnt.RefptCoord();
        step_closed_shell sh = body1.GetOuterShell();
        step_face_link link = sh.GetCfsFaces();
        double center[] = new double[2];
        math_vector3d coord = new math_vector3d();
        step_face temp;
        step_advanced_face face1;
        double s1;
        math_vector3d normal = new math_vector3d();
        math_vector3d coor1 = new math_vector3d();
        step_face_iterator it = new step_face_iterator(link);
        while (it.HasNext())
        {
            temp = it.GetNext();
            face1 = (step_advanced_face) temp;

            face1.FindCentroid(center);

            coord.Set(center[0], center[1], 0.0);
            if (face1.GetTrimFace() == null)
            {
                face1.ToTrimFace();
            }
            coord = face1.GetTrimFace().ChangeToGlobalDomain(coord);
            center[0] = coord.X();
            center[1] = coord.Y();

            face1.GetFaceGeometry().Coordinate(center[0], center[1], coor1);

            supnt.NearestProjectPoint(coor1, center);
            supnt.GetNormalWithNormalize(center[0], center[1], normal);
            supnt.Coordinate(center[0], center[1], refpt);

            s1 = (coor1.Subtract(refpt)).DotProduct(normal);
            if (sense == true)
            {
                if (s1 > 0.0)
                {
                    face1.SetEntityFlag(ENTITY_KEEP);
                    frontFaceArray.Add(face1);
                }
                else
                {
                    face1.SetEntityFlag(ENTITY_DELETED);
                    rearFaceArray.Add(face1);
                }
            }
            else
            {
                if (s1 > 0.0)
                {
                    face1.SetEntityFlag(ENTITY_KEEP);
                }
                else
                {
                    face1.SetEntityFlag(ENTITY_DELETED);
                }
            }
            finalBodyFaceArray[0] = frontFaceArray;
            finalBodyFaceArray[1] = rearFaceArray;
        }

//        body1.RemoveAdvancedFace();
        body1.CheckCorrectness();
    }  /*
     * end of procedure body_plane_split::SplitClassification
     */


    public void FindCentroid(step_advanced_face adFace, double center[])
    {
        step_face_bound fb = adFace.FindExternalLoop();
        trim_loop tloop = fb.GetTrimLoop();
        if (tloop == null)
        {
            step_edge_curve_array ecArray = new step_edge_curve_array();
            adFace.GetEdgeCurves(ecArray);

            step_edge_curve epnt;
            int eccount = ecArray.Count();
            for (int i = 0; i < eccount; ++i)
            {
                epnt = ecArray.Get(i);
                epnt.SetExtremeFlag(false);
                epnt.CheckEdgeCurveOrientation();
                epnt.SetStartAndEndParam();
                epnt.CreateDiscretePoints();
            }

            adFace.CalculateTrimEdgeParamOfOrientedEdge();
            adFace.SetFaceParam();
            adFace.NormalizeTrimEdgeParamOfOrientedEdge();
        }

        double gauss[] = new double[2];
        double weight[] = new double[2];
        int ngaus = 2;

        Extern.GaussPoint(ngaus, gauss);
        Extern.WeightFactor(ngaus, weight);

        double x1, y1, x2, y2;
        int ii = 0;
        int vnumber;
        double dy;
        double s1, s2;
        double gx, gy;
        double we;
        double area = 0.0;
        double centx = 0.0;
        double centy = 0.0;

        double controlPoint[];
        step_face_bound tempbound;
        int total = 0;
        step_face_bound_link boundlink = adFace.GetBounds();
        step_face_bound_iterator it2 = new step_face_bound_iterator(boundlink);
        while (it2.HasNext())
        {
            tempbound = it2.GetNext();
            controlPoint = tempbound.FindParamInLoop();

            if (controlPoint == null)
            {
                continue;
            }

            vnumber = controlPoint.length / 2;
            for (int ipoint = 0; ipoint < (vnumber - 1); ++ipoint)
            {
                ii = ipoint * 2;
                x1 = controlPoint[ii];
                y1 = controlPoint[ii + 1];
                x2 = controlPoint[ii + 2];
                y2 = controlPoint[ii + 3];

                for (int igaus = 0; igaus < 2; igaus++)
                {
                    s1 = 0.5 * (1.0 - gauss[igaus]);
                    s2 = 0.5 * (1.0 + gauss[igaus]);

                    gx = s1 * x1 + s2 * x2;
                    gy = s1 * y1 + s2 * y2;
                    // dx = x2 - x1;
                    dy = y2 - y1;

                    we = weight[igaus];
                    area += 0.50 * dy * gx * we;
                    centx += 0.25 * dy * gx * gx * we;
                    centy += 0.50 * dy * gx * gy * we;
                }
            }
            ++total;
        }

        centx = centx / area;
        centy = centy / area;
        center[0] = centx;
        center[1] = centy;
    }  /* end of procedure step_advanced_face::FindCentroid */


    private void MergeFaceToBody(step_shell_based_surface_model body2)
    {
        step_edge_curve_array ecArray = new step_edge_curve_array();
        step_face_array faceArray = new step_face_array();
        body2.GetAdvancedFacesAndEdgeCurves(faceArray, ecArray);

        this.HashLaminaTopology(body2, ecArray);

        this.MergeEdge(ecArray);
        this.JoinFace();
        this.JoinEdge();
    }  /*
     * end of procedure body_plane_split::MergeFaceToBody
     */


    private void HashLaminaTopology(step_shell_based_surface_model body2,
            step_edge_curve_array ecArray)
    {
        step_manifold_solid_brep bpnt1 = this.body;

        bpnt1.ResetFlagFromSequence();
        body2.ResetFlagFromSequence();

        step_closed_shell sh1 = bpnt1.GetOuterShell();
        long maxid = -1;
        int i;
        int total;
        step_advanced_face fpnt2;

        maxid = bpnt1.FindMaximumIDNumber(maxid) + 3;
        int nshell = body2.GetSbsmBoundary().Count();
        step_connected_face_set faceset;
        step_open_shell openshell;
        for (i = 0; i < nshell; ++i)
        {
            faceset = body2.GetSbsmBoundary().Get(i);

            openshell = (step_open_shell) faceset;

            step_face_link link2 = openshell.GetCfsFaces();
            step_face temp2;

            step_face_iterator it = new step_face_iterator(link2);
            while (it.HasNext())
            {
                temp2 = it.GetNext();
                fpnt2 = (step_advanced_face) temp2;

                if (fpnt2 != null && fpnt2.EntityFlag() != ENTITY_DELETED)
                {
                    maxid = fpnt2.ResetIDNumberFromSequence(maxid);

                    this.GetSecondBodyEdges(fpnt2, ecArray);
                    sh1.LinkAdvancedFace(fpnt2);
                    fpnt2.SetShell(sh1);
                    fpnt2.Reverse();
                }
                else
                {
                    // body2.DeleteTopo(fpnt);
                }
            }
        }

        int count = ecArray.Count();
        step_vertex_point vpnt1;
        step_vertex_point vpnt2;
        step_edge_curve ec;
        boolean report;
        step_vertex_point_array varray = new step_vertex_point_array();
        total = 0;
        for (i = 0; i < count; ++i)
        {
            ec = ecArray.Get(i);
            vpnt1 = ec.GetEdgeStart();
            vpnt2 = ec.GetEdgeEnd();

            report = varray.CheckExisted(vpnt1);
            if (report == false)
            {
                varray.Set(total, vpnt1);
                ++total;
            }
            report = varray.CheckExisted(vpnt2);
            if (report == false)
            {
                varray.Set(total, vpnt2);
                ++total;
            }
        }
    }  /*
     * end of procedure body_plane_split::HashLaminaTopology
     */


    private void GetSecondBodyEdges(step_advanced_face adv, step_edge_curve_array earray)
    {
        step_face_bound_link fblink = adv.GetBounds();
        step_face_bound tempface;
        int ecount = 0;
        boolean ans = false;
        step_face_bound_iterator fbit = new step_face_bound_iterator(fblink);
        while (fbit.HasNext())
        {
            tempface = fbit.GetNext();

            step_path lp = tempface.GetBound();
            if (lp.Type() != VERTEX_LOOP)
            {
                step_edge_loop el = (step_edge_loop) lp;
                step_oriented_edge_link orilink = el.GetEdgeList();

                step_edge_curve epnt;
                step_oriented_edge temp;
                step_oriented_edge_iterator oriit = new step_oriented_edge_iterator(orilink);
                while (oriit.HasNext())
                {
                    temp = oriit.GetNext();
                    epnt = temp.GetEdgeElement();

                    if (epnt == null)
                    {
                        continue;
                    }

                    if (epnt != null && epnt.GetSameEdge() != null)
                    {
                        ans = earray.CheckExisted(epnt);

                        if (ans == false)
                        {
                            ecount = earray.Count();
                            earray.Set(ecount, epnt);
                        }
                    }
                }
            }
        }
    }  /*
     * end of procedure body_plane_split::MergeEdge
     */


    private void MergeEdge(step_edge_curve_array earray)
    {
        int edgenum = earray.Count();
        step_edge_curve epnt;
        for (int i = 0; i < edgenum; ++i)
        {
            epnt = earray.Get(i);
            this.MergeEdge(epnt);
        }
    }  /*
     * end of procedure body_plane_split::MergeEdge
     */


    private void MergeEdge(step_edge_curve epnt)
    {
        step_edge_curve keepedge = epnt.GetSameEdge();

        if (keepedge == null)
        {
            return;
        }

        epnt.SetEntityFlag(ENTITY_DELETED);
        // epnt.GetBody().DeleteDependent( epnt );

        step_vertex_point startvpnt1 = keepedge.GetEdgeStart();
        step_vertex_point endvpnt1 = keepedge.GetEdgeEnd();

        step_vertex_point startvpnt2 = epnt.GetEdgeStart();
        step_vertex_point endvpnt2 = epnt.GetEdgeEnd();

        boolean found = true;
        if (startvpnt1.GetSameVertex() == startvpnt2 && endvpnt1.GetSameVertex() == endvpnt2)
        {
            found = true;
        }
        else if (startvpnt1.GetSameVertex() == endvpnt2 && endvpnt1.GetSameVertex() == startvpnt2)
        {
            found = false;
        }

        step_oriented_edge_array orientedArray = new step_oriented_edge_array();
        this.body.GetAdjacentOrientedEdges(epnt, orientedArray);

        int orinum = orientedArray.Count();
        step_oriented_edge oriedge;
        boolean orient1;
        int count;
        boolean ans;
        for (int i = 0; i < orinum; ++i)
        {
            oriedge = orientedArray.Get(i);

            if (oriedge.EntityFlag() != ENTITY_DELETED)
            {
                oriedge.SetEdgeElement(keepedge);

                orient1 = oriedge.GetOrientation();
                if (found == false)
                {
                    if (orient1 == true)
                    {
                        oriedge.SetOrientation(false);
                    }
                    else
                    {
                        oriedge.SetOrientation(true);
                    }
                }
            }
        }

        this.MergeVertex(startvpnt2, endvpnt2);
    }  /*
     * end of procedure body_plane_split::MergeEdge
     */


    private void MergeVertex(step_vertex_point startvpnt2, step_vertex_point endvpnt2)
    {
        step_edge_curve ec;
        step_edge_curve_array edgeArray = new step_edge_curve_array();
        this.body.GetAdjacentEdgeCurves(startvpnt2, edgeArray);

        int ecount = edgeArray.Count();
        int i;
        for (i = 0; i < ecount; ++i)
        {
            ec = edgeArray.Get(i);

            if (ec.GetSameEdge() != null)
            {
                continue;
            }
            if (ec.EntityFlag() == ENTITY_DELETED)
            {
                continue;
            }

            step_vertex_point startvertex = ec.GetEdgeStart();
            if (startvertex == startvpnt2)
            {
                ec.SetEdgeStart(startvpnt2.GetSameVertex());
            }
            else if (startvertex == endvpnt2)
            {
                ec.SetEdgeStart(endvpnt2.GetSameVertex());
            }

            step_vertex_point endvertex = ec.GetEdgeEnd();
            if (endvertex == startvpnt2)
            {
                ec.SetEdgeEnd(startvpnt2.GetSameVertex());
            }
            else if (endvertex == endvpnt2)
            {
                ec.SetEdgeEnd(endvpnt2.GetSameVertex());
            }
        }

        startvpnt2.SetEntityFlag(ENTITY_DELETED);
        // body.DeleteDependent( startvpnt2 );

        edgeArray = new step_edge_curve_array();
        this.body.GetAdjacentEdgeCurves(endvpnt2, edgeArray);

        ecount = edgeArray.Count();
        for (i = 0; i < ecount; ++i)
        {
            ec = edgeArray.Get(i);

            if (ec.GetSameEdge() != null)
            {
                continue;
            }
            if (ec.EntityFlag() == ENTITY_DELETED)
            {
                continue;
            }

            step_vertex_point startvertex = ec.GetEdgeStart();
            if (startvertex == startvpnt2)
            {
                ec.SetEdgeStart(startvpnt2.GetSameVertex());
            }
            else if (startvertex == endvpnt2)
            {
                ec.SetEdgeStart(endvpnt2.GetSameVertex());
            }

            step_vertex_point endvertex = ec.GetEdgeEnd();
            if (endvertex == startvpnt2)
            {
                ec.SetEdgeEnd(startvpnt2.GetSameVertex());
            }
            else if (endvertex == endvpnt2)
            {
                ec.SetEdgeEnd(endvpnt2.GetSameVertex());
            }
        }

        endvpnt2.SetEntityFlag(ENTITY_DELETED);
        // body.DeleteDependent( endvpnt2 );
    }  /*
     * end of procedure body_plane_split::MergeVertex
     */


    private void JoinFace()
    {
        step_edge_curve_array edgeArray = new step_edge_curve_array();
        this.body.GetEdgeCurves(edgeArray);

        step_edge_curve edge;
        int i;
        int count = edgeArray.Count();
        for (i = 0; i < count; i++)
        {
            edge = edgeArray.Get(i);
            if (edge != null)
            {
                edge.SetTraversal((short) 0);
            }
        }

        step_face_array faceArray = new step_face_array();
        for (i = 0; i < count; i++)
        {
            edge = edgeArray.Get(i);
            if (edge != null)
            {
                if (edge.Traversal() == 0)
                {
                    faceArray.SetCount(0);
                    this.body.GetAdjacentFaces(edge, faceArray);
                    boolean ans = edge.JoinFaceCriteria(faceArray);

                    if (ans == true)
                    {
                        step_advanced_face face1 = (step_advanced_face) faceArray.Get(0);
                        step_advanced_face face2 = (step_advanced_face) faceArray.Get(1);
                        face1.JoinFace(face2);
                    }
                }
            }
        }

        // body.DeleteSelectedTopo( ENTITY_DELETED );
        // body.DeleteSelectedGeom( ENTITY_DELETED );
    }  /*
     * end of procedure body_plane_split::JoinFace
     */


    private void JoinEdge()
    {
        step_vertex_point_array vertexArray = new step_vertex_point_array();
        this.body.GetVertices(vertexArray);

        step_manifold_solid_brep body1 = this.body;
        step_vertex_point vpnt;

        for (int i = 0; i < vertexArray.Count(); i++)
        {
            vpnt = vertexArray.Get(i);
            if (vpnt != null)
            {
                step_edge_curve_array edgeArray = new step_edge_curve_array();
                step_face_array faceArray = new step_face_array();

                this.body.GetAdjacentEdgeCurvesAndFaces(vpnt, edgeArray, faceArray);

                boolean ans = vpnt.JoinEdgeCriteria(edgeArray);

                if (ans == true)
                {
                    step_edge_curve epnt1 = edgeArray.Get(0);
                    step_edge_curve epnt2 = edgeArray.Get(1);
                    step_advanced_face adv = (step_advanced_face) faceArray.Get(0);

                    epnt1.JoinEdge(adv, vpnt, epnt2);
                }
            }
        }

        // this.body.DeleteSelectedTopo( ENTITY_DELETED );
        // this.body.DeleteSelectedGeom( ENTITY_DELETED );
    }  /*
     * end of the procedure body_plane_split::JoinEdge
     */


    private void CheckFaceOrientation(step_shell_based_surface_model body2,
            math_vector3d normal)
    {
        step_face_bound_array fbarray = new step_face_bound_array();

        // please check here
        // body2.ResetFlagFromSequence();
        // int count1 = 54;
        // body2.ResetIDNumberFromSequence( count1 );
        // body2.ResetFlagFromSequence();
        // String name = "CutInitial0.stp";
        // body2.WriteToSurfaceModel( name );

        body2.CheckCorrectness();

        body2.GetFaceBounds(fbarray);

        int looptotal = fbarray.Count();
        step_face_bound loop;
        double s1;
        for (int i = 0; i < looptotal; ++i)
        {
            loop = fbarray.Get(i);
            loop.SetLoopNormal();
            s1 = loop.Normal().DotProduct(normal);
            if (s1 < 0.0 && (s1 + 1.0) < EPSI4)
            {
                loop.Reverse();
            }
        }

        // body2.ResetFlagFromSequence();
        // count1 = 54;
        // body2.ResetIDNumberFromSequence( count1 );
        // body2.ResetFlagFromSequence();
        // name = "CutInitial4.stp";
        // body2.WriteToSurfaceModel( name );
    }  /*
     * end of procedure step_body_plane_split::ReorderNewFace
     */


    public step_shell_based_surface_model PlaneSplitWithRawStock(step_advanced_face cutface)
    {
        step_plane supnt = this.cutPlane;
        step_manifold_solid_brep body1 = this.body;

        supnt.SetPlaneBox();
        supnt.SetParamBound();

        step_shell_based_surface_model body2;
        body2 = this.PlaneSplitWithRawStockAndNoSetBox(cutface);

        return body2;
    }  /*
     * end of procedure body_plane_split::PlaneSplit
     */


    public step_shell_based_surface_model PlaneSplitWithoutSetBox(step_manifold_solid_brep body1,
            step_plane supnt,
            step_advanced_face cutface)
    {
        math_vector3d normal = supnt.AxisVector();

        step_shell_based_surface_model body2 = new step_shell_based_surface_model();

        this.VertexPlaneIntersect(body2, cutface);

        math_intercurve_link intercurves = new math_intercurve_link();
        this.EdgePlaneIntersect(body2, intercurves);

        short report = this.FacePlaneIntersect(body2, intercurves);

        if (this.output == 1)
        {
            body1.ResetFlagFromSequence();
            body1.WriteToSolidModel("split01.stp");
            body2.ResetFlagFromSequence();
            body2.WriteToSurfaceModel("split02.stp");
        }

        if (report == 0)
        {
            return body2;
        }

        this.CheckFaceOrientation(body2, normal);

        if (this.output == 1)
        {
            body1.ResetFlagFromSequence();
            body1.WriteToSolidModel("split07.stp");
            body2.ResetFlagFromSequence();
            body2.ResetIDNumberFromSequence(54);
            body2.ResetFlagFromSequence();
            body2.WriteToSurfaceModel("PlaneSplitWithoutSetBox.stp");
        }
        return body2;
    }  /*
     * end of procedure body_plane_split::PlaneSplit
     */


    public step_shell_based_surface_model PlaneSplitWithRawStockAndNoSetBox(step_advanced_face cutface)
    {
        step_shell_based_surface_model body2 = new step_shell_based_surface_model();
        step_manifold_solid_brep body1 = this.body;
        step_plane supnt = this.cutPlane;

        math_vector3d normal = supnt.AxisVector();

        body1.SetBox();

        this.VertexPlaneIntersect(body2, cutface);

        math_intercurve_link intercurves = new math_intercurve_link();
        this.EdgePlaneIntersect(body2, intercurves);

        short report = this.FacePlaneIntersect(body2, intercurves);
        if (report == 0)
        {
            return body2;
        }

// intercurves.WriteData("AAA111.txt");

        step_face_bound_array looparray = new step_face_bound_array();
        if (Math.abs(this.stockTolerance) > EPSI4)
        {
            this.FindRawStockShape(looparray);
        }

        this.CheckFaceOrientation(body2, normal);

        body2.CheckCorrectness();
        body2.CheckEdgeCurveOrientation();

// body2.ResetFlagFromSequence();
// int count = 54;
// body2.ResetIDNumberFromSequence(count);
// body2.ResetFlagFromSequence();
// body2.WriteSurfaceModel("split.stp");

        return body2;
    }  /*
     * end of procedure body_plane_split::PlaneSplitWithRawStockAndNoSetBox
     */


    private void FindRawStockShape(step_face_bound_array looparray)
    {
        step_manifold_solid_brep body1 = this.body;
        step_plane supnt = this.cutPlane;

        double toler = this.stockTolerance;
        math_vector3f allow = new math_vector3f((float) toler, (float) toler, (float) toler);
        math_vector3f bmax = body1.Mmax().Add(allow);
        math_vector3f bmin = body1.Mmin().Subtract(allow);
        double xmax = bmax.X();
        double xmin = bmin.X();
        double ymax = bmax.Y();
        double ymin = bmin.Y();
        double zmax = bmax.Z();
        double zmin = bmin.Z();

        math_vector3d coor1 = new math_vector3d(xmax, ymax, zmax);
        math_vector3d coor2 = new math_vector3d(xmax, ymax, zmin);
        math_vector3d coor3 = new math_vector3d(xmin, ymax, zmin);
        math_vector3d coor4 = new math_vector3d(xmin, ymax, zmax);
        math_vector3d coor5 = new math_vector3d(xmax, ymin, zmax);
        math_vector3d coor6 = new math_vector3d(xmax, ymin, zmin);
        math_vector3d coor7 = new math_vector3d(xmin, ymin, zmin);
        math_vector3d coor8 = new math_vector3d(xmin, ymin, zmax);

        step_line line[] = new step_line[12];
        line[0] = new step_line(coor1, coor2);
        line[1] = new step_line(coor2, coor3);
        line[2] = new step_line(coor3, coor4);
        line[3] = new step_line(coor4, coor1);
        line[4] = new step_line(coor5, coor6);
        line[5] = new step_line(coor6, coor7);
        line[6] = new step_line(coor7, coor8);
        line[7] = new step_line(coor8, coor5);
        line[8] = new step_line(coor1, coor5);
        line[9] = new step_line(coor2, coor6);
        line[10] = new step_line(coor3, coor7);
        line[11] = new step_line(coor4, coor8);

        step_curve_array carray = new step_curve_array();
        step_curve newcpnt;
        step_line cpnt;
        int j;
        int curvecount;
        int found;
        int ans;
        step_curve curcpnt;
        boolean result;
        for (int i = 0; i < 12; ++i)
        {
            cpnt = line[i];
            newcpnt = cpnt.ProjectToPlane(supnt);
            if (newcpnt != null)
            {
                result = carray.CheckExisted(newcpnt);
                if (result == false)
                {
                    j = 0;
                    found = 0;
                    curvecount = carray.Count();
                    while (j < curvecount && found == 0)
                    {
                        curcpnt = carray.Get(j);
                        ans = curcpnt.Compare(newcpnt);
                        if (Math.abs(ans) > 0)
                        {
                            found = 1;
                        }

                        ++j;
                    }
                    if (found == 0)
                    {
                        carray.Set(curvecount, newcpnt);
                    }
                }
            }
        }

        oriented_edge_traversal_array ecarray = new oriented_edge_traversal_array();
        ecarray.SetFromCurveArray(carray);
        ecarray.SearchBranchLoops(supnt, looparray);

        ecarray = null;
    }  /*
     * end of procedure body_plane_split::FindRawStockShape
     */


    private void FirstFaceSplit(step_advanced_face fpnt1,
            step_curve cpnt1,
            step_vertex_point pvert,
            step_vertex_point nvert,
            step_advanced_face newfpnt1[],
            step_edge_curve newepnt1[])
    {
        step_face_bound newlpnt[] = new step_face_bound[1];

        boolean ans = fpnt1.CurveIsSameWithEdgeGeometry(cpnt1);
        if (ans == true)
        {
            return;
        }

        step_manifold_solid_brep body = fpnt1.GetBody();
        body.ME(fpnt1, pvert, nvert, cpnt1, newepnt1, newfpnt1, newlpnt);
        fpnt1.SetIntersectType(true);
    }  /*
     * end of procedure body_plane_split::FirstFaceSplit
     */


    private void SecondFaceSplit(step_advanced_face fpnt2,
            step_curve cpnt2,
            step_vertex_point pvert,
            step_vertex_point nvert,
            step_advanced_face newfpnt2[],
            step_edge_curve newepnt2[])
    {
        step_face_bound newlpnt[] = new step_face_bound[1];

        boolean ans = fpnt2.CurveIsSameWithEdgeGeometry(cpnt2);
        if (ans == true)
        {
            return;
        }

        step_shell_based_surface_model body = fpnt2.GetSurfaceModel();
        body.ME(fpnt2, pvert, nvert, cpnt2, newepnt2, newfpnt2, newlpnt);

        fpnt2.SetIntersectType(true);
    }  /*
     * end of procedure body_plane_split::SecondFaceSplit
     */


    private void CreateCrossEdge(step_edge_curve newepnt1[], step_edge_curve newepnt2[],
            math_intercurve crosscurve)
    {
        newepnt1[0].AssignSameEdge(newepnt2[0]);
        crosscurve.SetCurve(null);  // prevent deleting cpnt1
    }  /*
     * end of procedure body_plane_split::CreateCrossEdge
     */

}
